#!/usr/bin/env python3
"""跳跃强化学习环境(MuJoCo 直接力矩控制 + Gymnasium)

架构: 三环负责稳定, RL 只做短窗口"跳跃爆发"(分阶段):
      - 训练前由真实 C 三环(ident_bridge)预热 1~1.5s, 稳定到部署状态。
      - RL 接管固定 2s 爆发窗口, 动作用"虚拟坐标力矩":
          [腿长力 τ_L, 腿摆力 τ_sw, 左右腿长差 τ_diff, 左轮, 右轮]
        环境经运动学 Jᵀ 映射到 4 个腿主关节(不独立驱动关节,
        不破坏连杆约束), 轮子直接力矩。
      - 分阶段控制:
          地面接触: 用腿长发力爆发起跳(跳跃只发生在接触阶段);
          空中    : 动腿摆 + 轮子, 只调落地姿态, 方便恢复平衡;
          落地    : 姿态正给高分(恢复平衡是底线), 之后交还三环。
      - 观测含"离地标志"(最后一维), 让策略明确知道当前阶段。

动作(5维): [tau_L, tau_sw, tau_diff, lw_ctrl, rw_ctrl]  [-1,1]
  - tau_L   : 腿长方向虚拟力矩 ±60 Nm (负=伸腿蹬地/起跳)
  - tau_sw  : 腿摆方向虚拟力矩 ±40 Nm (负=向后摆)
  - tau_diff: 左右腿长差虚拟力矩 ±40 Nm (roll 姿态)
  - 轮      : ctrl = a*10 (10:1 减速, 每轮 ±100 Nm 轮上扭矩)
  - 腿部分经运动学 Jᵀ 映射到 4 个主关节(不独立驱动关节, 不破坏连杆约束)

观测(34维), 与 C 侧 rl_jump_controller.c::build_obs 严格一致:
   0,1     pitch, roll(rad)
   2,3     pitch_rate, roll_rate(世界系角速度, 已核实 == IMU gyro)
   4..6    腿虚拟坐标: 腿摆 sw, 左腿长 L, 右腿长 R
           (由 4 个主关节角经线性运动学推导, 与平衡控制器同一套映射,
            等于把"关节电机↔腿长/腿摆"关系直接告诉模型)
   7..10   四腿关节速度 lf, lr, rf, rr
   11,12   左右轮速
   13      机体世界 z
   14,15   |水平速度 vx|, |vy|
   16,17   pitch, roll(deg)
   18      z - 0.085(相对名义高度增量)
   19      时间戳(回合内相对时间 s)
   20..22  机体世界线速度 vx, vy, vz
   23,24   左右轮位置
   25,26   机体世界位置 x, y
   27..32  执行器"电流"(电机侧力矩): lf, lr, lw, rf, rr, rw
   33      离地标志(0=接触地面, 1=空中), 分阶段控制的阶段信号

初始姿态采用模型默认位(腿折叠, 轮子着地, 脚离地):
lf=0, lr=0, rf=-0.1, rr=0(虚拟坐标 sw≈0.2, L≈0.2, R≈0.2) + z=0.08。
零动作下自然稳定(2s 不倒), 是 RL 最容易学平衡/跳跃的起点;
实测位(脚撑地, L≈-0.10)零动作 0.04s 即倒, 不适合做训练起点。
部署侧 RL 接管前由 PREP 把腿收到脚离地, 与该起点一致。

奖励: 高度为主 + 站立底线 + 跳过后的恢复平衡(站稳+6/歪-2) + 落地软
      - 高度底线 min_height 达不到 -> 回合结束重罚 -100。
"""
import os
import math
import ctypes
import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XML = os.path.join(ROOT, "model", "COD-2026RoboMaster-Balance.xml")

SW_NOM = 0.23          # 名义腿摆(仅用于参考, 新策略直接出力矩, 不再用位置伺服)
NOM_HEIGHT = 0.085     # 观测相对高度基准
TAU_L_SCALE = 150.0    # 腿长虚拟力矩量程 Nm(降低起跳力度, 高度可控防落地翻)
TAU_SW_SCALE = 60.0    # 腿摆虚拟力矩量程 Nm
TAU_DIFF_SCALE = 60.0  # 左右腿长差虚拟力矩量程 Nm
LEG_GAIN = 50.0        # 腿执行器 gainprm
# 电流通道归一化量程(电机侧力矩 Nm): 腿 ±250, 轮 ±100
OBS_CUR_SCALE = np.array([250.0, 250.0, 100.0, 250.0, 250.0, 100.0])
BRIDGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ident_bridge.so")
WARMUP_RANGE = (1.0, 1.5)   # 三环预热时长 s(只需稳定到平衡态,
                            #  长时间漂移状态由 _randomize_takeover 注入,
                            #  训练速度大幅提升)
EPISODE_S = 3.0             # RL 全程窗口时长 s(起跳+空中+落地+恢复)
RETRACT_S = 0.25            # 收腿到最短的 leg_extend_manual 指令
RETRACT_DONE_L = 0.10       # 腿收到 L>=0.10 视为"最短腿"


class IdentFrame(ctypes.Structure):
    """与 ident_bridge.c 的 IdentFrame 一致。"""
    _fields_ = [
        ("pitch", ctypes.c_float), ("pitch_rate", ctypes.c_float),
        ("roll", ctypes.c_float), ("roll_rate", ctypes.c_float),
        ("left_wheel_vel", ctypes.c_float), ("right_wheel_vel", ctypes.c_float),
        ("acc_mag", ctypes.c_float), ("acc_x", ctypes.c_float), ("acc_z", ctypes.c_float),
        ("yaw", ctypes.c_float), ("yaw_rate", ctypes.c_float),
        ("left_wheel_pos", ctypes.c_float), ("right_wheel_pos", ctypes.c_float),
        ("lf_pos", ctypes.c_float), ("lr_pos", ctypes.c_float),
        ("rf_pos", ctypes.c_float), ("rr_pos", ctypes.c_float),
        ("lf_tau", ctypes.c_float), ("lr_tau", ctypes.c_float),
        ("rf_tau", ctypes.c_float), ("rr_tau", ctypes.c_float),
        ("left_wheel_tau", ctypes.c_float), ("right_wheel_tau", ctypes.c_float),
        ("lf_vel", ctypes.c_float), ("lr_vel", ctypes.c_float),
        ("rf_vel", ctypes.c_float), ("rr_vel", ctypes.c_float),
        ("acc_y", ctypes.c_float),
        ("quat", ctypes.c_double * 4),
        ("body_pos", ctypes.c_double * 3),
        ("body_vel", ctypes.c_double * 3),
        ("sim_time", ctypes.c_double),
        ("step_count", ctypes.c_int),
    ]


class JumpRLEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, timestep=0.001, episode_steps=None, min_height=0.15):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path(XML)
        self.model.opt.timestep = timestep
        self.data = mujoco.MjData(self.model)
        self.episode_steps = (episode_steps if episode_steps is not None
                              else int(EPISODE_S / timestep))
        self.min_height = min_height

        # 动作: 5 维 [腿长力, 腿摆力, 左右差, 左轮, 右轮] (归一化 [-1,1])
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(5,), dtype=np.float32)
        # 观测: 34 维(33 + 离地标志, 与 C 侧 build_obs 一致)
        self.obs_dim = 34
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(self.obs_dim,), dtype=np.float32)

        self.step_count = 0
        self.max_height = 0.0
        self.reached_min = False
        self.fallen = False
        self._act_filt = np.zeros(5)   # 动作低通, 防扭矩抖振
        # 训练课程: RL_CURRICULUM=balance -> 只学站立(奖励只有站立/姿态/软腿);
        #           默认 full -> 站立+跳跃完整奖励。
        self.curriculum = os.environ.get("RL_CURRICULUM", "full")

        # 关节地址
        self.leg_names = ["Left_front_joint", "Left_rear_joint",
                          "Right_front_joint", "Right_rear_joint"]
        self.leg_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                        for n in self.leg_names]
        self.leg_qpos = [self.model.jnt_qposadr[j] for j in self.leg_ids]
        self.leg_dof = [self.model.jnt_dofadr[j] for j in self.leg_ids]
        self.lw_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "Left_Wheel_joint")
        self.rw_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "Right_Wheel_joint")
        self.lw_qpos = self.model.jnt_qposadr[self.lw_id]
        self.rw_qpos = self.model.jnt_qposadr[self.rw_id]
        self.lw_dof = self.model.jnt_dofadr[self.lw_id]
        self.rw_dof = self.model.jnt_dofadr[self.rw_id]
        # 模型默认位(腿折叠/轮着地): 零动作自然稳定, 训练/跳跃最优起点
        self._home_qpos = self.data.qpos.copy()
        # 三环预热桥(与部署同一套 C 三环+腿伺服+轮混控)
        self._bridge = None
        if os.path.exists(BRIDGE):
            self._bridge = ctypes.CDLL(BRIDGE)
            self._bridge.ident_init()
            self._bridge.ident_reset()
            self._bridge.ident_robot_step.argtypes = [
                ctypes.POINTER(IdentFrame), ctypes.c_double,
                ctypes.POINTER(ctypes.c_float * 6)]
            self._bridge.ident_warmup_step.argtypes = [
                ctypes.POINTER(IdentFrame), ctypes.c_double,
                ctypes.POINTER(ctypes.c_float * 6)]
            self._bridge.ident_set_leg_len_direct.argtypes = [ctypes.c_float]
        self._setup_initial()

    def _fill_frame(self):
        d = self.data
        if not hasattr(self, "_warm_steps"):
            self._warm_steps = 0
        quat = d.sensordata[0:4]
        w, x, y, z = quat
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - x * z))))
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return IdentFrame(
            pitch=pitch, pitch_rate=float(d.sensordata[5]),
            roll=roll, roll_rate=float(d.sensordata[4]),
            left_wheel_vel=float(d.qvel[self.lw_dof]),
            right_wheel_vel=float(d.qvel[self.rw_dof]),
            acc_mag=float(np.sqrt(d.sensordata[7] ** 2 + d.sensordata[9] ** 2)),
            acc_x=float(d.sensordata[7]), acc_z=float(d.sensordata[9]),
            yaw=yaw, yaw_rate=float(d.sensordata[6]),
            left_wheel_pos=float(d.qpos[self.lw_qpos]),
            right_wheel_pos=float(d.qpos[self.rw_qpos]),
            lf_pos=float(d.qpos[self.leg_qpos[0]]), lr_pos=float(d.qpos[self.leg_qpos[1]]),
            rf_pos=float(d.qpos[self.leg_qpos[2]]), rr_pos=float(d.qpos[self.leg_qpos[3]]),
            lf_tau=float(d.actuator_force[0]), lr_tau=float(d.actuator_force[1]),
            rf_tau=float(d.actuator_force[3]), rr_tau=float(d.actuator_force[4]),
            left_wheel_tau=float(d.actuator_force[2] * 10.0),
            right_wheel_tau=float(d.actuator_force[5] * 10.0),
            lf_vel=float(d.qvel[self.leg_dof[0]]), lr_vel=float(d.qvel[self.leg_dof[1]]),
            rf_vel=float(d.qvel[self.leg_dof[2]]), rr_vel=float(d.qvel[self.leg_dof[3]]),
            acc_y=float(d.sensordata[8]),
            quat=(ctypes.c_double * 4)(*d.sensordata[0:4]),
            body_pos=(ctypes.c_double * 3)(*d.qpos[0:3]),
            body_vel=(ctypes.c_double * 3)(*d.qvel[0:3]),
            sim_time=self._warm_steps * self.model.opt.timestep,
            step_count=self._warm_steps,
        )

    def _step_warmup(self):
        """一步部署控制预热: 真实 robot_control_step, 与部署完全一致。"""
        fr = self._fill_frame()
        ctrl = (ctypes.c_float * 6)()
        self._bridge.ident_robot_step(ctypes.byref(fr),
                                      self.model.opt.timestep, ctrl)
        self.data.ctrl[:] = [ctrl[j] for j in range(6)]
        mujoco.mj_step(self.model, self.data)
        self._warm_steps += 1

    def _randomize_takeover(self):
        """域随机化"接管状态": 覆盖部署时三环预热后的各种漂移/姿态偏差,
        让 RL 学会从任意近平衡状态接管并完成跳跃。"""
        r = self.np_random
        d = self.data
        d.qpos[2] = r.uniform(0.07, 0.10)                 # 机体高度
        euler = r.uniform(-0.10, 0.10, 3)                  # 姿态扰动(含偏航, ±6°)
        q = np.zeros(4)
        mujoco.mju_euler2Quat(q, euler, 'xyz')
        d.qpos[3:7] = q
        d.qpos[0] = r.uniform(-1.0, 1.0)                   # 世界位置 x,y(漂移)
        d.qpos[1] = r.uniform(-1.0, 1.0)
        # 腿位: 围绕默认位(脚离地/轮着地)小扰动, L∈[0.12,0.28]
        leg_bounds = [(-0.06, 0.08), (-0.06, 0.08),       # lf, lr
                      (-0.16, -0.04), (-0.06, 0.06)]      # rf, rr
        for adr, (lo, hi) in zip(self.leg_qpos, leg_bounds):
            d.qpos[adr] = r.uniform(lo, hi)
        d.qpos[self.lw_qpos] = r.uniform(-15.0, 15.0)      # 轮位置(漂移积分)
        d.qpos[self.rw_qpos] = r.uniform(-15.0, 15.0)
        d.qvel[0:3] = r.uniform(-1.0, 1.0, 3)              # 线速度(含 vz)
        d.qvel[3:6] = r.uniform(-1.5, 1.5, 3)              # 角速度
        d.qvel[self.lw_dof] += r.uniform(-6.0, 6.0)        # 轮速(漂移)
        d.qvel[self.rw_dof] += r.uniform(-6.0, 6.0)
        d.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, d)

    def _setup_initial(self):
        """模型默认位(腿折叠/轮着地) + z=0.08 + 水平。"""
        d = self.data
        d.qpos[:] = self._home_qpos
        d.qpos[2] = 0.08
        d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]       # 水平姿态
        d.qvel[:] = 0.0
        d.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, d)

    def _get_obs(self):
        d = self.data
        quat = d.sensordata[0:4]
        w, x, y, z = quat
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - x * z))))
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        angvel = d.qvel[3:6]
        # 关节电机 -> 腿长/腿摆: 4 个主关节角合成虚拟坐标
        lf, lr, rf, rr = (d.qpos[self.leg_qpos[i]] for i in range(4))
        sw = 0.25 * ((lf - lr + 0.4) + (0.3 - rf + rr))
        leg_l = 0.5 * (0.4 - lf - lr)
        leg_r = 0.5 * (rf + rr + 0.5)
        return np.array([
            pitch, roll,
            angvel[1], angvel[0],                      # pitch_rate, roll_rate
            sw, leg_l, leg_r,                           # 腿虚拟坐标(运动学已告诉模型)
            d.qvel[self.leg_dof[0]], d.qvel[self.leg_dof[1]],
            d.qvel[self.leg_dof[2]], d.qvel[self.leg_dof[3]],
            d.qvel[self.lw_dof], d.qvel[self.rw_dof],
            d.qpos[2],                                  # body z
            abs(d.qvel[0]), abs(d.qvel[1]),             # |vx|, |vy|
            pitch * 57.3, roll * 57.3,
            d.qpos[2] - NOM_HEIGHT,                     # 高度增量
            self.step_count * self.model.opt.timestep,  # 时间戳(回合内)
            d.qvel[0], d.qvel[1], d.qvel[2],            # 世界线速度 vx,vy,vz
            d.qpos[self.lw_qpos], d.qpos[self.rw_qpos], # 轮位置
            d.qpos[0], d.qpos[1],                       # 机体世界位置 x,y
            # 电流(电机侧力矩)按量程归一化, 避免大数值主导网络输入
            d.actuator_force[0] / OBS_CUR_SCALE[0],
            d.actuator_force[1] / OBS_CUR_SCALE[1],
            d.actuator_force[2] / OBS_CUR_SCALE[2],
            d.actuator_force[3] / OBS_CUR_SCALE[3],
            d.actuator_force[4] / OBS_CUR_SCALE[4],
            d.actuator_force[5] / OBS_CUR_SCALE[5],
            1.0 if (d.qpos[2] - NOM_HEIGHT) > 0.02 else 0.0,  # 离地标志
        ], dtype=np.float32)

    def _step_ctrl(self, action):
        """混合控制一步:
        - 腿: RL 输出虚拟坐标力矩(腿长/腿摆/左右差), 经运动学 Jᵀ 映射到
          4 个主关节(不独立驱动关节, 不破坏连杆约束)。
        - 轮子分阶段:
            地面接触: 由真实三环(ident_robot_step)稳住, 防止起跳侧翻,
                       RL 只管"跳跃爆发";
            空中    : 交给 RL(动轮子调落地姿态, 方便恢复平衡)。"""
        a = np.clip(action, -1.0, 1.0)
        tau_L = a[0] * TAU_L_SCALE
        tau_sw = a[1] * TAU_SW_SCALE
        tau_diff = a[2] * TAU_DIFF_SCALE
        tau_Ll = tau_L + 0.5 * tau_diff
        tau_Lr = tau_L - 0.5 * tau_diff
        leg_ctrl = np.array([
            (tau_sw - tau_Ll) / LEG_GAIN,     # lf
            (-tau_sw - tau_Ll) / LEG_GAIN,    # lr
            (-tau_sw + tau_Lr) / LEG_GAIN,    # rf
            (tau_sw + tau_Lr) / LEG_GAIN,     # rr
        ])
        airborne = self._get_obs()[33] > 0.5
        if airborne:
            wheel = np.array([a[3] * 10.0, a[4] * 10.0])   # 空中: RL 控轮
        elif self._bridge is not None:
            fr = self._fill_frame()
            ctrl3 = (ctypes.c_float * 6)()
            # 地面: 纯三环核心稳轮(不含 supervisor 自救, 避免起跳大 pitch
            # 触发自救与 RL 腿动作打架)
            self._bridge.ident_warmup_step(ctypes.byref(fr),
                                           self.model.opt.timestep, ctrl3)
            wheel = np.array([ctrl3[2], ctrl3[5]])          # 地面: 三环稳轮
        else:
            wheel = np.array([0.0, 0.0])
        ctrl = np.array([leg_ctrl[0], leg_ctrl[1], wheel[0],
                         leg_ctrl[2], leg_ctrl[3], wheel[1]])
        self.data.ctrl[:] = np.clip(ctrl, -10.0, 10.0)
        mujoco.mj_step(self.model, self.data)

    def _rew(self, obs):
        # 34 维观测索引: 14,15=|vx|,|vy|; 16,17=pitch°,roll°;
        # 18=高度增量; 22=vz; 33=离地标志
        pitch_deg = obs[16]
        roll_deg = obs[17]
        h = obs[18]
        airborne = obs[33]
        # 高度: 离地给分, 创新高加分, 超过 0.20m 饱和(防超高翻车)
        h_cap = min(max(0.0, h), 0.20)
        r_height = 40.0 * h_cap - 0.5 * max(0.0, -h)   # 下沉轻罚, 允许蓄力
        r_high_pen = -6.0 * max(0.0, h - 0.25)          # 超过 0.25m 强罚防超高
        mh_cap = min(max(0.0, self.max_height), 0.20)
        if h_cap > mh_cap + 1e-4:
            r_height += 4.0 * (h_cap - mh_cap)
        self.max_height = max(self.max_height, h)
        # 分阶段奖励
        # 1) 地面(接触): 起跳高度奖励
        # 2) 空中: 姿态控制奖励(动腿摆+轮子只调落地姿态, 姿态越平越好)
        # 3) 落地: 姿态正=高分, 歪=扣分(恢复平衡是底线)
        r_air_pose = 0.0
        if airborne > 0.5:
            r_air_pose = -1.2 * (abs(pitch_deg) + abs(roll_deg))
        # 别侧翻(用户强调): roll 越大惩罚越重
        r_roll = -3.0 * abs(roll_deg)
        if self.max_height > 0.02 and h < 0.03:
            # 已跳过且回到地面(落地时刻附近): 姿态正=高分, 歪=扣分
            r_land = (6.0 if (abs(pitch_deg) < 20 and abs(roll_deg) < 15)
                      else -3.0)
        else:
            r_land = 0.0
        # 落地软: 腿执行器力矩小
        leg_tau = sum(abs(self.data.actuator_force[i]) for i in (0, 1, 3, 4))
        r_soft = -0.002 * leg_tau
        # 失稳判定: 侧翻(roll>35°)是底线红线, 重罚;
        # 前后倒(pitch>55°)部署时三环/自救能救回, 轻罚即可。
        r_pen = 0.0
        if abs(roll_deg) > 35:
            r_pen = -50.0
            self.fallen = True
            self.side_flip = True
        elif abs(pitch_deg) > 55:
            r_pen = -10.0
            self.fallen = True
            self.side_flip = False
        r_pen -= 0.03 * (abs(obs[20]) + abs(obs[21]))  # 起跳水平漂移轻罚
        return r_height + r_high_pen + r_air_pose + r_roll + r_land + r_soft + r_pen

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # 专家混合: 一半回合前半段(0.6s)用手工起跳轨迹演示,
        # 让 PPO 观察到"先收腿再蹬腿"的成功时序(直接学精确爆发时序难收敛)。
        self.expert_steps = (1000 if self.np_random.uniform() < 0.5 else 0)
        self._setup_initial()
        # 三环预热: 随机 1.0~2.5s(部署时任意时刻按空格), 让车稳定到真实稳态
        # 后由 RL 接管。阶段一(RL_NO_RANDOMIZE=1)也走预热(与部署一致)。
        if self._bridge is not None:
            # 预热 1~1.5s(当前三环状态: 姿态稳定 ±2°, 少量漂移),
            # 接管状态即真实部署分布, 不做人造冻结。
            for _attempt in range(4):
                self._bridge.ident_reset()   # 每回合清空控制器状态
                self._setup_initial()
                n = int(self.np_random.uniform(*WARMUP_RANGE) /
                        self.model.opt.timestep)
                self._warm_steps = 0
                for _ in range(n):
                    self._step_warmup()
                obs_tmp = self._get_obs()
                # 接管过滤: 姿态 + 轮速合格才接受(丢弃收腿阶段翻车/轮速爆炸的回合)
                w_max = max(abs(obs_tmp[11]), abs(obs_tmp[12]))
                if (abs(obs_tmp[16]) < 10.0 and abs(obs_tmp[17]) < 8.0
                        and w_max < 20.0):
                    break
            # 注入长时间漂移后的状态(位置/轮位置/轮速/姿态), 覆盖部署时
            # 用户等很久才按空格的真实分布, 替代长预热, 加速训练。
            self._randomize_takeover()
            # 收腿到最短: 通过三环 leg_extend_manual 极限收腿,
            # RL 统一从"最短腿"状态起跳(用户指定: 初始腿长不用管)。
            self._bridge.ident_set_leg_len_direct(RETRACT_S)
            for _ in range(int(1.5 / self.model.opt.timestep)):
                self._step_warmup()
                if self._get_obs()[5] > RETRACT_DONE_L:
                    break
            self._bridge.ident_set_leg_len_direct(0.0)
        else:
            self._randomize_takeover()
        self.step_count = 0
        self.max_height = 0.0
        self.reached_min = False
        self.fallen = False
        self.side_flip = False
        self._act_filt = np.zeros(5)
        return self._get_obs(), {}

    def step(self, action):
        # 动作低通 0.7/0.3: 直接力矩输出更平滑, 减少抖振
        a = np.clip(action, -1.0, 1.0)
        if self.step_count < self.expert_steps:
            # 专家起跳时序(手工验证过, max roll 1.7°):
            # 0.3~0.5s 收腿蓄力, 0.5~0.9s 猛蹬起跳;
            # 轮子 0(地面由三环稳, 空中由 RL 接管后的轮子动作学)
            t = self.step_count * self.model.opt.timestep
            a = np.zeros(5, dtype=np.float32)
            if 0.3 <= t < 0.5:
                a[0] = 0.8
            elif 0.5 <= t < 0.9:
                a[0] = -1.0
        self._act_filt = 0.5 * self._act_filt + 0.5 * a   # 低通加快, 起跳响应快
        self._step_ctrl(self._act_filt)
        self.step_count += 1
        obs = self._get_obs()
        r = self._rew(obs)
        if not self.reached_min and self.max_height >= self.min_height:
            self.reached_min = True
            r += 30.0
        terminated = self.fallen
        truncated = self.step_count >= self.episode_steps
        if terminated:
            if self.side_flip:
                r += -100.0          # 侧翻: 红线, 重罚
            else:
                r += -10.0           # 前后倒: 三环可救回, 轻罚
        elif truncated:
            # 爆发窗口结束: 跳够底线给分, 没跳够扣分(落点姿态三环接管)
            r += 10.0 if self.reached_min else -50.0
        info = {
            "height": self.max_height,
            "reached_min": self.reached_min,
            "pitch_deg": obs[16],
            "roll_deg": obs[17],
        }
        return obs, r, terminated, truncated, info


if __name__ == "__main__":
    env = JumpRLEnv()
    obs, _ = env.reset()
    print("obs dim:", len(obs), "action dim:", env.action_space.shape[0])
    print("obs:", np.round(obs, 3))
    for i in range(100):
        obs, r, term, trunc, info = env.step(np.zeros(5))
        if i % 20 == 0:
            print(f"t={i*env.model.opt.timestep:.2f} pitch={info['pitch_deg']:+.1f} "
                  f"roll={info['roll_deg']:+.1f} h={info['height']:.3f} r={r:+.2f}")
        if term or trunc:
            break
