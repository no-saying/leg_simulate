# leg_simulate - 轮腿自平衡机器人仿真框架

基于 **MuJoCo (MJCF)** 的轮腿（Wheel-Leg）自平衡机器人高保真仿真与控制框架。

## 项目概览

本项目仿真一台带**串联腿闭链机构**的轮腿自平衡车（v3 模型）：左右各一条
“大腿 + 五连杆闭链”腿机构（每侧 2 个驱动关节：claf0 + 大腿，被动关节由
equality connect 约束闭合），轮子位于闭链末端。整车依靠俯仰（pitch）方向的
轮扭矩平衡，通过左右差速实现转向，通过腿长/腿摆虚拟坐标实现升降与姿态控制。
控制器使用真实机器人可获得的传感器量（IMU、轮编码器、腿编码器、执行器力矩），
不依赖仿真世界坐标。

跳跃由 **强化学习（PPO）** 控制：策略输出“腿长 / 腿摆 / 左右腿差”三个虚拟坐标力矩，
经运动学 Jᵀ 映射到四个腿主关节（不破坏连杆约束），轮子按阶段由三环/RL 接管。

## 目录结构

```
leg_simulate/
├── model/                        # MuJoCo 模型
│   ├── COD-2026RoboMaster-Balance.xml   # 旧四腿模型(备份)
│   └── v3/COD-2026RoboMaster-Balance-v3.xml  # 新串联腿闭链模型(主模型)
│   ├── meshes/                   # STL 网格文件
│   └── terrain_hfield_include.xml # 平滑丘陵/坑洼地形(heightfield)
│
├── control/                      # 控制程序
│   ├── app/                      # 应用层: GUI / 无界面测试 / 键盘 / 仿真步进
│   │   ├── controller.c          # main 入口
│   │   ├── app_sim.c             # 模型加载/复位/单步推进
│   │   ├── app_gui.c             # GLFW 窗口/相机/渲染/HUD
│   │   ├── app_headless.c        # 无界面自动化测试(环境变量驱动)
│   │   └── app_keys.c            # 键盘状态与回调
│   ├── balance/                  # 平衡控制核心(三环)
│   │   ├── balance_core.c        # balance_update 入口(按段落 include)
│   │   ├── balance_step_sensor.inc     # 传感器预处理/滤波/接触状态
│   │   ├── balance_step_origin.inc     # 原点控制/手动驾驶/腿长/roll稳定
│   │   ├── balance_step_pitch_yaw.inc  # 俯仰PD/偏航环/输出
│   │   ├── balance_step_legacy*.inc    # 历史速度环路径(备用)
│   │   ├── balance_util.c        # 通用工具(滤波/限幅/窗口/复位)
│   │   └── balance_api.c         # 对外接口(getter/setter/触发)
│   ├── controllers/              # 上层控制器(模式切换/跳跃)
│   │   ├── lqr/                  # 分段 LQR(小角度稳定区, 叠加扰动)
│   │   ├── energy/               # 能量整形(大角度摆起, 备用)
│   │   ├── supervisor/           # 模式切换状态机(翻倒自救检测, 当前禁用)
│   │   └── rl_jump/              # RL 跳跃控制器(PPO 权重 + 状态机)
│   │       ├── rl_jump_net.h     # 导出的策略权重(勿手改, 由脚本生成)
│   │       ├── rl_jump_net.c     # 纯前向推理
│   │       ├── rl_jump_controller.c/.h  # 状态机/观测/动作映射
│   ├── runtime/                  # 低层执行器桥接(腿/轮 PID、扭矩混控)
│   ├── lowlevel/                 # 腿/轮电机 PID 库
│   ├── sim/                      # 传感器读取、遥测、轨迹绘制
│   ├── dashcam/                  # 行车记录仪(CSV + 事件日志)
│   └── include/                  # 配置参数头(按功能拆分)
│       ├── control_config.h      # 汇总头
│       ├── config_mech.h         # 机械常数/跳跃/自救
│       ├── config_balance.h      # 俯仰/偏航/roll 稳定
│       ├── config_safety.h       # 近原点/远场刹车/轮速保护
│       ├── config_odom.h         # 里程计/速度规划
│       ├── config_manual.h       # 手动驾驶/辨识
│       ├── config_lowlevel.h     # 低层输出
│       └── config_controllers.h  # supervisor/LQR/能量参数
│
├── scripts/                      # 训练/评估/导出/验证
│   ├── jump_rl_env.py            # RL 跳跃训练环境(含部署一致性)
│   ├── train_jump_rl.py          # PPO 训练(mjbatch / 独立进程)
│   ├── jump_rl_batch_env.py      # mjbatch 向量环境, 批量观测/动作/奖励
│   ├── rl_batch_control.py      # 每实例独立 C 控制器及构建缓存
│   ├── eval_jump_rl.py           # 训练结果评估
│   ├── run_jump_rl.py            # Python 渲染回放
│   ├── verify_rl_jump_c.py       # C 前向 vs Python 一致性验证
│   ├── gen_rl_jump_c.py          # PPO 权重 -> C 头文件导出
│   ├── ident_bridge.c            # 训练桥(真实三环控制 + 回合重置)
│   ├── rl_jump_c_test.c          # C 前向独立测试
│   ├── tools/                    # 辨识/调参/地形生成/分析
│   │   ├── identify_three_loop.py    # 三环辨识
│   │   ├── identify_discrete.py      # 离散辨识
│   │   ├── gen_lqr_models.py         # LQR 模型生成
│   │   ├── gen_split_models.py / split_models.py
│   │   ├── tune_balance.py / auto_tune.py
│   │   ├── gen_terrain_hfield.py     # 地形生成
│   │   ├── plot_fwd.py / sweep_nearfield.py / simulate_trim_lqr.py
│   └── jump_ppo.zip             # 当前导出权重(训练模型存档)
│
├── docs/                         # 分析文档
└── logs/                         # 行车记录仪输出(git 忽略)
```

## 控制架构

### 1. 三环平衡（基座）

三个主控制环并联工作（1 kHz 物理步长）：

| 控制环 | 频率 | 职责 |
| --- | --- | --- |
| 俯仰平衡环 | 1 kHz | pitch PD + 速度阻尼，输出共模轮扭矩 |
| 速度/里程计环 | 100 Hz | 轮速滤波、打滑/离地检测、位置规划 |
| 偏航差动环 | 50 Hz | 转向差速扭矩，A/D 手动转向 |

附加子系统：

- **roll 轴稳定**：左右差动腿伸长找平车体（哪边沉就抬哪边）。
  增益按几何标定：轮距 0.252m 时"完全补偿 1rad 侧倾"只要差动腿长 0.126m，
  所以 `ROLL_STAB_KP` 的量级是 0.1~0.3 m/rad；给到 8 会让 |roll|>0.3° 就吃满
  限幅、整环变 bang-bang 极限环（静止时 roll 峰峰 12°、腿长来回抽）。
- **腿长控制**：E/Q 手动升降，共模伸长 + 左右差动。
- **行车记录仪**：CSV 逐帧记录传感器/指令/操作 + 按键事件日志。

#### 手动驾驶（W/S/A/D）用腿摆走路

前后运动**不用目标俯仰角**，而是把腿摆 `sw` 当速度执行器：

```
W/S/A/D → v_ref(ramp 后的目标速度) → 腿摆速度环
        swing = -Kv·(v_ref - v_body)     (v_body 是已扣除腿运动学耦合的机体速度)
        → 腿摆前后移 → 质心相对轮轴前后移 → 俯仰环输出轮扭矩 → 车走
```

- `sw` 是"轮轴相对基座的前向偏移"，**sw 减小 = 质心前移 = 车前进**，
  所以前进指令给负摆角偏置（与直觉相反，改符号前先看 `balance_step_origin.inc`）。
- 目标俯仰恒为 `SIMPLE_BASE_NEUTRAL`(0)，俯仰环只负责站直。
- `A/D` 纯转向 = `v_ref` 由"钉住进入点"的位置环给出，腿摆负责不让车前后溜，
  差速只负责转向；`hold_pos` 在进手动时锚到当前里程计位置。
- 参数集中在 `control/include/config_manual.h`；摆角平衡点标定项是
  `SIMPLE_MANUAL_SWING_TRIM`（手动硬伺服下零速平衡的摆角偏置）。

### 2. 上层控制器（controllers/）

- **LQR**：小角度（|pitch|<3°）时叠加 LQR 扰动增强稳定。
- **supervisor**：模式切换状态机，负责翻倒自救检测（当前翻倒自救已注释禁用，
  避免跳跃/训练中误触发；三环/LQR 轮子不受影响）。
- **RL 跳跃**：见下节。

### 3. RL 跳跃（PPO 直接力矩）

**状态机**（`controllers/rl_jump/rl_jump_controller.c`）：

```
IDLE →(空格) PREP →(收腿到最短) ACTIVE →(落地稳定/超时) FADEOUT →(渐变) IDLE
```

- `PREP`：通过平衡器 `leg_extend_manual` 把腿收到最短（物理极限 L≈0.057），
  轮子由三环稳住；腿到位或超时 3s 后交 RL。
- `ACTIVE`：RL 全程接管 3s（起跳→空中→落地→恢复）。
  - 动作 5 维 = [腿长力 τ_L, 腿摆力 τ_sw, 左右腿差 τ_diff, 左轮, 右轮]；
    腿部分经运动学 Jᵀ 映射到 4 个主关节（不破坏连杆约束），量程
    τ_L=±150Nm / τ_sw=±60Nm / τ_diff=±60Nm / 轮=±100Nm。
  - 地面阶段轮子交三环（与部署一致），空中轮子固定 0（角动量守恒，空中不可控）；
  - 地面 τ_diff 前馈修正 roll（起跳与落地均生效，落地瞬间增益加大）。
- `FADEOUT`：0.5s 交叉渐变交还三环。

**训练环境**（`scripts/jump_rl_env.py`）与部署完全一致：

- 预热 1~1.5s（真实三环+LQR，supervisor 自救屏蔽）→ 注入长时间漂移状态
  （位置 ±0.5m、轮位置 ±10rad、轮速 ±4、姿态 ±3.5°）→ 收腿到最短 → RL 接管。
- 奖励：高度（封顶 0.18m）+ 起跳角速度惩罚 + **roll 高斯惩罚** +
  落地姿态/腿长对称 + 软落地；**不跳/侧翻重罚 −100**，前后倒轻罚 −10。
- 专家混合：50% 回合前 1s 用实测起跳组合（蹬腿 −0.6 + 腿摆 +0.3 + 轮 0.5）。

**训练流程**：

```bash
# 训练(独立进程并行)
GAMMA=0.999 N_ENVS=10 OUT=scripts/jump_rl_vN python3 scripts/train_jump_rl.py 500000

# 导出权重到 C 控制器
python3 -c "import shutil; shutil.copy('scripts/jump_rl_vN_500k.zip','scripts/jump_ppo.zip')"
JUMP_MODEL=scripts/jump_ppo.zip python3 scripts/gen_rl_jump_c.py

# C 前向 vs Python 一致性验证
JUMP_MODEL=scripts/jump_ppo.zip python3 scripts/verify_rl_jump_c.py

make -C control        # 重新编译
```

**mjbatch 训练后端**：已在 `.venv-mjbatch` 中安装本地 mjbatch，使用 MuJoCo 3.14.0。
每个实例有独立的 C 控制器状态，预热在原生线程中执行；RL 阶段按 1 ms 步长
更新控制，观测仍为 34 维、动作为 5 维，兼容现有 PPO 权重导出流程。

```bash
RL_BACKEND=mjbatch N_ENVS=16 MJ_THREADS=8 N_STEPS=1024 GAMMA=0.999 \
  OUT=logs/rl_training/jump_mjbatch \
  .venv-mjbatch/bin/python scripts/train_jump_rl.py 500000

# 数值一致性、控制状态隔离、局部自动重置测试
.venv-mjbatch/bin/python -m pytest scripts/tools/test_jump_rl_batch.py -q

# 含预热/回合重置开销的采样测速
.venv-mjbatch/bin/python scripts/tools/benchmark_jump_rl_batch.py --envs 16 --threads 8
```

首次构建需要 GCC/OpenMP。`RL_BACKEND=subproc` 可使用独立进程后端。
安装步骤与测试结果见 [mjbatch 训练说明](docs/MJBATCH_RL_20261016.md)。

**RL 调试总日志**：[从 mjbatch 接入到平地重建与 PD/P 参数确定](docs/RL_DEBUG_LOG_20261018.md)。
该日志汇总本次对话的方案演变、训练结果、已确认问题和未完成事项；下面的残差接口为历史版本。

**仿真后端分层**：[控制框架与 Isaac 迁移边界](docs/ROBOT_FRAMEWORK_20261020.md)。
可复用核心位于 `scripts/wheel_legged/`，物理接口使用传感器状态与关节 Nm；
MuJoCo 绑定独立在 `backends/`。目前完成 CPU 参考实现，Isaac/GPU 后端待接入。

**斜坡小陀螺训练（早期残差版本）**：独立的残差 PPO 任务，输入为 41 维传感器和控制状态的 8 帧历史
(328 维)，输出为 5 维腿长、腿摆和轮端力矩修正。roll 奖励为主，pitch 约束较弱。
控制器与奖励共用起停转速轨迹，每 10 万步保存并评估固定工况。

```bash
.venv-mjbatch/bin/python scripts/train_spin_rl.py --steps 500000 --slope 3
.venv-mjbatch/bin/python scripts/eval_spin_rl.py --slopes 0 3 5 8
.venv-mjbatch/bin/python -m pytest scripts/tools/test_spin_rl.py -q
```

接口、完整腿长范围、课程与验收见 [斜坡小陀螺训练说明](docs/SPIN_RL_20261016.md)。

**部署一致性保证**：

- 训练桥 `ident_bridge.so` 直接编译真实 `robot_control_step`（三环+LQR+supervisor），
  训练与 GUI 部署使用同一套控制器；
- 每次回合前 `ident_reset()` 清空控制器积分/滤波/状态机，避免跨回合残留；
- RL 期间 supervisor 翻倒自救屏蔽（`controller_supervisor_set_rl_busy`）；
- 空中轮子固定 0、地面轮子三环、τ_diff roll 修正等前馈/混合，训练与部署完全一致。

## 构建与运行

依赖：MuJoCo 3.x（`$HOME/mujoco`）、GLFW、OpenGL。

```bash
make -C control        # 编译
./controller           # 图形界面
./controller --headless 10   # 无界面运行 10 秒
```

### GUI 按键

| 按键 | 功能 |
| --- | --- |
| W / S | 前进 / 后退 |
| A / D | 左转 / 右转 |
| Z (或 T) | 原地小陀螺（恒定偏航角速度自转，1.2 rad/s 目标，实测 ~0.67 rad/s） |
| E / Q | 腿伸长(升高) / 腿缩短(降低) |
| SPACE | RL 跳跃(先收腿到最短再执行) |
| H | 侧翻自救(手动) |
| B | 平衡开关 |
| V | 切换视角(跟随/俯视/车后/侧面/对肩/车后向前) |
| L | 锁定车体位姿 |
| R | 复位 |
| ESC | 退出 |

### 无界面自动化测试（环境变量）

```bash
MANUAL_TEST=f@4,b@8 ./controller --headless 10   # 前进4s后退8s
JUMP_AT=5,9        ./controller --headless 12    # 旧脚本跳跃(备用)
RL_JUMP_AT=3       ./controller --headless 9     # 3s 触发 RL 跳跃
RL_JUMP_PERIOD=5   ./controller --headless 15    # 每 5s 周期跳跃
ROLL_TILT_START=8  ./controller --headless 8     # 初始8°倾角看roll稳定
FALL_START=1       ./controller --headless 15    # 初始90°侧翻自救测试
SIDE_EXT_TEST=0.1  ./controller --headless 6     # 差动腿伸长注入
ROLL_TORQUE_AT=3,6 ./controller --headless 10    # 3s 施加 6Nm roll 力矩
RL_LEG_DBG=1       ./controller --headless 10    # 打印腿位包络(标定用)
```

## 行车记录仪

`logs/dashcam_<时间戳>.csv` 逐帧记录姿态/轮速/腿位/控制量/里程计，
并追加 RL 跳跃列（`rl_st/rl_a0/rl_a1/rl_ed/rl_sd/rl_oh`）；
`_events.log` 记录按键事件。用 Python/pandas 或 `awk` 分析即可。

## 关键设计约定

1. **只用真实传感器**：控制闭环不读取仿真世界坐标。
2. **参数集中管理**：所有可调增益/限幅在 `control/include/config_*.h`，
   每个参数注明作用与单位。
3. **模块化拆分**：单文件不超过 500 行，按 工具/应用/平衡/控制器/低层 分层。
4. **训练=部署**：训练环境直接编译真实控制器代码，前馈/混合/屏蔽逻辑一致。
5. **停车保持点**：无指令时维持当前位置，不追回启动原点。

## 当前状态与注意事项（v3 模型）

- **模型已切换到 v3 串联腿闭链结构**（`model/v3/`），旧四腿模型保留在
  `model/COD-2026RoboMaster-Balance.xml` 备份。
- 控制器已按 v3 重接：
  - 传感器安装矩阵补偿（`sensors.c`：车体绕 x 立起 +90°，pitch/roll/gyro/acc 重映射）；
  - 腿运动学 J / J⁻¹（`balance/leg_kin_v3.c`）：claf0/大腿 关节角 ↔ (腿长 L, 腿摆 sw)，
    与虚拟力矩映射同步更新；
  - 轮速/轮扭矩符号按 v3 实测（车体 +x 前进时 lw/rw 为负）修正；
  - 腿伺服改为 v3 直接位置伺服（1kHz P+速度阻尼），绕开旧三环 PID 的
    微分先行+采样分频问题。
- **调参状态（传统 LQR+PID 重建中，RL 已暂停）**：
  - 俯仰平衡（轮扭矩 PD）已验证：新模型能站立约 7s（Kp=-120, Kd=-20, 扭矩限 100Nm）；
  - roll 差动腿长环、偏航环、速度阻尼的符号/增益待标定（当前关闭）；
  - LQR 分段模型待重新辨识（`scripts/tools/identify_*`），当前 `CTRL_LQR_MIX_GAIN=0`；
  - 建议从 `ROLL_STAB_KP`（腿差 roll）、`SIMPLE_YAW_SIGN`（偏航）、
    `SIMPLE_VEL_DAMP`（速度阻尼）依次恢复标定。
- RL 跳跃训练模型/脚本保留但暂停使用；恢复前需先按 v3 重新同步环境。
