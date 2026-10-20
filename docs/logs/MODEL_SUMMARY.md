# 项目模型与控制方案摘要

历史模型与早期控制方案记录。下文的“当前”对应原记录阶段；最新模块和数据契约见[代码框架](../architecture/README.md)。

## 1. 项目范围

本目录是 COD-2026 RoboMaster 轮腿平衡底盘的 MuJoCo 仿真工程。

主要文件：

- `model/COD-2026RoboMaster-Balance.xml` + `model/meshes/`：MJCF 机器人模型和网格。
- `control/app/controller.c`：MuJoCo 主循环、传感器读取、GUI、日志和最终执行器输出。
- `control/balance/`：平衡、速度规划、yaw、里程计、打滑/离地判断。
- `control/lowlevel/pid.c/h`：通用 PID。
- `control/lowlevel/leg_t.c/h`：腿关节伺服封装。
- `control/lowlevel/wheel_t.c/h`：轮电机串级 PID 封装，目前初始化保留，主路径仍直接输出轮扭矩。
- `control/dashcam/`：行车记录仪日志模块(CSV + 按键事件)，与 MuJoCo 解耦。
- `scripts/`：辨识/整定/离线仿真 Python 脚本(identify_discrete、simulate_trim_lqr、
  tune_balance、sweep_nearfield、plot_fwd 等)。
- `tools/`：日志分析工具(analyze_dashcam、plot_odom_plane)。
- `docs/`：ANALYSIS.md、MODEL_SUMMARY.md 等说明文档。
- `config/`：best_params.json 等参数产物。

### 1.1 代码结构与移植边界(2026-08-04)

```text
control/
├── app/               应用层 controller.c: GLFW GUI + headless 主循环
├── balance/           控制核心: 俯仰/速度/偏航 + 腿长(纯 C，无 MuJoCo)
├── lowlevel/          pid / leg_t / wheel_t 通用控制原语(纯 C，无 MuJoCo)
├── runtime/           低层混控入口 robot_control_step(纯 C，无 MuJoCo)
├── dashcam/           行车记录仪日志(纯 C，无 MuJoCo)
├── include/           数据契约 sim_state.h / 参数 control_config.h(纯 C)
├── sim/               仿真胶水: sensors.c 读 MuJoCo -> SensorState
└── legacy/            不参与构建的旧实验/备份(run_headless.c、*.bak)
```

移植边界规则：

- 控制算法部分(`balance/`、`lowlevel/`、`runtime/`、`dashcam/`)只依赖
  `sim_state.h` 数据契约和 `control_config.h` 参数，不 include MuJoCo/GLFW。
- 仿真专属 API 只允许出现在 `sim/sensors.c`、`sim/telemetry.c`、`sim/trail.c`
  和 `app/controller.c`；换真实硬件时替换 `sensors_read()` 实现即可。
- 用 `make -C control check-portable` 验证控制核心能在没有 MuJoCo/GLFW 头文件
  的情况下编译通过。

## 2. MJCF 模型

全局设置：

- timestep：`0.001 s`
- gravity：`0 0 -9.81`
- angle：radian
- floor：`z=-0.26`

机器人结构：

```text
base
├── Left_front_link  -> child1 -> child2 -> child3
├── Left_rear_link   -> child1 -> Left_Wheel_link
├── Right_front_link -> child1 -> child2 -> child3
└── Right_rear_link  -> child1 -> Right_Wheel_link
```

每侧前后腿由 equality `connect` 闭链约束连接。约束较硬，接触冲击时可能放大数值抖动。

当前轮关节：

```xml
Left_Wheel_joint  damping="0.08" armature="0.002"
Right_Wheel_joint damping="0.08" armature="0.002"
```

轮半径按模型真实值使用：

```text
r = 0.058 m
```

## 3. 执行器通道

| 通道 | 执行器 | 当前用途 |
| --- | --- | --- |
| `ctrl[0]` | 左前腿 | 腿关节角度伺服 |
| `ctrl[1]` | 左后腿 | 腿关节角度伺服 |
| `ctrl[2]` | 左轮 | 共模 pitch 扭矩 + 差动 yaw 扭矩 |
| `ctrl[3]` | 右前腿 | 腿关节角度伺服 |
| `ctrl[4]` | 右后腿 | 腿关节角度伺服 |
| `ctrl[5]` | 右轮 | 共模 pitch 扭矩 + 差动 yaw 扭矩，符号按右轮轴向取反 |

轮输出混合：

```text
pitch_mix = 0.6 * balance_tau
yaw_mix   = 0.4 * yaw_tau
left_ctrl  =  (pitch_mix - yaw_mix) / 10
right_ctrl = -(pitch_mix + yaw_mix) / 10
```

## 4. 传感器和现实可用性

控制闭环使用：

- IMU quaternion / gyro / accel。
- 轮编码器位置和速度。
- 腿关节编码器位置和速度。
- 执行器力矩反馈。

仿真世界坐标只用于：

- GUI 相机跟随。
- 日志 `bx/by/bz`。
- 平面轨迹对照显示。
- 离线辨识时剔除明显失效样本。

世界坐标不作为实时控制状态。

## 5. 当前默认控制方案

### 5.1 分频三环

```text
pitch 平衡环：1000 Hz
速度/位置规划：100 Hz
yaw 环：50 Hz
```

三环并联生成不同控制分量：

- pitch 环输出轮共模扭矩。
- 速度/位置规划输出 `target_pitch`。
- yaw 环输出轮差动扭矩。

手动输入：

- `W/A/S/D`：前进/左转/后退/右转。
- `E/Q`：伸长腿(车升高)/缩短腿(车降低)，与辨识腿长分段模型对应，
  行车记录仪含实测 `leg_len` 和手动目标 `le_man`。
- 无界面脚本 `MANUAL_TEST` 的 `f/b/l/r` 对应 WASD，`u/d` 对应 E/Q。

### 5.2 速度规划

原点控制现在按如下链路：

```text
里程计 x -> 位置误差 -> v_ref -> 速度误差 -> target_pitch
```

不是直接命令轮速。这样更符合平衡车实际调试：前后移动由身体倾角和共模扭矩共同实现。

### 5.3 轮速保护

输出级有轮速保护：

- 超过约 `3.0 m/s` 才介入。
- 只覆盖共模 pitch 扭矩。
- yaw 差动仍保留权重限制后的输出。
- pitch 角过大时不累积强刹车，避免恢复阶段翻车。

### 5.4 里程计可信度

轮里程计先经过：

- 离地判断。
- 冲击判断。
- 轮速/IMU 加速度残差打滑判断。
- 低速死区和 ZUPT。

离地和冲击时不使用轮速积分；打滑时只保守积分。

## 6. 当前参数方向

当前选择不是强行把 pitch 环打得很硬，而是：

1. 先让 pitch 快速稳定但不过度激发轮速。
2. 通过 `target_pitch` 规划让轮速反号/刹停。
3. 输出级轮速限幅只兜底，防止异常发散。
4. yaw 先弱化，pitch 共模扭矩不足时不让 yaw 抢力矩。

当前 60s 结果显示轮速已明显压下，原点位置 20s 后收敛到 `+-0.21m`、均值
`0.09~0.12m`（此前是 `+-0.45m` 持续极限环），还没有完全停在零点，
仍在 `-0.05~-0.13m` 小幅振荡，下一步标定质心/腿摆中位。

## 7. 后续修改路线

1. 标定质心和腿摆中位：确认 `BALANCE_LEG_SW_NOM` 对应的静态平衡角。
2. 重新测目标 pitch 到速度的闭环响应，记录延迟、稳态增益和超调。
3. 分阶段调原点控制：
   - 大于 `0.5m`：优先刹车和让速度反号。
   - `0.1m ~ 0.5m`：速度规划回原点。
   - 小于 `0.1m`：低速 ZUPT + 慢积分消偏。
4. yaw 只在 pitch 扭矩有余量时逐步增加。
5. `+-5deg` 内再接 LQR/MPC 精调，不能让局部模型承担大角度恢复。

当前 LQR/MPC 接入原则：

- 不使用单一全局 `compact` 模型，它的离线 mixed 测试会放大 wheel_v。
- 只考虑分段模型：`0~2.5deg` 内段，`+2.5~5deg` 和 `-2.5~-5deg` 外段。
- 分段切换必须使用 smoothstep 和输入变化率限制，不能硬切参数。
- 为后续腿长变换准备：`scripts/identify_discrete.py` 按腿长基线分桶拟合
  `compact_leglen_0..N` 模型，NPZ 带 `leg_len_range`/`expansion_leg_len` 元数据；
  `scripts/simulate_trim_lqr.py --leglen` 按实测 leg_len 平滑切换验证。
- LQR 输出只能作为 PD/速度规划的微调项，不能绕过大角度恢复和轮速保护。
- 辨识默认过滤高轮速样本：`|wheel_v| < 6m/s`。这是为了避免把打滑/空转状态拟合进精调模型。
- 当前辨识默认关闭 speed-plan，并使用对称 pitch bias：`0,+2.5,-2.5,+4.2,-4.2deg`。
- `trim_lqr_model_latest.*` 只在 pitch/roll/compact、三段 pitch piecewise 和全部
  腿长分段模型都导出成功时更新，失败试验只保留时间戳文件。
