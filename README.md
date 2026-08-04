# leg_simulate - 轮腿自平衡机器人仿真框架

基于 **MuJoCo (MJCF)** 的轮腿（Wheel-Leg）自平衡机器人高保真仿真与控制框架。

## 项目概览

本项目仿真一台带四腿闭链机构的轮腿自平衡车：四个轮腿通过连杆闭链约束构成支撑机构，
整车依靠俯仰（pitch）方向的轮扭矩平衡，通过左右差速实现转向，通过腿长变化实现
升降/跳跃/侧翻自救。控制器使用真实机器人可获得的传感器量（IMU、轮编码器、
腿编码器、执行器力矩），不依赖仿真世界坐标。

## 目录结构

```
leg_simulate/
├── model/                        # MuJoCo 模型
│   ├── COD-2026RoboMaster-Balance.xml   # 主 MJCF 模型(车体+腿+轮+地形)
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
│   ├── balance/                  # 平衡控制核心
│   │   ├── balance_core.c        # balance_update 入口(按段落 include)
│   │   ├── balance_step_sensor.inc     # 传感器预处理/滤波/接触状态
│   │   ├── balance_step_origin.inc     # 原点控制/手动驾驶/腿长/roll稳定
│   │   ├── balance_step_pitch_yaw.inc  # 俯仰PD/偏航环/输出
│   │   ├── balance_step_legacy*.inc    # 历史速度环路径(备用)
│   │   ├── balance_util.c        # 通用工具(滤波/限幅/窗口/复位)
│   │   └── balance_api.c         # 对外接口(getter/setter/触发)
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
│       └── config_lowlevel.h     # 低层输出
│
├── scripts/                      # 辨识/调参/地形生成脚本
├── tools/                        # 日志分析工具
├── docs/                         # 分析文档
└── run.sh                        # 一键编译+运行
```

## 控制架构

三个主控制环并联工作（1 kHz 物理步长下）：

| 控制环 | 频率 | 职责 |
| --- | --- | --- |
| 俯仰平衡环 | 1 kHz | pitch PD + 速度阻尼，输出共模轮扭矩 |
| 速度/里程计环 | 100 Hz | 轮速滤波、打滑/离地检测、位置规划 |
| 偏航差动环 | 50 Hz | 转向差速扭矩，A/D 手动转向 |

附加子系统：

- **roll 轴稳定**：左右差动腿伸长找平车体（`哪边沉就抬哪边`），
  参数见 `config_balance.h` 的 `ROLL_STAB_*`。
- **腿长控制**：E/Q 手动升降，共模伸长 + 左右差动。
- **跳跃**：SPACE 触发 下蹲→蹬伸→空中收腿→落地缓冲 四段时序。
- **侧翻自救**：H 触发，收腿→推腿脉冲 循环恢复平衡。
- **行车记录仪**：CSV 逐帧记录传感器/指令/操作 + 按键事件日志。

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
| E / Q | 腿伸长(升高) / 腿缩短(降低) |
| SPACE | 跳跃 |
| H | 侧翻自救 |
| B | 平衡开关 |
| V | 切换视角(跟随/俯视/车后/侧面/对肩/车后向前) |
| L | 锁定车体位姿 |
| R | 复位 |
| ESC | 退出 |

### 无界面自动化测试（环境变量）

```bash
MANUAL_TEST=f@4,b@8 ./controller --headless 10   # 前进4s后退8s
JUMP_AT=5,9        ./controller --headless 12    # 5s/9s 各跳一次
ROLL_TILT_START=8  ./controller --headless 8     # 初始8°倾角看roll稳定
FALL_START=1       ./controller --headless 15    # 初始90°侧翻自救测试
SIDE_EXT_TEST=0.1  ./controller --headless 6     # 差动腿伸长注入
ROLL_TORQUE_AT=3,6 ./controller --headless 10    # 3s 施加 6Nm roll 力矩
```

## 关键设计约定

1. **只用真实传感器**：控制闭环不读取仿真世界坐标。
2. **参数集中管理**：所有可调增益/限幅在 `control/include/config_*.h`，
   每个参数注明作用与单位。
3. **模块化拆分**：单文件不超过 500 行，按 工具/应用/平衡/低层 分层。
4. **停车保持点**：无指令时维持当前位置，不追回启动原点。
