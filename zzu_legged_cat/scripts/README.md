# Python 代码入口

当前模块边界见[代码框架](../docs/architecture/README.md)，实验演变与指标见[日志总目录](../docs/logs/README.md)。

| 类别 | 文件 | 说明 |
| --- | --- | --- |
| 独立控制库 | `wheel_legged/` | 核心、推理、后端协议与 MuJoCo 适配；可从根目录 `pip install -e .` |
| 训练 | `train_spin_rl.py` | 历史 spin/pitch PPO 入口，使用 `--help` 查看模式 |
| 评估 | `eval_spin_rl.py` | 固定工况、实际姿态、漂移与旋转指标 |
| RL GUI | `run_spin_rl_gui.py` | Python 策略回放，需对应 checkpoint |
| 旧环境链 | `spin_rl_env.py` → `spin_mixed_env.py` → `spin_travel_env.py` → `spin_standard_env.py` → `spin_anchor_env.py` → `spin_asymmetric_env.py` | 历史地形与观测版本继承链，仍依赖 MuJoCo |
| 课程与直接输出 | `spin_return_env.py`、`spin_wrench_env.py` | 回归课程、独占控制与旧紧凑历史接口 |
| 兼容接口 | `spin_joint_pd.py`、`spin_leg_geometry.py`、`spin_wrench_control.py`、`spin_imu.py` | 复用 `wheel_legged` 实现，保留旧导入名 |
| 公共兼容依赖 | `jump_rl_env.py` | spin 环境复用的 XML 路径、四元数工具与 C 帧结构；保留旧模块名 |
| C 桥接 | `ident_bridge.c`、`rl_batch_bridge.c`、`spin_contact_bridge.c` | 编译原生控制、批量控制与接触读写 |
| 开发工具 | `tools/` | 辨识、扫描、验证、pytest 与 C 回归源码 |

训练、测试和导出命令从本工程根目录运行。本目录不包含旧跳跃训练入口和历史 `*.zip`。`jump_rl_env.py` 与 C 的 `controllers/rl_jump/` 是当前编译和导入链仍依赖的兼容代码，保留它们不代表新任务已经训练通过。新训练输出统一放在 `logs/rl_training/`，结果从根目录 `make catalog-logs` 建立索引。

现有任务默认值保留历史行为，重构未修改奖励、动作语义或输出接管方式。新训练前应明确选定任务契约，不能用旧残差默认配置代替用户要求的互斥接管。
