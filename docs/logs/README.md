# 日志总目录

调试、训练、验证和历史说明统一归档在本目录。每篇记录保留原文件名、时间、复现命令和实验结论。

## 总览

| 文档 | 用途 |
| --- | --- |
| [RL_DEBUG_LOG_20261018.md](RL_DEBUG_LOG_20261018.md) | RL 完整演变、接口对照、训练数字、已知问题和待办；包含 10 月 20 日后续记录 |
| [ANALYSIS.md](ANALYSIS.md) | 传统控制长期调参记录，含 PID/LQR、运动、转向和里程计 |
| [README_LEGACY.md](README_LEGACY.md) | 归档前首页，保存早期命令与描述；不作为当前状态说明 |
| [MODEL_SUMMARY.md](MODEL_SUMMARY.md) | 早期模型与控制方案摘要；当前接口见[架构目录](../architecture/README.md) |
| [MUJOCO_LOG.TXT](MUJOCO_LOG.TXT) | 已跟踪的历史 MuJoCo 引擎原始输出 |

## 传统控制与接触验证

| 日期 | 记录 |
| --- | --- |
| 10-15 | [LQR 控制验证](LQR_VERIFICATION_20261015.md) |
| 10-15 | [小陀螺优化](SPIN_OPTIMIZATION_20261015.md) |
| 10-15 | [左右轮独立障碍判断](PER_WHEEL_OBSTACLE_20261015.md) |
| 10-15 | [打滑与 roll 验证](SLIP_ROLL_VERIFICATION_20261015.md) |
| 10-15 | [斜坡小陀螺验证](SLOPE_SPIN_VERIFICATION_20261015.md) |
| 10-16 | [方向性腿长余量与分配](LEG_TRAVEL_ALLOCATION_20261016.md) |
| 10-17 | [轮速与 IMU 审计](WHEEL_IMU_AUDIT_20261017.md) |
| 10-18 | [腿位置 PD 与轮速度 P 台架](JOINT_PD_BENCH_20261018.md) |

## RL 训练与排查

| 阶段 | 记录 |
| --- | --- |
| mjbatch | [训练后端接入、安装与吞吐](MJBATCH_RL_20261016.md) |
| V1/V2 | [初始斜坡残差训练](SPIN_RL_20261016.md) |
| V2 排查 | [短腿、碰撞、接管边界](SPIN_BOUNDARY_20261016.md) |
| V3 | [有限斜坡与混合地形](SPIN_MIXED_RL_20261016.md) |
| V4 | [方向性余量与奖励](SPIN_TRAVEL_RL_20261016.md) |
| V5 | [统一腿长标准](SPIN_STANDARD_RL_20261016.md) |
| V6 | [定点、漂移与回归](SPIN_ANCHOR_RL_20261016.md) |
| V7 | [非对称 Actor-Critic 审计](SPIN_ASYMMETRIC_AUDIT_20261017.md) |
| 传感器 | [噪声、可信度与回归实测](SPIN_SENSOR_ROBUSTNESS_20261017.md) |
| 历史与映射 | [历史网络、残差耦合及精确力矩映射](SPIN_RESIDUAL_HISTORY_20261018.md) |
| 课程 | [回归课程对照](SPIN_RETURN_CURRICULUM_20261018.md) |
| 独占接管 | [互斥输出与 pitch 任务](SPIN_EXCLUSIVE_CONTROL_20261018.md) |

## 原始数据

| 原始位置（仓库根目录下） | 内容 |
| --- | --- |
| `logs/dashcam_*.csv`、`logs/*_events.log` | 逐帧传感器、指令、实际输出与按键事件 |
| `logs/manual_verification/` | WASD、腿长、障碍与斜坡固定工况结果 |
| `logs/rl_training/` | 训练控制台、回合 JSONL、元数据、评估 JSON、checkpoint 与桥接缓存 |
| `logs/rl_deployment/` | 部署和 GUI 回放结果 |
| `logs/tuning/` 及 `logs/` 旧文件 | 历史扫描、辨识与调参产物 |

原始数据仍在原路径，续训脚本与历史报告可继续查找。`logs/` 被 Git 忽略，克隆仓库不包含这些本地产物；仓库中的文档是可共享的实验记录。

```bash
make catalog-logs
```

生成本地[分类索引](../../logs/catalog/index.md)及[完整文件清单](../../logs/catalog/index.json)，按类别记录数量、大小、最近文件和全部文件路径；不读取模型内容、不改动旧文件。新文件产生后重新运行即可更新。

后续每次实验记录控制模式、代码版本、模型与参数来源、实际步数、固定工况指标、原始路径和未完成事项；专题记录加入本目录索引，当前架构说明写入 `docs/architecture/`。
