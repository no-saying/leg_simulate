# zzu_legged_cat 整理清单

整理日期：2026-10-20。以原工程当前工作区为来源，包含未提交的目录与文档整理成果；不是只导出某个旧提交。原工程文件保留。

## 模型

唯一机器人模型为 [COD-2026RoboMaster-Balance-v3.xml](../model/v3/COD-2026RoboMaster-Balance-v3.xml)，全部 15 个 STL 资产放在同级 `meshes/`。模型文件与网格保持原始内容，闭链约束、惯量、传感器、执行器、地面以及 17/42 mm 测试球场景一起保留。

保留 `model/v3/` 层级，以兼容现有 C 入口、Python 环境、标定来源与测试路径。旧四腿 XML、旧网格和旧 heightfield 未复制。程序构建和运行从本目录完成，不读取上级模型。

## 代码

| 内容 | 位置与说明 |
| --- | --- |
| 原生控制与 GUI | `control/`，包含 app/sim、balance、LQR、切换、执行器分配与记录仪 |
| 可复用 Python 核心 | `scripts/wheel_legged/`，包含算法、接口、运行循环、推理和 MuJoCo 后端 |
| 现有训练与回放 | `scripts/train_spin_rl.py`、`eval_spin_rl.py`、`run_spin_rl_gui.py` 与 `spin_*.py` |
| v3 工具 | `scripts/tools/` 中的腿运动学、伺服扫描、v3 辨识、碰撞、斜坡和手动验证 |
| 回归 | `test_spin_*.py`、核心后端测试、碰撞资产测试及 C 控制工具测试 |
| 参数 | `config/rl_joint_pd.json` 与 `config/v3_robot_calibration.json` |
| 日志工具 | `tools/`，分类索引、CSV 分析、仓库引用与依赖检查 |

`jump_rl_env.py` 仍提供 spin 环境使用的 `ROOT/XML`、四元数工具和 `IdentFrame`。C 的 `controllers/rl_jump/` 与 `balance_step_legacy*.inc` 仍被现有构建或条件编译引用，因此保留源码。没有复制独立的旧跳跃训练流程与 ZIP 权重，也没有用删除依赖的方式改变控制行为。

`wheel_legged` 导入名保留；独立安装发行名为 `zzu-legged-cat`。MuJoCo 适配与物理 Nm 契约不变。Isaac 后端尚未实现，现有 RL 环境继承链也未完成引擎迁移。

## 文档与数据

当前架构见[架构入口](architecture/README.md)，v3 相关的训练、调试和验证见[日志总目录](logs/README.md)。`ANALYSIS.md` 和 RL 总日志保留前期问题背景；其中的历史参数或结果不表示最新配置。

没有复制 `.git`、虚拟环境、缓存、编译产物、原始 CSV/JSONL、训练 checkpoint 和旧模型资产。原始实验文件仍在原工程 `logs/`；专题文档中的历史路径用于说明来源。新工程的 `logs/` 在运行时创建。

`scripts/requirements-dev.txt` 覆盖当前训练、GUI 和常用回归依赖；碰撞生成与 FCL 接触审计另装 `requirements-model-tools.txt`。仅调用引擎无关核心时可只安装根目录 Python 包。

## 验证方式

在本目录执行 `make check`、`make test-rl`、`make backend-smoke`；`make build` 构建本地 C 程序，`make headless SECONDS=1` 检查其实际加载和步进。可通过 `PYTHON=/path/to/python` 使用已有依赖环境，不需要上级工程的 Python 源文件或二进制。

整理不改变控制参数、训练奖励和 checkpoint 的接口语义。没有附带经过整车新任务验收的策略，RL GUI 使用时需要显式提供相匹配的权重。

独立目录的 3 项权重迁移测试改用固定随机种子在临时目录生成 checkpoint，仍检查 Actor/critic 权重迁移、输出一致性与保存加载；不再依赖原工程的 V5/V6/V7 ZIP。这些测试不等同于对旧历史权重训练效果的复测。

本轮完整验证记录见[独立工程验证日志](logs/PROJECT_EXTRACTION_20261020.md)。根目录与新目录的 pytest 默认发现范围均设为各自的 `scripts/tools/`，避免原工程扫描到独立副本后产生同名测试导入冲突。
