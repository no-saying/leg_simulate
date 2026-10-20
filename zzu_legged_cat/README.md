# zzu_legged_cat

ZZU 轮腿平衡机器人最新 v3 模型的独立工程目录。包含该模型的全部 STL、C 控制程序、Python 控制核心、现有 spin/pitch 训练代码与实验文档。可以单独复制或建立 Git 仓库使用。

整理自原工程当前工作区，原工程保留。本目录只包含 v3 模型，不包含旧模型资产、原始训练日志、虚拟环境、预编译库和历史 checkpoint。详细范围见[整理清单](docs/PROJECT_SCOPE.md)。

模型与标定文件的来源及 SHA256 校验值见 [PROJECT_MANIFEST.json](PROJECT_MANIFEST.json)。

## 项目入口

| 需要做什么 | 入口 |
| --- | --- |
| 在另一台机器安装运行环境 | [环境安装与异机使用](docs/ENVIRONMENT.md) |
| 迁移到 Isaac Sim / Isaac Lab 训练 | [Isaac 训练交接](docs/architecture/ISAAC_MIGRATION.md) |
| 查看本次上传范围与分支说明 | [分支说明](BRANCH_NOTES.md) |
| 查看模块边界与调用链 | [代码框架](docs/architecture/README.md) |
| 查看提取范围和保留依赖 | [整理清单](docs/PROJECT_SCOPE.md) |
| 查看传感器、单位和仿真后端接口 | [后端分层与 Isaac 迁移](docs/architecture/ROBOT_FRAMEWORK_20261020.md) |
| 查看所有调试、训练、验证记录 | [日志总目录](docs/logs/README.md) |
| 查看 RL 方案演变、结果与遗留问题 | [RL 调试总日志](docs/logs/RL_DEBUG_LOG_20261018.md) |
| 查找原始 CSV、训练输出和权重 | [原始日志管理](docs/logs/README.md#原始数据) |

## 目录

```text
control/                 C 控制程序
  app/                   MuJoCo 主循环、GUI、键盘与无界面入口
  sim/                   MuJoCo 传感器与可视化适配
  balance/               平衡、里程计、腿运动学、打滑判断
  controllers/           LQR、能量整形、切换与历史跳跃策略
  runtime/               控制输出分配与执行器桥接
  lowlevel/              PID 等控制原语
  include/               C 数据结构与参数
  dashcam/               CSV 与事件记录
scripts/
  wheel_legged/          可独立安装的 Python 控制核心
    core/                运动学、坐标、PD/P、观测、奖励
    policies/            Torch 推理网络
    backends/            MuJoCo 物理适配
    interfaces.py        传感器、训练真值与后端协议
    runtime.py           目标 -> PD/P -> 物理力矩 -> 仿真步进
  spin_*.py              现有 RL 环境与兼容接口
  train_spin_rl.py       历史 spin/pitch 训练入口
  eval_spin_rl.py        策略评估
  run_spin_rl_gui.py     RL GUI
  tools/                 辨识、调参、验证与回归测试
tools/                   仓库检查、日志索引和离线日志分析
config/                  伺服参数与机器人几何标定
model/                   MuJoCo 模型与网格；主模型在 v3/
docs/architecture/       当前代码架构与迁移说明
docs/logs/               全部实验记录与历史文档
logs/                    本地原始数据、训练产物、构建缓存（Git 忽略）
```

## 构建与运行

C GUI 依赖 MuJoCo 3.x、GLFW 与 OpenGL，默认从 `$HOME/mujoco` 查找 MuJoCo。C 库默认位于 `build/lib`；标准 SDK 使用 `lib` 时需覆盖链接参数，详见[环境说明](docs/ENVIRONMENT.md#c-程序的-mujoco-库)。

```bash
make build
make gui
make headless SECONDS=10
# 等价旧命令仍可用：make -C control && ./controller
# 从其他工作目录也可调用：
bash /path/to/zzu_legged_cat/run.sh --headless 10
```

GUI：W/S 前后、A/D 转向、E/Q 升降、Z 小陀螺、R 复位、V 视角、ESC 退出。
原生 GUI 不会自动加载 Python spin 策略。RL GUI 需显式指定与环境契约匹配的本地模型：

```bash
.venv-mjbatch/bin/python scripts/run_spin_rl_gui.py --help
```

## Python 核心与训练依赖

在本目录新建环境，不复用原工程里的源码或构建缓存：

```bash
python3 -m venv .venv-mjbatch
.venv-mjbatch/bin/python -m pip install -r scripts/requirements-dev.txt
```

仅使用独立控制核心时，无需安装训练依赖：

```bash
# 核心仅依赖 NumPy，可在没有 MuJoCo 的环境安装。
python3 -m pip install -e .
# 需要纯 Torch 推理网络时：
python3 -m pip install -e '.[policy]'
# 只安装现有 MuJoCo/mjbatch 训练链时：
.venv-mjbatch/bin/python -m pip install -r scripts/requirements-mjbatch.txt
```

安装包只包含 `wheel_legged`。模型、JSON 标定和历史训练脚本在仓库中，调用者显式传入模型与配置路径。未安装时也可使用 `PYTHONPATH=scripts`。

## 检查与日志索引

```bash
make check               # 文档引用、核心依赖边界、C 可移植性检查
make test-core           # Python 核心与后端回归，需要训练环境依赖
make test-rl             # 本目录内全部 spin/pitch 回归
make backend-smoke       # 4 实例、100 步物理接口检查
make catalog-logs        # 生成 logs/catalog/index.md 和 index.json
```

默认 Python 为已有 `.venv-mjbatch/bin/python`，否则使用 `python3`；可用 `make PYTHON=/path/to/python ...` 覆盖。

`make build` 自动创建运行日志目录。训练、评估和模型参数读取均以本目录为工程根，不引用上级原工程。RL GUI 的 `--model` 需要另外提供匹配权重，本目录没有附带可直接验收的新训练策略。

## 当前进度

已抽取引擎无关的控制、观测、几何与推理核心，并完成 MuJoCo CPU 参考后端。Isaac Sim/Isaac Lab 后端及 GPU 批处理任务尚未实现。

后续训练目标是先做好平地平衡，再扩展移动、旋转和斜坡。已选伺服基线为腿位置 PD `Kp=120, Kd=1.6`、轮速度 P `Kv=0.50`，属于固定机体台架结果。新的目标跟踪架构尚未完成整车训练验收，旧 checkpoint 不能当成其训练成果。

历史训练记录包含残差叠加和独占接管两类方案。后续要求 RL 与 PID/LQR 平衡输出互斥；旧命令和旧默认配置需按具体实验记录理解。详细证据及未完成事项见 [RL 总日志](docs/logs/RL_DEBUG_LOG_20261018.md)。
