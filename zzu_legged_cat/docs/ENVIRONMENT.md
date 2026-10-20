# 环境安装与异机使用

更新日期：2026-10-20。本目录可以单独复制到另一台机器；源码、v3 MJCF、15 个 STL、标定和伺服配置均在目录内。运行依赖需要在目标机器安装。当前可运行后端是 MuJoCo，Isaac 训练迁移任务见 [Isaac 迁移说明](architecture/ISAAC_MIGRATION.md)。

## 1. 已验证环境

以下是整理与验证时的本机环境，供复现参考，不是 Isaac 的安装要求：

| 项目 | 实际版本或状态 |
| --- | --- |
| 系统 | Ubuntu 22.04.5 LTS，x86_64，glibc 2.35 |
| 编译器 | GCC 11.4.0 |
| Python | 3.10.12 |
| NumPy / SciPy | 2.2.6 / 1.15.3 |
| PyTorch | 2.12.0；本次 Python 环境 CUDA 不可用 |
| MuJoCo / mjbatch | 3.14.0 / 0.1.4 |
| stable-baselines3 / Gymnasium | 2.9.0 / 1.3.0 |
| Python glfw | 2.10.0 |

已完成 C 构建、独立目录启动、97 项 Python 回归与 MuJoCo 批次接口检查，详见[验证记录](logs/PROJECT_EXTRACTION_20261020.md)。这些结果来自已有依赖环境，尚未在全新系统逐项安装验证；`requirements-dev.txt` 的开发工具版本范围也不等于本机所有已安装版本。

## 2. 复制范围与工作目录

复制整个 `zzu_legged_cat/`，保留内部相对路径。无需原仓库其他目录、原仓库 `.git` 或旧虚拟环境。不要把旧机器的 `.venv-mjbatch/`、`controller`、`control/build/` 和 `.so` 当成可移植依赖；在目标机器重新安装与构建。

下列命令均在复制后的 `zzu_legged_cat/` 内执行。模型、JSON 参数和历史训练脚本不打入 Python wheel；使用时还要保留本目录资产。

## 3. MuJoCo 参考环境

### Python 环境

Ubuntu 下先安装系统工具；原生 GUI 需要 GLFW/OpenGL 和可用显示会话：

```bash
sudo apt-get update
sudo apt-get install build-essential python3-venv python3-dev libglfw3-dev libgl1-mesa-dev
python3 -m venv .venv-mjbatch
.venv-mjbatch/bin/python -m pip install --upgrade pip
.venv-mjbatch/bin/python -m pip install -r scripts/requirements-dev.txt
.venv-mjbatch/bin/python -m pip install -e '.[policy]'
```

训练的固定依赖见 [requirements-mjbatch.txt](../scripts/requirements-mjbatch.txt)，开发与常用回归依赖见 [requirements-dev.txt](../scripts/requirements-dev.txt)。碰撞生成/FCL 审计另装 [requirements-model-tools.txt](../scripts/requirements-model-tools.txt)。不使用 MuJoCo、SB3 或训练脚本时，可以在单独环境只执行 `python -m pip install -e .`，仅安装 NumPy 核心。

### C 程序的 MuJoCo 库

Python 的 `pip install mujoco` 不会自动满足本项目 C Makefile 的目录约定。需准备对应 MuJoCo 原生 SDK，包含 `include/mujoco/mujoco.h` 和可链接的 `libmujoco.so`。建议与 Python 使用同一 MuJoCo 版本。

默认 [control/Makefile](../control/Makefile) 从 `$HOME/mujoco/include` 和 `$HOME/mujoco/build/lib` 查找头文件与库。本机使用此布局，因此直接 `make build` 即可。若下载的 SDK 库位于 `/opt/mujoco-3.14.0/lib`，使用：

```bash
make build MUJOCO_DIR=/opt/mujoco-3.14.0 \
  LDFLAGS='-L/opt/mujoco-3.14.0/lib -lmujoco -lglfw -lGL -lm -lpthread -ldl' \
  RPATH='-Wl,-rpath,/opt/mujoco-3.14.0/lib'
./controller --headless 1
./controller
```

将示例中的绝对路径换成目标机器实际 SDK 路径；`MUJOCO_DIR` 只改变 SDK 根目录，不会自动把 `build/lib` 改成 `lib`。切换 SDK 后先用 `make -C control clean` 清理本地构建产物再重建。`run.sh` 会再次调用构建，使用它时也要通过环境/Make 配置保持相同 SDK 路径。

### 安装后的检查顺序

```bash
make check
make test-rl
make backend-smoke
make headless SECONDS=1
```

C 启动检查需先按实际 SDK 路径完成构建；非默认路径也可直接调用已构建的 `./controller --headless 1`。无桌面的服务器先使用 headless。`backend-smoke` 只推进 4 个实例各 100 步，验证状态、125 维历史和力矩管线，不代表已学会平衡。

本目录没有历史训练 checkpoint，也没有经过新任务验收的策略。`make gui` 启动原生控制器；Python RL 回放使用 `scripts/run_spin_rl_gui.py` 并显式提供兼容权重，二者不是同一控制入口。

## 4. Isaac 目标环境

Isaac Sim 负责仿真，Isaac Lab 提供向量化任务与训练接口。目标机器先按官方当前支持矩阵选择配套版本、操作系统、NVIDIA 驱动、Python、PyTorch 与 CUDA：

- [Isaac Sim 安装要求](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- [Isaac Lab 安装说明](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html)

**当前目录没有 Isaac 后端、USD/URDF 导出资产或可直接启动的 Isaac Lab 训练任务，本机也未安装验证 Isaac。** 不要把上述 MuJoCo 的 Python 3.10 / Torch 2.12 当成 Isaac 环境锁定版本，也不要在 Isaac 环境直接安装整份 `requirements-dev.txt` 覆盖其 Torch/NumPy。

先用目标 Isaac Lab 环境成功运行其官方示例，再用该环境的 Python 安装本目录核心。以下 `python` 必须已指向目标 Isaac 环境：

```bash
python -m pip install -e /path/to/zzu_legged_cat --no-deps
python -c "import numpy, torch, wheel_legged; print(torch.cuda.is_available())"
```

`--no-deps` 用于保留 Isaac 已配套的依赖；调用方需先确认 NumPy 与 Torch 已可用。MuJoCo 参考环境与 Isaac 训练环境分别管理，不复制虚拟环境。完整迁移阶段、观测/动作约定和验收指标见 [Isaac 迁移说明](architecture/ISAAC_MIGRATION.md)。

## 5. 交接时保留的记录

迁移后记录目标 OS/GPU/驱动、Isaac Sim/Isaac Lab 提交或版本、Python/Torch/CUDA、物理步长/控制降采样、求解器参数、资产哈希、关节索引、动作定义、归一化、奖励权重和随机种子。训练权重必须连同这些元数据保存，避免把历史 wrench 策略误当成新的关节目标策略。
