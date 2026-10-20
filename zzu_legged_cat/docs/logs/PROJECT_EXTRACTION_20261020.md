# zzu_legged_cat 独立工程整理验证

日期：2026-10-20。提取范围见[整理清单](../PROJECT_SCOPE.md)，模型和配置校验值见[来源清单](../../PROJECT_MANIFEST.json)。本轮没有启动新的正式训练。

## 结果

| 检查 | 结果 |
| --- | --- |
| v3 XML 与 STL | 1 个 XML、15 个网格逐文件与原工作区一致，资产路径全部落在本工程内 |
| 标定与参数 | 模型、网格与两份配置共 18 个 SHA256 校验通过；标定中的来源 XML 哈希一致 |
| 本地 Python 依赖 | 从导入语法树核对原工程本地模块，spin 依赖完整；没有上级源码软链接 |
| 文档与核心边界 | 仓库内文档链接检查通过，核心不直接导入 MuJoCo/mjbatch/Gym/SB3 |
| C 可移植性 | 控制核心不使用 MuJoCo/GLFW 头文件编译通过 |
| 原生程序 | 在新目录从源码构建成功；从 `/tmp` 调用 `run.sh --headless 1`，完成模型加载与 1 秒步进 |
| Python 回归 | 完整核心与 spin/pitch 回归 **97 项通过** |
| 物理后端 | 4 实例推进 100 步，共 0.1 秒，125 维历史、数值有限、最大关节力矩 7.5 Nm |
| C 功能检查 | 打滑观测、567 组姿态变换、腿长分配、6 个相机跟随预设全部通过 |
| Python 安装包 | `zzu_legged_cat-0.1.0-py3-none-any.whl` 构建成功 |

初次回归有 3 项因历史 V5/V6/V7 checkpoint 未复制而失败；独立目录改用固定种子的临时源模型生成 checkpoint。保持迁移前后 Actor 输出、critic 保留/重置、特权量隔离与保存加载等断言，之后完整复跑为 97 项通过。该修改只涉及测试，不修改训练或控制逻辑。

## 复现

从 `zzu_legged_cat/` 根目录运行；依赖可使用新建环境，也可通过 `PYTHON` 指定已有解释器。

```bash
make check
make build
make test-rl
make backend-smoke
make -C control check-slip-observer check-attitude-rates check-leg-allocation check-gui-camera
bash run.sh --headless 1
```

仍有原有 TorchScript 弃用提示。短时间启动与接口检查只验证提取后可加载和运行，不是整车长期平衡、GUI 交互或斜坡训练效果验收；本次未打开交互 GUI。
