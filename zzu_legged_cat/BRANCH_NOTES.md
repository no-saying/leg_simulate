# 分支说明

分支：`feat/zzu-legged-cat-isaac-handoff`

基线：`584a11b`，原仓库 `main`

日期：2026-10-20

## 用途

整理可独立复制的 `zzu_legged_cat/`，用于把最新 v3 机器人、相关控制/训练代码及文档交接到另一台机器，并为后续 Isaac Sim / Isaac Lab 训练提供明确的环境与数据约定。

## 本分支内容

- 原仓库文档按架构与实验日志归档，补充构建、检查和日志索引入口。
- 新建独立工程目录，包含 v3 MJCF、全部 15 个 STL、C 控制与 GUI、Python 核心和现有训练代码、标定及伺服配置。
- 独立工程补充打包配置和回归依赖整理；权重迁移测试使用临时生成的测试 checkpoint。
- 新增[异机环境安装说明](docs/ENVIRONMENT.md)和[Isaac 训练迁移交接](docs/architecture/ISAAC_MIGRATION.md)，记录单位、关节方向、观测/动作、接管和分阶段验收要求。

## 验证与未完成项

独立目录已完成 97 项 Python 回归、C 构建与核心可移植性检查、目录外启动检查、MuJoCo 批次接口检查；详见[验证记录](docs/logs/PROJECT_EXTRACTION_20261020.md)。本次交接补充文档后重新执行文档引用与依赖边界检查。

当前可运行的是 MuJoCo 参考工程。Isaac 资产转换、闭链重建、GPU 控制与任务适配尚未实现；没有附带新 25×5 关节目标策略的已训练权重，也未声明斜坡/平地整车新任务已验收。

原始训练数据、虚拟环境、编译产物和 checkpoint 不随本分支打包。保留历史文档中的结果作为实验记录，不将其视为新任务表现。

本文件随 Git 提交上传，作为远端可读的分支描述；本地 Git `branch.description` 不会由普通 push 自动传到 GitHub。
