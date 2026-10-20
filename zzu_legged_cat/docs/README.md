# 文档目录

异机安装先看[环境说明](ENVIRONMENT.md)，迁移训练看 [Isaac 交接](architecture/ISAAC_MIGRATION.md)。当前已运行后端为 MuJoCo，Isaac 适配仍待实现。本次整理范围见[分支说明](../BRANCH_NOTES.md)。

| 目录 | 内容 | 入口 |
| --- | --- | --- |
| `architecture/` | 当前代码层次、数据契约、仿真迁移边界 | [代码框架](architecture/README.md) |
| `logs/` | 调试总日志、专题实验、验证结果与历史说明 | [日志总目录](logs/README.md) |

运行命令以[项目首页](../README.md)为入口。实验文档中的参数、默认模式和训练结果对应各自记录日期。

原先直接放在 `docs/` 的文档已归入上述目录，文件名保留，仓库内链接已同步更新。原始 CSV、权重、JSON 和控制台输出保存在仓库根目录 `logs/`，通过日志总目录和本地索引查找。
