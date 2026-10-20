# 平衡模块目录结构

这个目录放平衡控制器内部实现，外部代码通过 `balance.h`（实际路径 `control/balance/balance.h`）访问公开 API。

完整模块边界见[代码框架](../../docs/architecture/README.md)，调参与验证见[日志总目录](../../docs/logs/README.md)。

- `balance_api.c`
  公开 API 适配层。负责把 `BalanceSensorFrame` 这种有字段名的输入转换成旧核心还能识别的数组接口，同时保留 GUI 调参函数入口。

- `balance_core.c`
  控制核心。包含 pitch 高频平衡、速度/里程计、yaw 差动、接触/打滑判断和 telemetry getter。

当前拆分原则:

- 参数宏统一在 `control/include/control_config.h`。
- MuJoCo 传感器读取在 `control/sim/sensors.c`。
- 腿/轮输出分配在 `control/runtime/robot_control.c`，MuJoCo 状态读写与推进在 `control/sim/` 和 `control/app/`。
- `BalanceRuntime` 封装所有平衡控制状态，避免文件顶部散落几十个隐式全局变量。
