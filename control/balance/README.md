# 平衡模块目录结构

这个目录放平衡控制器内部实现，外部代码只应该包含 `control/balance.h`。

- `balance_api.c`
  公开 API 适配层。负责把 `BalanceSensorFrame` 这种有字段名的输入转换成旧核心还能识别的数组接口，同时保留 GUI 调参函数入口。

- `balance_core.c`
  控制核心。包含 pitch 高频平衡、速度/里程计、yaw 差动、接触/打滑判断和 telemetry getter。

当前拆分原则:

- 参数宏统一在 `control/include/control_config.h`。
- MuJoCo 传感器读取在 `control/sim/sensors.c`。
- MuJoCo actuator 写入和腿/轮低层混控在 `control/runtime/robot_control.c`。
- `BalanceRuntime` 封装所有平衡控制状态，避免文件顶部散落几十个隐式全局变量。
