# Exclusive RL Control, 2026-10-18

## Current Direction

The user replaced the residual approach with mutually exclusive RL/native
control. Residual checkpoints are archived and are not usable as full-output
policies. The residual A/B runs had finished; their last partial diagnostic
probe was interrupted when the requirement changed.

`scripts/spin_wrench_env.py:WrenchSpinEnv` now selects one owner for all six
motors. Ownership is binary at every physics tick. There is no weighted
PID/LQR-to-RL torque fade. During RL ownership, native controller `step` is
not called. Native commands and integrators therefore cannot oppose the RL
outputs. Native state is reset when returning from RL to PID/LQR.

The actuator boundary limits physical torque and torque slew, using only the
previous applied command and the selected owner's new command. Slew limits
are [300, 300, 40, 300, 300, 40] Nm/s. This carries the previous output across
a switch without continuing to calculate or mix the outgoing controller.
Motor PI remains disabled.

## Sensors and Network

Leg/wheel encoder observations, closed-chain kinematics and IMU odometry
continue independently of native control. Reward and terrain geometry also
use these refreshed leg observations. Stopping PID/LQR does not freeze wheel
speeds, leg lengths, swing positions or the actor's position estimate.

The actor receives 32 sensor frames, 58 values each (1856 inputs). The previous
31 frames are compressed through FC256 -> FC64; the current 58-value frame
and compressed history pass through FC256 -> FC256 -> six outputs. The critic
also sees 23 privileged features (1879 inputs total). Simulator position,
velocity and contact truth remain excluded from actor inference.

The six outputs are left Fx/Fz, right Fx/Fz, common forward wheel torque and
differential yaw torque. Their scales are [30 N, 40 N, 30 N, 40 N, 1 Nm, 1 Nm].
`spin_wrench_control.py:WrenchTorqueMapper.map` converts these into all six
joint motor torques through encoder geometry and virtual work, including
wheel-support reaction. This is deterministic force allocation, not another
balance/position controller. Physical limits are [60, 60, 10, 60, 60, 10] Nm.

Fresh training initializes the network's two vertical-force output biases
to half the known robot weight. Exploration standard deviation starts at
exp(-2.3), about 0.10. There is no runtime gravity feedforward, servo PID,
external stabilizing force or teacher torque added after the network.

## Training and Evaluation

The first pilot learns balance with zero yaw command. The fresh 1536-step
pipeline run failed all three 3-second checks after takeover; it only confirms
execution, learning, evaluation and export, not control quality. Native output
leakage was zero in all three cases, and native execution time stopped at the
handover instant. This new policy is not ready for slope spin deployment.

The managed 65,536-transition balance pilot uses:

- Service `spin-exclusive-hold-20261018-pilot64k.service`.
- Prefix `logs/rl_training/spin_exclusive_hold_pilot64k_20261018`.
- Fresh six-output model; no residual checkpoint migration.
- All leg lengths 47-210 mm, all five locations and PID/LQR handovers.
- Maximum slope 1 degree, friction 0.3-1.2, yaw reference `hold`.
- 32 environments, 8 threads, 256-step rollouts, 5 PPO epochs, lr 1e-4, seed 21.
- Checkpoints every 16,384 steps; 30-case hold screens every 32,768 steps.

The hold screen retains the existing fixed 8-degree locations, lengths and
directions, but sends zero yaw. Its `bounded_spin_pass` and yaw ratio are not
valid success measures for this preparatory task. Completion, tilt, drift and
time under exclusive RL must be examined. Training on 1-degree scenes does
not establish 8-degree generalization. Spin/return training comes afterward.

The pilot finished all 65,536 transitions in 585.2 seconds. Its final hold
screen completed 0/30 cases. Every case failed the pitch limit; exclusive RL
survival averaged 0.608 seconds (range 0.20-1.10 seconds). The reported total
episode duration of 1.808 seconds includes 1.2 seconds of native control.
Mean max drift 0.228 m is not evidence of improvement because episodes ended
early. Native torque leakage was zero.

## Pitch Foundation Task

The next fresh policy uses `--reward-profile pitch --reference-mode hold`.
It retains all six RL outputs and the same sensor-only actor. After native
reset preparation, RL owns every motor immediately for the whole episode.
Training uses flat ground, initial lengths 107-127 mm, 12-second episodes,
no yaw command, no pushes and no controller dropouts. It does not train
return-to-origin or spin yet. Mechanical torque/slew limits still apply.

Reward density per second is:

- Living reward: +1.
- Pitch: +6 / (1 + (pitch / 10 degrees)^2).
- Pitch rate: -0.5 * tanh((pitch_rate / 1.5 rad/s)^2).
- Secondary roll guard: -0.15 * tanh((roll / 10 degrees)^2).
- Secondary leg-height guard: -0.3 * tanh((length_error / 25 mm)^2).
- Action magnitude: -0.02 * mean(action^2).
- Action change: -0.03 * mean((action - previous_action)^2).
- Failure: fixed -10, replacing the old remaining-horizon penalty (~14,000).

The bounded living density stays positive throughout the permitted action
range, so early termination does not improve return by avoiding large ongoing
negative costs. Native intervals contribute no training reward. Pitch and
pitch-rate use the existing task attitude convention and Euler rate transform;
new RMS/peak metrics count only RL-owned physics ticks. Old anchor/standard
accumulators are excluded from this task's reports because their reward
updates are inactive. Existing finite-state, roll, pitch and 3 m escape safety
termination remains; there is no position tracking cost in this first task.

PPO uses gamma 0.99, GAE lambda 0.95 and entropy coefficient 0.001 for the
short balance horizon. The fixed validation uses 107/117/127 mm at headings
0/90 degrees, immediate exclusive takeover and 12-second hold. Completion,
RL survival, pitch RMS/peak, final leg lengths and drift must all be examined;
survival alone does not establish good pitch tracking or height control.

The 2,048-step execution pilot completed training/evaluation/export but passed
0/6 three-second checks (mean RL survival 0.913 seconds). This is an execution
check, not a trained result. The fresh main run targets 131,072 transitions,
prefix `logs/rl_training/pitch_exclusive_v1_131k_20261018`, with checkpoints and
six-case validation every 16,384 transitions. Full leg range, slopes, return
and spin remain subsequent training tasks after basic pitch control works.

`ReturnWrenchSpinEnv` combines the already implemented return curriculum with
exclusive outputs for subsequent spin training. `--return-curriculum` works
with `--control-mode wrench`; its strict and return-gate evaluations use the
same exclusive actuation and original acceptance thresholds.

## GUI and Calls

`scripts/run_spin_rl_gui.py:LiveWrenchSpinEnv` shares the exclusive allocator
and independent sensor path with training. F switches RL/native ownership;
native manual driving also relinquishes RL ownership. Turning spin off can
leave RL in balance/hold mode. A hold-trained checkpoint starts with spin off.
The GUI's default model points to the new pilot; opening a five-output model
requires the explicit `--legacy-residual` option.

Inference remains `PPO.predict` for simulation, or
`AsymmetricSpinPolicy.predict_actor`/exported `SensorActor.forward` for sensor-
only deployment. The new actor input size is 1856, output size six. Existing
416/448-input, five-output GUI/model files are not interchangeable with it.

## Verification

51 tests passed across full actuation, residual compatibility, return stages,
asymmetric sensor isolation, anchor, mixed terrain and base spin. Two additional
GUI tests passed, checking exact live/training actor and physics agreement and
owner switching without any native execution during RL ownership. Tests also
cover fresh encoder updates without native stepping, torque slew limits,
dropout/re-entry reset, virtual work and TorchScript export.
