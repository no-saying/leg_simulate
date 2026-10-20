# Residual Spin Control Follow-up, 2026-10-18

## Active Design

The active experiment retains native PID/LQR balance and the existing five
residual actions. Full torque takeover is not enabled. The source checkpoint is
`logs/rl_training/spin_asymmetric_v7_friction_1m_20261017_655360.zip`.

The source actor and critic weights are migrated. A separate history branch
compresses the preceding seven sensor frames through FC128 -> FC32, projects
the result into the actor's latent space, and adds it to the existing actor.
Its final projection starts at zero, preserving initial actions and values.
PPO trains both actor paths. This is a finite eight-frame window at 50 Hz,
not persistent recurrent memory. Actor inputs remain 448 sensor features;
23 simulator-only features remain restricted to the critic.

## Network Calls

- `scripts/eval_spin_rl.py:evaluate` calls `PPO.predict(obs, deterministic=True)`.
- `scripts/spin_asymmetric_policy.py:AsymmetricSpinPolicy.predict_actor` takes
  only the 448 sensor values for deployment inference.
- `scripts/spin_asymmetric_policy.py:SensorActor.forward` is the exported
  deterministic actor, with no critic inputs.
- `HistoryResidualActor.forward` combines the migrated network and compressed
  history. It does not calculate actuator torques itself.
- Native residual mixing remains in
  `control/runtime/robot_control.c:robot_control_step` and `mix_wheel_outputs`.

## Linkage Mapping

The native polynomial Jacobian differs from the closed-chain encoder geometry
at the reset poses: approximately 10% at requested 47 mm, 1% at 117 mm, and
16% at 210 mm. These are Frobenius relative errors in the position Jacobian,
not measured force errors or proven causes of drift.

`--linkage-map exact` reconstructs the native virtual axle forces from the
native Jacobian and motor torques, then applies the encoder geometry Jacobian
transpose. It retains the original servo feedback and wheel motor torques.
Near an invalid or poorly conditioned mapping it retains the native command.
This is a force-allocation correction, not a full dynamics compensation.

The reusable mapper also accounts for wheel support rotation in virtual work:
`tau_leg = -J_axle.T F_body + J_support.T tau_wheel`. Body coordinates are
x forward, y left, z up. Quasistatic point-force moments are `r cross F`.
Wheel motor reaction, body inertia, gyroscopic effects and contact dynamics
must not be conflated with that point-force wrench. MuJoCo continues to
simulate the full mechanism dynamics.

Wheel mixing instrumentation records native common/differential torque,
RL common/differential torque, common torque before/after speed caps and
the wheel commands. In the 18-case audit the native and RL common torques
often oppose, including successful cases; opposition alone is not a fault.
No speed-cap removal of common torque was observed in this audit.

## Matched Tests

Source actor held fixed, seed 1000, friction 1, slope 8 degrees, short/middle/
long legs, both spin directions, lower flat/ramp/toe, physical push at 7 s.
Each comparison covers 18 scenarios. It is a screening test, not acceptance
over all terrain locations, handovers, friction values and random seeds.

| Mapping | Duration | Strict Pass | Mean Maximum Drift | Return Successes |
| --- | --- | --- | --- | --- |
| Native | 15 s | 7/18 | 0.4875 m | 1/11 |
| Exact | 15 s | 8/18 | 0.4605 m | 1/11 |
| Native | 30 s | 7/18 | 0.5030 m | 1/12 |
| Exact | 30 s | 8/18 | 0.4628 m | 1/11 |

The exact map modestly improves this screen. Return behavior remains poor,
and all ramp cases still fail. No claim of task completion is justified.

Artifacts are in `logs/rl_training/spin_residual_mix_{ideal,exact}_20261018.json`
and `spin_residual_{native,exact}_30s_20261018.json`.

## Torque PI Experiment

The original model uses ideal fixed-gain torque actuators. A selectable motor
model uses an assumed 8 ms first-order response and per-motor gains in
[0.8, 1.2]. Its six PI loops use Kp=0.4, Ki=60/s, torque feedforward and
conditional integration anti-windup. These are uncalibrated assumptions.

The bench test confirms rejection of motor gain/bias errors and recovery
from saturation. The frozen robot policy performs poorly with these added
motor dynamics: without PI 1/18 pass, with PI 0/18. This does not show that
PI is intrinsically unsuitable; the policy and baseline were tuned for the
ideal plant. Neither configuration is enabled in the continuation experiment.
Torque PI controls motor torque tracking, not ground friction or body drift.

## Continuation

A 32,768-transition pilot uses residual control, the history branch, exact
linkage mapping, ideal motors, learning rate 3e-5, three PPO epochs, 32
environments and full leg/terrain/handover randomization. Rollout friction
is sampled from [0.3, 1.2]. No curriculum narrows the leg range in this pilot.

Service: `spin-residual-history-20261018-pilot32k.service`.
Output prefix: `logs/rl_training/spin_residual_history_exact_pilot32k_20261018`.

Final screening uses 30 cases covering all five locations, three leg lengths
and both directions at 8 degrees with LQR handover, 30 s each. This screen
does not replace the 240-case full evaluation or a new held-out test.

## Verification

Actor/critic migration and export, privileged observation isolation, encoder
Jacobian finite differences, virtual work including wheel support rotation,
body wrench reconstruction, exact remapping and PI tracking/reset are tested.
The default residual path remains compatible with existing checkpoints.
