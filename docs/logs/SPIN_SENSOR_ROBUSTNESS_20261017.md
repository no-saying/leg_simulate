# Spin Sensor Robustness and Actual Return

Follow-up to `WHEEL_IMU_AUDIT_20261017.md`. No long training, replacement
policy, or GUI deployment was performed. The current policy still fails
the bounded-area continuous-spin task.

## Directional Reference Correction

The V1 innovation observer requires both wheels and the lateral prior to
agree before correcting its independent inertial velocity reference. This
discards a healthy wheel whenever the other slips. A small attitude error
can then integrate until the healthy wheel is rejected too.

A deterministic 12 s regression rotates at 2 rad/s with 0.05 degree world
attitude tilt and one wheel slipping from 2 to 10 s. The old observer ends
with about 0.212 m position error. The directional observer keeps peak
error below 0.02 m and reacquires both wheels. Both slipping sides are tested.

The candidate V2 corrects each heading direction independently. A consistent wheel can
correct forward inertial velocity while the other wheel is rejected; the
lateral prior is gated separately. This does not identify friction or add
an absolute position measurement. It is available as
`InnovationSpinOdometry(..., directional_reference=True)` and
`AsymmetricSpinEnv(..., directional_reference=True)` for comparisons.

The candidate provenance is
`encoder_support_innovation_v2_directional_reference`. It was not promoted:
the default training/environment observer remains V1 after the task-level
regressions below. The audit CLI selects the candidate explicitly. Native
PID/LQR sensing and the existing V5 GUI remain on their previous paths.

## Sensor Stress Comparison

The frozen V6 actor drives its original observations. Shadow observers
receive identical sensor streams and do not affect these trajectories.
Cases cover nominal 47/117/210 mm, both rotation directions, lower flat,
8 degree ramp and toe, with effective friction 0.3 or 1.0.

The horizon is 30 s. Failures are retained: only 11 of 36 trajectories
last the full horizon. Paired case identities and durations were checked
to match exactly between the V1 and V2 runs.

Profiles are illustrative stress settings, not measured hardware specs:

- White noise at each 1 ms sample: acceleration sigma 0.03 m/s^2;
  gyro sigma 0.002 rad/s.
- Residual profile: the same white noise plus acceleration bias
  [0.01, -0.006, 0.008] m/s^2, gyro bias [0.0005, -0.0003, 0.0004] rad/s,
  and a 0.05 degree body roll attitude error.
- Startup integration remains clean; perturbations begin at the anchor.
  These trials do not validate noisy startup or a noisy closed-loop actor.

Each table entry is mean / worst peak position-estimation error in meters.

| Friction | Sensor Profile | V1 | V2 |
| --- | --- | --- | --- |
| 0.3 | Clean | 0.06188 / 0.26847 | 0.06896 / 0.30577 |
| 0.3 | White | 0.06043 / 0.26075 | 0.06832 / 0.30616 |
| 0.3 | Residual | 0.10507 / 0.37337 | 0.08806 / 0.33387 |
| 1.0 | Clean | 0.05024 / 0.13029 | 0.05006 / 0.14559 |
| 1.0 | White | 0.05060 / 0.12885 | 0.04991 / 0.14429 |
| 1.0 | Residual | 0.07166 / 0.16613 | 0.05515 / 0.14948 |

Directional correction improves the residual-error profiles but regresses
the clean and white-noise low-friction cases. Worst error still exceeds
the task's 0.30 m radius. This is not a general estimator acceptance gate.

Artifacts:

- `logs/rl_training/spin_velocity_sensor_stress_30s.json`: V1.
- `logs/rl_training/spin_velocity_directional_reference_stress_30s.json`: V2.

Current reproduction compares both reference rules in one physics rollout:

```bash
.venv-mjbatch/bin/python -u scripts/tools/audit_spin_velocity.py \
  --compare-reference --sensor-stress --frictions .3 1 --seconds 30 \
  --lengths .047 .117 .210 --locations lower_flat ramp toe \
  --model logs/rl_training/spin_anchor_v6_resume131k_to1m.zip \
  --out logs/rl_training/spin_reference_comparison.json
```

`innovation` is V2 and `innovation_all_required` is V1; both have `_white`
and `_residual` variants sharing exactly the same sampled perturbations.

## Actual Closed-Loop Results

These are new physical rollouts using V2 observations and unchanged V6
actor weights. The five-action actor receives only its 448 sensor values.
World/contact state is used for scoring and the unused privileged Critic
path. There is no PPO update in this comparison.

Each friction group has 18 cases with the same geometry/leg/direction grid
above. LQR takeover is fixed at episode start with a 0.3 s residual fade.
Each surviving case receives a 60 ms physical force pulse at 7 s, with
nominal delta-v [0.15, -0.10] m/s. All 18 normal-friction cases and 11 of
18 low-friction cases survive to the push. These results do not cover all
PID/LQR transition modes or changing friction during a closed-loop episode.

Pass conditions retain the existing 0.30 m working radius, return within
0.15 m for 0.5 s after an excursion, 6 s return deadline, continuous spin,
and the final/tail position requirements. Surviving is insufficient.

| Friction | Control | Strict Pass | Mean Maximum Real Drift | Mean Final Real Drift |
| --- | --- | --- | --- | --- |
| 1.0 | Zero residual | 1/18 | 0.630 m | 0.609 m |
| 1.0 | Frozen V6 actor + V1 observer | 3/18 | 0.527 m | 0.451 m |
| 1.0 | Frozen V6 actor + V2 observer | 3/18 | 0.515 m | 0.451 m |
| 0.3 | Zero residual | 2/18 | 0.691 m | 0.676 m |
| 0.3 | Frozen V6 actor + V1 observer | 4/18 | 0.676 m | 0.651 m |
| 0.3 | Frozen V6 actor + V2 observer | 3/18 | 0.662 m | 0.644 m |

The zero-residual physical trajectories are independent of the observer;
they were collected with V1. Their estimator scores should not be used as
V2 scores. V2 actor runs have mean peak odometry error 0.05875 m at friction
1 and 0.06950 m at friction 0.3, far below their actual drift. Failure here
cannot be assigned solely to odometry.

Strict passes by initial mapped uphill-leg band are short 0/12, middle
5/12, long 1/12. All 12 ramp cases fail. Drift/return-time failure triggers
15 normal-friction and 13 low-friction terminations. The V1 actor comparison
passes 7/36 overall versus V2's 6/36. Return successes fall from 11/40
excursions to 6/36 excursions. Switching the observer has not established
a task-level improvement, despite slightly lower mean maximum real drift.
The candidate therefore remains opt-in and was not made the default.

Artifacts:

- `logs/rl_training/spin_closed_loop_innovation_30s.json`: zero residual
  and frozen actor with V1.
- `logs/rl_training/spin_closed_loop_directional_reference_30s.json`:
  frozen actor with V2.

```bash
.venv-mjbatch/bin/python -u scripts/tools/check_spin_closed_loop.py \
  --reference-fusion directional --controllers zero frozen \
  --out logs/rl_training/spin_closed_loop_directional.json

.venv-mjbatch/bin/python -u scripts/tools/check_spin_closed_loop.py \
  --reference-fusion all-required --controllers zero frozen \
  --out logs/rl_training/spin_closed_loop_all_required.json
```

## Verification and Next Gate

The next native-control audit has a concrete starting point:

- `balance_set_spin_rate_reference()` calls `balance_set_manual(0, rate, ...)`.
  In `balance_step_origin.inc`, the manual holding reference uses scalar
  `pos - hold_pos`, and `pos` integrates forward speed as heading rotates.
  This scalar is not a two-dimensional displacement from a fixed origin.
- The full LQR branch in `controller_supervisor.c` takes a zero manual
  body-speed reference during spin, adjusted for leg/pitch axle motion.
  The RL return policy adds swing and wheel-torque residuals afterward.
  Its two-dimensional anchor is not passed as a native position reference.
- `odom_x = pos*cos(yaw)` and the analogous `odom_y` are display projections;
  their mere presence does not establish a control cause. The separate
  `TURN_DRIFT` torque branch also has a default zero torque limit.

These different reference paths warrant an actuator/reference ablation.
They are not yet evidence that disabling the native holding loop improves
the robot. No native balancing code was changed in this investigation.

The 58-test controller/environment/observer regression suite passes,
including per-side and simultaneous slip, lateral sliding, geometry,
physical pushes, actor/Critic isolation and PPO update/export. Short CLI
checks exercise both comparison entry points; they are not spin results.

There is no newly trained usable asymmetric policy. The next control work
requires examining the return action and native holding loop, then training
against a validated observation pipeline and physical pushes with effective
friction variation and sensor uncertainty, followed by strict held-out
return evaluations. Existing checkpoints need evaluation
after any observer change. Online friction identification, noisy startup,
and indefinite position observability during common wheel/lateral slip
remain unresolved.
