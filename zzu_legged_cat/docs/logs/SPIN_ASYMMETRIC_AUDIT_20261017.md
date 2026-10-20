# IMU Frame Audit and Asymmetric PPO V7

Follow-up: `WHEEL_IMU_AUDIT_20261017.md` documents the replacement wheel/IMU
fusion, continuous preparation integration and physical disturbance pulses.
The initial estimator results below are retained as the comparison record.

## Confirmed Measurement Error

The model uses `framequat(body, base_link)`, which reports the inertial
principal-axis orientation. Gyro and accelerometer use the `imu` site.
The base inertial quaternion is approximately
`[0.99974325, 0.00011563, 0.00160520, 0.02260175]`: a 2.60 degree frame
rotation. Applying the same mount bake to these different frames does not
make them consistent. Gravity leaks into the horizontal acceleration.

`spin_imu.py` computes a fixed sensor extrinsic calibration:

```text
q_world_site = q_framequat_body * inverse(q_body_inertial) * q_body_site
q_world_control = q_world_site * inverse(q_mount_bake)
```

This uses installation/model constants and measured orientation. It does
not use runtime world position, velocity or contact truth.

Changing the global sensor frame also changes PID/LQR equilibrium targets.
A trial with globally aligned sensors gave approximately 2.2 m/s short-leg
preparation speed. That global change was withdrawn. The original XML,
PID/LQR trim and old policy input contract are retained. V7 corrects the
odometry input independently. Native slip observer acceleration still uses
the legacy attitude convention; it must not be called fully calibrated.

## Identical-Trajectory Audit

Command:

```bash
.venv-mjbatch/bin/python -u scripts/tools/audit_spin_odometry.py \
  --seconds 8 \
  --model logs/rl_training/spin_anchor_v6_resume131k_to1m.zip
```

Nine cases: nominal 47/117/210 mm, lower flat/ramp/toe, 8 degree terrain.
Each estimator sees the same physical trajectory, controller and actions.
Two ramp trajectories terminated early (3.94 and 7.70 s) for physical
drift; these are reported, not discarded. The corrected estimates do not
drive the audit policy, so this is a measurement comparison, not evidence
that V7 completes the spin task.

Results in `logs/rl_training/imu_frame_audit.json`:

| Estimator | Mean IMU integrated velocity RMSE | Mean peak position error | Worst peak position error |
| --- | --- | --- | --- |
| Legacy frames | 0.9860 m/s | 49.87 mm | 71.75 mm |
| Aligned IMU | 0.00202 m/s | 37.84 mm | 86.42 mm |
| Aligned IMU + calibrated axle offsets | 0.00202 m/s | 38.01 mm | 89.24 mm |

The integrated velocity metric compares the integrated IMU acceleration
against the change in simulator base velocity, including the IMU lever
arm correction. It does not assume initial velocity is zero.

Calibrated lateral axle offsets are -119.375/+133.000 mm; the old observer
uses symmetric +/-136.1375 mm. V7 uses the calibrated geometry. This is a
geometry correction, not an across-case performance improvement claim.

Correcting IMU frames sharply reduces the acceleration integration error.
Fused position error does not improve everywhere: wheel kinematics,
lateral slip and the existing wheel confidence logic remain relevant.
Do not start a large training run based solely on these short trials.

## Asymmetric PPO Contract

- Actor: 448 sensor observations (56 per frame, eight frames), five residual
  actions. Same action allocator and fixed-origin return reward as V6.
- Critic: the same 448 observations plus 23 training-only values. These
  include true displacement/velocity, per-wheel load/slip/support height,
  mapped leg lengths, finite ramp position/grade and return task state.
- `AsymmetricExtractor.policy_net` has only 448 input columns. The critic
  has 471. Privileged values cannot reach actor forward computation.
- PPO still computes advantages and clipped policy updates normally. The
  critic's additional observations improve its value estimate; the actor
  never receives them as direct inputs.
- Deployment export: `.actor.pt`, deterministic `float32[...,448]` to
  bounded `[...,5]`. It contains neither critic nor simulator queries.
- The actor still receives legacy native balance sensor features, while
  its four odometry features use calibrated IMU acceleration. Full native
  frame unification requires PID/LQR trim validation first.
- This version does not estimate friction coefficient online. Native
  observation index 19 is a speed command, not measured chassis velocity.

## Verification

49 focused and existing regression tests passed, including coordinate
consistency at three orientations, true-state observation isolation,
zero actor gradient with respect to privileged inputs, sensitivity of
critic value to privileged inputs, actual PPO parameter updates, save/load,
V6 actor migration and standalone actor export equivalence.

```bash
.venv-mjbatch/bin/python -m pytest -q \
  scripts/tools/test_spin_asymmetric.py scripts/tools/test_spin_anchor.py \
  scripts/tools/test_spin_rl.py scripts/tools/test_spin_mixed.py \
  scripts/tools/test_spin_travel.py scripts/tools/test_spin_standard.py \
  scripts/tools/test_spin_gui.py
```

The training CLI completed a 32-transition pipeline check, actor migration,
two 0.12 s evaluations, checkpoint and actor export:

```bash
.venv-mjbatch/bin/python -u scripts/train_spin_rl.py \
  --terrain asymmetric --steps 32 --envs 2 --threads 2 --slope 8 \
  --rollout 8 --epochs 1 --no-curriculum \
  --warm-start logs/rl_training/spin_anchor_v6_resume131k_to1m.zip \
  --eval-case-limit 2 --eval-episode-seconds 0.12 \
  --out logs/rl_training/spin_asymmetric_v7_pipeline_check
```

These shortened evaluations verify the pipeline only; they do not reach
spin or handover and provide no task success evidence. Without these two
evaluation overrides, anchor/asymmetric evaluation remains 240 cases of
60 s. No new long training or GUI deployment was started.

## Remaining Work

Validate independent wheel confidence against corrected IMU innovations,
including both-wheel slip and sideways motion during spin. Test multiple
friction settings and changes before treating an online friction estimate
as identified. After the measurement chain passes those comparisons,
compare V7 against V6 and zero residual over the full return task. Native
attitude frame migration needs separate static and disturbed PID/LQR
validation across the full leg range.
