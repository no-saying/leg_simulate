# Wheel/IMU Odometry Follow-up

Further sensor-stress and actual return results are recorded in
`SPIN_SENSOR_ROBUSTNESS_20261017.md`, including the directional-reference
revision and its remaining regressions.

This supersedes the initial V7 estimator described in
`SPIN_ASYMMETRIC_AUDIT_20261017.md`. Native PID/LQR trim and the V6 comparison
estimator retain their prior behavior. No long training or GUI deployment
was started.

## Findings and Changes

1. Wheel encoder speed is relative to its moving support, not the chassis.
   The old rolling-speed formula includes body angular velocity but omits
   wheel support rotation caused by the closed-chain legs. A diagnostic
   using support angular truth reduced wheel forward velocity RMSE from
   0.055--0.081 to 0.036--0.064 m/s across three ramp leg lengths.
   Runtime truth is not required: `EncoderLegGeometry` solves the two
   circle-intersection closures from the four driven encoder angles and
   differentiates their geometric constraints. It supplies axle position,
   axle velocity and support angular velocity. It falls back to the old
   approximate geometry when an intersection is unreachable/singular.

2. Corrected IMU integration was repeatedly pulled toward wheel speed in
   about 15 ms, including when both wheels slipped. V7 now keeps a separate
   inertial velocity reference and scores each wheel's signed innovation
   against that reference. Confidence is continuous and independent for
   the two wheels. Mutual wheel agreement cannot alone confirm traction.
   Rejected corrections gradually return to the inertial reference, so an
   earlier false wheel correction does not persist indefinitely.

3. Removing all lateral constraints worsened estimates in these trials.
   The new observer retains the lateral kinematic prior but reduces its
   confidence when the inertial prediction disagrees. Acceleration-derived
   lateral sliding can therefore persist through wheel corrections.
   Confidence is consistency with measurements, not a friction estimate.

4. The estimator used to start after 2.5 s of PID preparation, initializing
   velocity from potentially slipping wheels. An optional C bridge output
   now integrates calibrated IMU specific force throughout preparation.
   It uses only measured quaternion/gyro/acceleration and fixed sensor
   extrinsics/lever arm, never qvel/qacc/contact truth. Integration starts
   at the stationary simulator reset. Measured yaw change rotates the
   estimate during artificial scene placement. The V7 observer continues
   with this velocity when the position anchor is reset.
   Warmup velocity errors were about 0.001--0.008 m/s in the tested cases.
   Correcting initialization alone did not solve sustained slip.

5. The legacy V6 disturbance directly changes qvel. Such a velocity jump
   bypasses the corresponding accelerometer impulse. V7 overrides this
   with a 60 ms physical force pulse and removes it afterward. Native
   balancing and the IMU both see the same physical event. V6 retains its
   old disturbance for existing checkpoint comparisons.

6. Friction experiments must respect MuJoCo material combination: contacts
   between equal-priority geoms use the larger sliding coefficient.
   Lowering only terrain friction below the wheel's default 1 does not
   lower effective contact friction. The audit changes both collision
   wheel geoms and all terrain pieces. An initial floor-only comparison
   was discarded after the two supposedly different runs were identical.

## Identical-Trajectory Results

Each estimator receives the same sensor stream on the same physical
trajectory. The frozen V6 actor controls that trajectory with its original
observations. New observer results are scoring results only: they do not
demonstrate successful control or return to the original position.

The first audit has 36 cases: nominal 47/117/210 mm, lower flat/ramp/toe,
both spin directions, 8 degree terrain, effective friction 0.3 or 1.
Horizon is 8 s; physical failures are retained with their actual duration.
No infeasible pose was rejected by the geometry solver in these trials.

| Effective Friction | V7 Initial Mean Peak Error | New Mean Peak Error | Initial Worst | New Worst |
| --- | --- | --- | --- | --- |
| 0.3 | 0.19037 m | 0.04265 m | 0.58647 m | 0.21254 m |
| 1.0 | 0.04858 m | 0.03535 m | 0.16067 m | 0.11498 m |

Artifact: `logs/rl_training/spin_velocity_innovation_final.json`.

The additional 16 cases use nominal 60/90/150/180 mm, crest/upper flat,
heading -1.2 rad, both directions, 12 s, and effective friction changing
from 1 to 0.3 at 4 s. All 16 lasted long enough to experience the change.

| Metric | V7 Initial | New |
| --- | --- | --- |
| Mean peak position error | 0.16488 m | 0.07084 m |
| Worst peak position error | 0.48919 m | 0.24128 m |
| Mean final position error | 0.15852 m | 0.06924 m |

Artifact: `logs/rl_training/spin_velocity_switch_heldout.json`.

Some individual cases regress despite improvement in the aggregate. For
example, the normal-friction short-leg ramp with negative rotation has
peak error 0.08590 -> 0.11498 m. The low-friction worst case is still over
0.21 m. These estimates are not yet a guarantee for a 0.30 m working area.

## Reproduction

```bash
.venv-mjbatch/bin/python -u scripts/tools/audit_spin_velocity.py \
  --frictions .3 1 --seconds 8 \
  --model logs/rl_training/spin_anchor_v6_resume131k_to1m.zip \
  --out logs/rl_training/spin_velocity_innovation_final.json

.venv-mjbatch/bin/python -u scripts/tools/audit_spin_velocity.py \
  --frictions 1 --seconds 12 --lengths .06 .09 .15 .18 \
  --locations crest upper_flat --heading -1.2 \
  --switch-friction .3 --switch-at 4 \
  --model logs/rl_training/spin_anchor_v6_resume131k_to1m.zip \
  --out logs/rl_training/spin_velocity_switch_heldout.json
```

The observer is connected to `AsymmetricSpinEnv` without changing the
448-sensor actor / 471-input critic / five-action contract. Model provenance
records `odometry_version=encoder_support_innovation_v1`. Episode metrics
include rejected-wheel seconds per side, rejected-lateral-prior seconds and
invalid-geometry seconds. Existing V7 checkpoints need renewed evaluation
because the odometry feature distribution has changed.

## Verification and Limits

Regression tests cover separate and simultaneous wheel slip during spin,
reacquisition, lateral sliding, closed-chain geometry and support angular
rates against constrained multibody motion, IMU preparation integration and
mask isolation, plus existing policy/observation/controller contracts.
The physical disturbance test checks that velocity is not edited directly,
that the IMU observes the resulting velocity change, and that the force
pulse ends after 60 ms.

A 32-transition PPO pipeline check completed after connecting the new
observer, with checkpoint and standalone actor export at
`logs/rl_training/spin_innovation_pipeline_check.*`. Its two 0.12 s evaluations
do not exercise spinning or recovery and are not task performance results.

The observer still has observability limits. Inertial bias or an unknown
initial velocity can resemble steady slip. Confidence thresholds and slow
re-anchoring need validation with sensor noise and longer trials. Neither
friction coefficient identification nor friction randomization in long
training was added here. Native balancing still retains the legacy attitude
convention; the new estimator does not consume its mismatch as confidence.
The next control gate is a fresh policy evaluation with real force
disturbances and strict continuous-spin return metrics, after longer sensor
validation. Short shadow-estimator improvements alone do not pass that gate.
