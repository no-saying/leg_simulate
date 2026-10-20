# Directional Leg Travel RL V4

V5 adds the standard-pose uphill-leg mapping and asymmetric task weights. See
[SPIN_STANDARD_RL_20261016.md](SPIN_STANDARD_RL_20261016.md). V4 remains available
with `--terrain travel` for matched physical comparisons.

## Scope

V4 retains finite-ramp geometry and randomized PID/LQR handovers from V3.
It revises physical leg travel, the common-length action, reward and curriculum.
The native PID/LQR controllers and actuator limits are unchanged. This remains
residual RL. The native GUI does not yet execute the spin policy or this Python
reference allocator.

The original code already supplied eight frames of sensor/action history,
action smoothing, per-millisecond XY drift costs and residual control. V4
strengthens or changes these existing mechanisms rather than adding duplicates.
No recurrent network or asymmetric critic has been added.

## Direction and Observation Contract

Lengths are measured through the existing C
`leg_kin_v3_calibrated_side()` forward kinematics from the two joint encoders
on each side, including the right-side mirror. They are not world-truth axle
coordinates. Physical reference limits are 0.047-0.210 m, derived from the same
nominal midpoint and extension limits used by the controller.

Physical positive length means lengthening. Native positive extension means
retraction, so a positive common physical offset SUBTRACTS from native extension.
Positive differential means left lengthening and right shortening.

The original V3 43 features remain, with these updates:

- Index 24 retains the original requested nominal height.
- Index 37 now represents the applied common-reference offset / 0.040 m,
  replacing the unused native common residual filter state.
- Indices 43-50 add physical travel in this order: left shorten, left lengthen,
  right shorten, right lengthen, common lengthen, common shorten,
  differential positive, differential negative. Normalize by 0.163 m.
- Index 51 adds common-reference velocity / 0.025 m/s.

For a leg with measured length L, shortening travel is L-Lmin; lengthening
travel is Lmax-L. Common travel is the minimum corresponding travel across
both legs. Positive differential travel is min(left lengthen, right shorten),
and negative differential travel is min(left shorten, right lengthen).
Negative margin values preserve evidence of a measured limit excursion.
These are configured physical-range margins, not certificates of the complete
reachable workspace at every swing angle. The existing calibrated map has its
documented fit limitations near the deepest folded poses.

Actor input is **52 features x 8 frames = 416 float32 values**. Output remains
five normalized actions. Contact force, contact state, contact location, true
position, true body velocity and slope are not actor or critic inputs. V3's
344-input policies cannot directly resume V4.

## Action Allocation

Actions 0/2/3/4 retain their differential-length, swing, common-wheel-torque
and differential-wheel-torque meanings and physical maxima. Differential leg
correction is reduced only in its unavailable direction using measured physical
travel. Wheel balancing and yaw authority are not scaled down at short/long legs.

Action 1 now requests a common physical reference offset up to +/-40 mm,
instead of the old +/-10 mm instantaneous residual. Available range is computed
in each direction relative to the original height request. Its reference passes
through a 0.2 s target-to-rate response, a 25 mm/s speed limit and nominal
100 mm/s2 acceleration limit. Target crossing and hard physical bounds project
the reference for protection. A leg-obstacle reflex suppresses positive common
extension targets. Original final servo/leg limits still apply.

The applied reference offset is exposed in the actor history. Native common
residual action 1 is zero to avoid applying the correction twice. Authority fade
and dropout multiply the requested action; the reference subsequently returns
smoothly towards zero. Zero policy actions preserve V3's physical evolution.

## Contact Computation

`spin_contact_bridge.c` queries copied integration states at 50 Hz in its own
MuJoCo data pool. It never advances physics or controller state. Only wheel-body
contacts against floor, ramp and upper platform count; wheel/leg self-contact
and remote spheres are excluded.

Each side supplies total normal load, load-weighted tangential slip speed,
load-weighted contact height and loaded contact count. Relative point velocity
is J_wheel(point)*qvel - J_support(point)*qvel, projected onto the contact plane.
It includes body yaw and wheel rotation, so desired rolling rotation is not
penalized merely because wheel encoders differ from body translation.

Contact measurements enter reward and the reward-side inverse support geometry
only. A higher left contact requires a shorter left leg for upright posture:
the geometry residual is (Lleft-Lright)+(hleft-hright), when both sides are loaded.
Terrain support samples do not generate hidden actor commands.

## Reward Changes

Reward densities integrate at 1 ms. Contact measurements are held for 20 ms.
Roll (-5 at 3 degrees), roll-rate and weak pitch (-0.25 at 8 degrees) retain
their original definitions.

- Drift: negative squared speed / 0.2 m/s (capped at 9), plus position Huber
  cost. Position cost is e^2 for e<=1, otherwise 2e-1, e=distance/0.3 m.
  It keeps increasing through the 3 m failure boundary.
- New maximum-drift cost: -4*(new_peak-old_peak)/0.3 m, charged once per
  excursion. Returning does not refund maximum drift.
- Yaw: 2 - 0.8*(1+0.5*extremity)*Huber(abs(rate-target)/0.6). Extremity
  increases as the minimum directional physical margin falls below 25 mm.
- Spin progress: up to +0.6/s for signed rate/target tracking, capped at one.
- Leg reserve: -0.8 times mean squared reserve deficit / 20 mm, capped at 4.
  Needed reserve is 6 mm + up to 6 mm for spin + half measured support-height
  difference, capped at 40 mm. This softly encourages leaving either limit;
  it does not force every height request to the middle.
- Nominal-height preference weakens to -0.03*(mean length error / 40 mm)^2,
  capped at 4, allowing useful clearance instead of fighting it.
- Action-change weight rises from 0.05 to 0.15; common-reference speed has
  an additional bounded cost of up to 0.05/s.
- Loaded contact slip costs up to 2.4/s. Loss of support costs up to 0.3/s.
  Signed support-geometry error costs up to 0.8/s.

Failure retains the same physical thresholds but charges 10 + 85 times remaining
seconds, accounting for the stronger bounded reward costs. V3 and V4 returns
are on different reward scales and must not be compared as learning progress.
Compare actual roll, drift, rotation and failures on identical scenes.

## Curriculum and Evaluation

All five terrain locations and four handover requests remain randomized at
every stage. A fixed training schedule expands newly reset episodes:

| Transitions | Nominal Lengths | Maximum Slope |
| --- | --- | --- |
| 0-65,535 | 90-150 mm | 3 degrees |
| 65,536-131,071 | 70-180 mm | 5 degrees |
| 131,072 onward | 47-210 mm | configured maximum, 8 degrees in this run |

This is a step schedule, not an automatic performance acceptance gate. Existing
episodes retain their original height targets. Evaluation always uses the full
240-case grid, including both endpoint heights, two slopes and both directions.
Reports additionally group physical metrics and return by short/middle/long bands.

## Verification and Probe

- The original 24 regressions plus five initial V4 tests passed (29 total).
- Six V4 tests passed after adding curriculum and peak-drift checks.
- Coverage includes directional margins, inward common reference signs and rate
  bounds, original physics with zero actions, no contact truth in the actor,
  native contact forces and point Jacobians, reward signs and curriculum stages.

A paired 20 s diagnostic used the same seeds/poses on an 8 degree ramp with
heading 90 degrees. This was a constant hand-written action, not a learned policy.

| Nominal Height | Common Action | Roll RMS | Max Drift | Actual/Target Turn |
| --- | --- | --- | --- | --- |
| Short | zero | 1.505 deg | 1.478 m | 65.2% |
| Short | +40 mm inward | 1.215 deg | 1.051 m | 94.6% |
| Long | zero | 1.443 deg | 1.565 m | 90.8% |
| Long | -40 mm inward | 1.099 deg | 2.085 m | 91.9% |

This supports extra height authority at the short boundary and shows why blindly
shortening every long-leg spin is inadequate. The maximum-drift cost was added
after this probe; its logged returns precede that addition. Original results:
`logs/rl_training/spin_travel_v4_inward_probe.jsonl`.

The full V3 zero-residual 240-case comparison completed using the exact callback
grid and seed 1000. Controller build, model XML hash, MuJoCo version and all
ordered case identities (including initial XY) match the 393,216-step policy.
The baseline completed 234/240; the old policy completed 238/240.

| Nominal Height | Zero Drift | V3 393k Drift | Zero Turn Ratio | V3 393k Turn Ratio |
| --- | --- | --- | --- | --- |
| Long | 1.282 m | 1.130 m | 94.9% | 94.8% |
| Middle | 0.579 m | 0.580 m | 93.5% | 94.2% |
| Short | 0.885 m | 0.869 m | 53.0% | 57.2% |

Mean maximum drift improved from 0.915 to 0.859 m, while mean trial roll RMS
changed from 0.767 to 0.778 degrees. This is a modest baseline-relative change,
not acceptance of stationary spin or endpoint control. The baseline is the
matched PID/LQR supervisor with zero residual, not a PID-only build. Baseline
drift alone is not proof of a mechanical limit; baseline control can also be wrong.
Raw comparison: `logs/rl_training/spin_mixed_v3_paired_393216.json`.

V4's 1,024-transition PPO pipeline check completed with finite parameters,
416-input observations and five-action predictions. The full fresh V4 run was
started with the command below and has saved its first 32,768-step checkpoint.
These checks establish execution and learning updates, not policy improvement.
The first fixed V4 evaluation is scheduled at 131,072 transitions. The previous
V3 run is retained as a separate experiment. V4 training logs are at
`logs/rl_training/spin_travel_v4_524k.train.log`.

## Commands

```bash
.venv-mjbatch/bin/python scripts/train_spin_rl.py --terrain travel \
  --steps 524288 --envs 32 --threads 4 --slope 8 --rollout 512 --epochs 10 \
  --eval-interval 131072 --out logs/rl_training/spin_travel_v4_524k

.venv-mjbatch/bin/python scripts/eval_spin_rl.py --terrain travel --callback-grid \
  --slopes 3 8 --extensions -0.093 0 0.070 \
  --model logs/rl_training/spin_travel_v4_524k_131072.zip \
  --out logs/rl_training/spin_travel_v4_policy_240.json

.venv-mjbatch/bin/python -m pytest scripts/tools/test_spin_travel.py \
  scripts/tools/test_spin_mixed.py scripts/tools/test_spin_rl.py \
  scripts/tools/test_jump_rl_batch.py scripts/tools/test_wheel_colliders_v3.py -q
```
