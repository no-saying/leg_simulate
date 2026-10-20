# Slope-spin boundary diagnosis (2026-10-16)

Later short-stop roll travel changes and their separate measurements are recorded
in [Short-leg roll travel allocation](LEG_TRAVEL_ALLOCATION_20261016.md).
That stance allocator is now enabled by default. The collision candidates below
remain experimental and have not replaced the production model.

## Accepted Changes

This follows the unsuccessful 327,680-transition residual PPO experiment in
`SPIN_RL_20261016.md`. No new training or policy deployment was performed.
The actor contract remains 328 observations and five residual actions, with
roll as the main reward and weak pitch shaping. The existing PID identification
and LQR matrices have not been regenerated in this follow-up.

1. Preserve pitch recovery authority after a long-leg spin. The old post-spin
   branch reduced the normal 4 Nm common torque limit to 1 Nm at long lengths.
   Recorded pitch failures followed this reduction during stopping. The branch
   now retains at least the ordinary manual recovery limit, including when a
   lower experimental long-turn limit is configured.
2. Resolve the longitudinal swing permission window dynamically. Enabling the
   full-range residual interface previously expanded the yaw window while the
   speed/swing path retained its old 0.066-0.165 m window. Both now use the full
   physical length bounds with an 8 mm measurement margin in this interface.
   Explicit LEGWIN overrides remain available. LQR eligibility still depends
   on posture; leg length selects and interpolates model segments.
3. Include known wheel joint damping in yaw tracking. Each wheel has
   0.05 Nm/(rad/s) damping in the existing MJCF. For in-place yaw, the ideal
   rolling speed magnitude is omega * track / (2 * radius). Add the required
   steady differential wheel torque to both PID and LQR yaw paths:

   ```text
   tau_drag = 0.05 * 0.272275 / (2 * 0.043) * omega_ref
            = 0.1583 * omega_ref Nm per wheel
   ```

   This is a wheel damping term, not a gyroscopic stabilization model.
   Mixer units are converted once and existing output limits remain active.
   At 2.76 rad/s the damping term alone is approximately 0.437 Nm per wheel.

Evaluation/training output now records controller build ID, PID-only selection,
MuJoCo version and source XML SHA-256. The XML digest does not hash external
mesh assets. Traces save this information in a separate `.meta.json`.
`eval_spin_rl.py --pid-only` and `trace_spin_control.py --pid-only` build isolated
controllers with BOTH `CTRL_LQR_FULL_ENABLE=0` and `CTRL_LQR_MIX_GAIN=0`.
The build cache distinguishes these libraries from the normal controller.
Preprocessing confirms both macros remain zero after loading all config headers.

## Identical 24-case Comparison

Zero residual actions, slopes 0/3/5/8 degrees, extensions -0.093/0/+0.070 m,
both yaw directions, heading zero, seed 1000, 20 s episodes. Nominal physical
lengths span 0.047-0.210 m. Loaded shortest length is approximately 0.052 m.
MuJoCo is 3.14.0; controller build is `41dbc3a7e710fb66`.

| Metric | Before | Accepted controller |
| --- | --- | --- |
| Completed / 24 | 8 | 17 |
| Mean trial roll RMS (deg) | 1.3650 | 1.3385 |
| Mean maximum drift (m) | 1.8243 | 1.6622 |
| Mean duration (s) | 14.998 | 17.127 |
| Recovery pitch-limit failures | 5 | 0 |
| Drift-limit failures | 5 | 4 |
| Rotation stalls | 6 | 3 |

The final rerun exactly reproduced every result in the initial accepted-fix
evaluation. Completion is not a spin acceptance criterion: substantial drift
and under-rotation remain even in some completed cases.

| Slope (deg) | Completed / 6 | Remaining termination |
| --- | --- | --- |
| 0 | 5 | One shortest-leg stall |
| 3 | 5 | One shortest-leg stall |
| 5 | 4 | Shortest-leg drift and stall |
| 8 | 3 | Both longest-leg drift; one shortest-leg drift |

Long and middle cases rotate through approximately 94-97% of the requested
integrated angle when they run the full duration. Longest flat cases now finish
with maximum drift 0.818/0.605 m. Longest 3-degree cases finish with
1.906/1.834 m drift. These are improved recovery results, not stationary spins.
At 8 degrees both longest cases still escape 3 m during the episode.

The shortest mixed PID/LQR cases reach only 0.065-0.542 of their commanded
integrated angle. Pure PID on the original physics completes the four 0/3-degree
shortest cases, but reaches only 0.425-0.609 of the angle; maximum drift is
0.301-0.647 m. Removing LQR therefore does not resolve short-leg rotation.
Pure PID traces repeatedly reach the existing 0.56 Nm differential output cap.
This demonstrates limited output authority, but does not establish that raising
the cap alone would fix the contact and balance problems.

## Geometry Evidence and Rejected Trials

Shortest-leg native contact snapshots show wheel/link contacts involving
`L_claf_link_1_mesh_1` / `L_Wheel_mesh_1` and their right-side counterparts.
Convex-hull penetrations are approximately 3.1-3.8 mm, with individual contact
forces around 18-25 N. These are internal contacts, not terrain obstacles.

Exact STL triangle collision checks using python-fcl found approximately
0.700 mm clearance at reset and at 2 s, despite convex-hull contacts. At 4 s
the right pair actually intersects in the sampled case. Consequently, disabling
all self-collision or excluding these entire body pairs would hide real collisions.

The wheel STL combines 22 vertex-connected CAD solids into one collision hull.
`scripts/tools/build_wheel_colliders_v3.py` generates a separate experimental XML
using a convex hull per connected solid. Explicit body masses/inertias, joint
axes and damping match the original. This is still an approximation: connected
concave solids are convexified. Both old LQR and pure PID fail with this candidate;
all four shortest pure PID cases escape 3 m within 2.66-6.98 s. It has NOT replaced
the primary XML. Controller identification must not be reused as if the physics
were unchanged after a future collision repair.

Other rejected experiments were restored before final validation:

- Hard geometric-center clamping of shortest LQR swing broke its static trim.
- Suppressing shortest PID yaw scissoring increased drift.
- Using independent full-gain leg swing damping during LQR improved long-leg
  drift but destabilized all four shortest cases within 1.68-1.80 s. The existing
  automatic damping schedule was restored. Differential damping is a possible
  future model/control change and is not an accepted one-line fix.

## Verification

- Native `make -C control -j4`, portable control compilation and Ruff passed.
- Spin/jump batch regression: 17 tests passed in 14.22 s. New coverage includes
  immediate full-range window changes, post-spin pitch recovery authority and
  PID-only mode remaining PID when the normal controller enters full LQR.
- Quaternion attitude-rate regression: 567 integration cases passed.
- Native WASD commands each ran 5 s and produced actual translation/rotation.
  W/S displacement was 1.817/2.430 m; A/D rotation was +271.9/-273.0 degrees.
  A separate 5 s spin rotated +480.8 degrees, maximum excursion 0.118 m,
  maximum roll 0.43 degrees. E/Q reached 0.1504/0.0707 m. These native tests
  use MuJoCo 3.10 and the ordinary manual height range, not the full-range RL
  endpoints. They do not validate the unresolved shortest 0.047 m target.
- All ten native positive/reverse 5 s spins at five ordinary heights passed.
  Maximum excursion was 0.166 m; maximum roll/pitch was 0.79/0.78 degrees.
  Logs: `logs/rl_training/spin_boundary_native_heights.log`.
- `git diff --check` passed. No GUI was stopped or restarted.

## Artifacts and Next Work

Accepted grid: `logs/rl_training/spin_boundary_final_eval.{json,log}`.
Native manual log: `logs/rl_training/spin_boundary_native_manual_final.log`.
Pure PID traces: `logs/rl_training/spin_short_original_pid.{csv,json,log}`.
Final PID-only CLI evaluation with provenance:
`logs/rl_training/spin_boundary_short_pid_final.json`; all four cases complete,
mean integrated-angle ratio 0.4918. It reproduces the original PID trace metrics.
Rejected collider: `logs/rl_training/spin_collision_candidate.xml`,
`wheel_colliders/` and `spin_short_collision_pid.*`.
Rejected damping: `logs/rl_training/spin_side_damping.*`.
Geometry evidence: `spin_boundary_contacts.log`, `spin_contact_mesh_check.log`
and `spin_contact_fcl_check.log` under the same directory.

The next prerequisite for full-range training is a physically checked collision
representation and a stable pure PID short-leg operating point on that model.
Then collect new PID-driven identification and validate LQR across the length
range before comparing a new residual policy to this corrected baseline.
The v2 checkpoints have not been accepted or deployed on the changed controller.
Increasing training volume, changing roll weights or expanding the network has
not yet been shown to address these remaining boundary failures.

## Follow-Up: Rigid Ownership and Static PID Checks

These are isolated experiments, not accepted main-model/controller changes.
The production XML, trim center, yaw limits and LQR matrices remain unchanged.
No training or GUI restart was performed. `WHEEL_OBSTACLE_RETRACT` now accepts a
compile override for diagnosis; its default remains 0.012 m.

### Collision Ownership

The whole wheel CAD mesh includes 22 vertex-connected solids. Contacts from
three phase leads (parts 19-21) were observed resisting wheel rotation against
the calf link: a sampled contact torque reached 1.83 Nm, and exact raw triangles
intersected. These leads cannot physically rotate through the calf link.

The experimental builder can attach motor/electronics/lead solids 13-21 to
the wheel carrier while leaving solids 0-12 on the wheel joint. Source STL SHA
and component count are checked before applying these component indices. All
raw visual vertices preserve their zero-wheel-angle world positions. Rotation
tests confirm stationary leads and a rotating rim. The original wheel hull is
retained only for terrain contacts, with detailed solid hulls for robot contact;
MuJoCo's collision mask uses an OR of the two directional bitmask checks.

This is NOT a complete corrected mechanical model. Original explicit masses,
inertial positions/tensors, joint axes, damping and actuator gear are retained,
so motor mass has not yet been transferred to the fixed carrier. Connected
concave components are still convexified for native contact. Blanket wheel/link
collision exclusions have not been adopted.

`audit_spin_contacts.py` now checks raw visual triangles at each component's
actual rigid-body pose. Checking the original invisible terrain hull would
incorrectly rotate the fixed leads and give a misleading audit.

The corrected audit also found a remaining real collision. With extension
+0.070 m, a nominal 0.047 m length loads down to approximately 0.0426 m; the
inner wheel component 00 intersects the calf link. Across 32 saved poses of
the eight-case spin test, 30 of 64 wheel-side checks intersected rotating raw
triangles; fixed parts had no triangle intersections with the audited calf link.
Native wheel contact torque still reached 1.93 Nm. Consequently this candidate
has NOT passed collision acceptance despite better yaw tracking.

### Actual Spin Results

Pure PID, zero policy actions, candidate fixed-motor collision model, temporary
short swing center 0.008 m, zero commanded yaw scissor, 0.8 Nm per-wheel
differential cap, 20 s, seed 1000:

| Slope | Integrated yaw / requested, -/+ | Maximum drift, -/+ (m) | Completed |
| --- | --- | --- | --- |
| 0 degrees | 0.824 / 0.888 | 0.427 / 0.530 | 2/2 |
| 3 degrees | 0.866 / 0.875 | 0.841 / 0.731 | 2/2 |
| 5 degrees | 0.928 / 0.855 | 1.355 / 1.341 | 2/2 |
| 8 degrees | 0.986 / 0.807 | 3.014 / 3.015 | 0/2 |

The 8-degree ratios use the requested angle up to early termination, not a
full-duration request. The smaller original 0.56 Nm yaw cap also completed all
four 0/3-degree cases, with ratios 0.731-0.876 and drift 0.259-1.011 m.
Neither completion nor a larger cap establishes stationary rotation.

### Static and Perturbation Results

The trace tool now supports an explicitly labelled `hold` reference, configurable
duration and a one-time physical pitch angular-velocity increment. The default
training reference and rewards retain their existing behavior. Perturbations
below are +0.12 rad/s at 5 s in a 15 s hold, not an applied-force test.

On the fixed-motor candidate with temporary center 0.008 m and no scissor:

| Requested length | Static max drift, flat / 3 degrees | Perturbed max drift, flat / 3 degrees |
| --- | --- | --- |
| 0.117 m | 0.000018 / 0.00444 m | 0.00349 / 0.00444 m |
| 0.047 m | 0.000067 / 0.00674 m | 0.000207 / 0.00668 m |
| 0.210 m | 1.150 / >3.0 m | 1.268 / >3.0 m |

The shortest results are contact-constrained and cannot authorize identification.
The original production collision model reproduces the longest static metrics
exactly, proving that this longest-case failure predates the collision candidate.
Normal GUI/manual maximum height is not this full-range 0.210 m request.

Long-case diagnostics show wheel/IMU disagreement, persistent obstacle flags
and approximately 12 mm automatic retraction. This shifts the operating point:
the requested length is not the loaded/actual servo reference. Disabling only
per-wheel obstacle retraction made both flat and 3-degree long holds exceed
3 m, so disabling the reflex is rejected, not a fix for the underlying drift.
After the first second, both per-wheel flags remain active throughout the
sampled longest flat hold. Mean extension changes from the requested -0.093 m
to -0.081 m and measured left length averages 0.1913 m. Estimated forward
velocity averages -1.199 m/s while world translation speed magnitude averages
0.113 m/s. That operating point cannot pass a static identification check.

Raising the shortest request to 0.057 m (extension +0.060) gives an initial
loaded length about 0.0506 m. All eight audited wheel-side checks at 0/2/4/8 s
on flat ground had no self contact and at least 0.700 mm exact clearance to the
audited calf link. However, that spin still escaped 3 m. Raising the loaded
height alone therefore does not fix PID control. Reversing the short tilt sign
also failed to fix these collision-free-height trials. Center 0.005 m failed
static holds at the raised-height trials; it is not an accepted replacement.

### Follow-Up Verification and Artifacts

- Spin/jump/collider tests: 19 passed in 13.27 s, including a hold diagnostic
  remaining at zero yaw beyond the ordinary spin start time.
- Collider regression checks unchanged inertial/joint/actuator properties,
  terrain geometry/friction/solver properties, mask routing, zero-angle CAD
  vertex positions and fixed-versus-rotating ownership.
- Ruff passed for modified Python tooling. New trace columns expose independent
  mismatch/obstacle/retraction states and actual common extension reference.
- The final native `make -C control -j4`, 19-test regression rerun and
  `git diff --check` passed. The GUI process was not touched.

Artifacts under `logs/rl_training/`:
`spin_fixed_motor_candidate.xml`, `spin_fixed_motor_yaw08_pid.*`,
`spin_fixed_motor_exact_audit.*`, `spin_fixed_motor_static_pid.*`,
`spin_fixed_motor_perturb_pid.*`, `spin_fixed_motor_loaded_margin_pid.*`,
`spin_fixed_motor_loaded_margin_audit.*`, `spin_original_static_pid.*`,
`spin_long_{retract,noretract}_static_pid.*`.

Next work must establish collision-free PID equilibria at the actual loaded
endpoints, then test disturbance recovery and motion before re-identification.
The remaining wheel-mismatch/retraction feedback needs to be resolved with
the normal obstacle response retained. Existing LQR matrices and policies have
not been revalidated for this candidate plant and must not be presented as such.

Reproduce the accepted comparison and pure PID diagnostic from the project root:

```sh
make -C control -j4
.venv-mjbatch/bin/python scripts/eval_spin_rl.py \
  --slopes 0 3 5 8 --extensions -0.093 0 0.070 --headings 0 \
  --out logs/rl_training/spin_boundary_recheck.json
.venv-mjbatch/bin/python scripts/eval_spin_rl.py --pid-only \
  --slopes 0 3 --extensions 0.070 --headings 0 \
  --out logs/rl_training/spin_boundary_pid_recheck.json
.venv-mjbatch/bin/python -m pytest \
  scripts/tools/test_spin_rl.py scripts/tools/test_jump_rl_batch.py -q
```
