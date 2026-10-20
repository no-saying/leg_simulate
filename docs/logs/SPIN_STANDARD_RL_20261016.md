# Standard-Pose Leg Length RL V5

## Measurement Contract

The measurement standard is the uphill leg when the wheel line is aligned with
the uphill/downhill terrain direction. It is a counterfactual pose, not the
instantaneous minimum leg length, a minimum over history, or a constant saved
at episode start. The mapping is evaluated during rotation. At equal base
height and wheel midpoint, changing yaw alone on a given surface must not
change the mapped length of a supported, level robot.

The existing calibrated encoder map supplies body-down axle displacement L
and forward axle displacement sw. L is not a radial link length. With body
Euler roll r, pitch p and fixed lateral axle offset y, the world vertical axle
offset is:

    dz = -sin(p)*sw + cos(p)*sin(r)*y - cos(p)*cos(r)*L

The lateral offsets are extracted once from the existing CAD model, with its
existing +90 degree IMU mount transformation. No new manual track calibration
is substituted for the encoder map. The equal-radius support approximation is:

    H_without_radius = mean(current_surface_height - dz)
    L_high = H_without_radius - max(aligned_surface_heights)
    L_low  = H_without_radius - min(aligned_surface_heights)

The counterfactual levels the body and rotates the planar wheel line about its
current midpoint to the terrain x direction. Its span includes unequal swing
offsets. Heights at both counterfactual wheel positions are queried against the
finite ramp and both flats. At a toe/crest, either, both or neither position can
be on the ramp. A whole-slope tan(angle) correction is not applied there.
No division by the current heading projection is used near the cross-slope
heading. Both aligned high and low lengths are retained, including excursions
beyond 47-210 mm. A feasible high leg does not imply a feasible downhill leg.

This is reward-side inverse support geometry, assuming supported wheels and
equal effective wheel radii. It is not an exact rolling-contact solution at an
edge, nor a valid airborne base-height estimator. Actual contact loss, roll and
slip remain separately penalized. Geometry is refreshed at policy frequency,
50 Hz; measured L, sw and attitude update its vertical projection at 1 kHz.

## Task Reference and Weights

The original native common-length request is mapped using neutral swing and a
level reference footprint at the current position and yaw. This mapped request
sets task weights. Policy actions, actual common-length offset and body tilt do
not directly change that reference. Moving between terrain pieces can change
the reference. Observed equivalent lengths remain independently measured.

The native length request and the 47-210 mm sampling range retain their physical
controller meaning. Their mapped standard lengths can be outside that range on
a slope. In particular a 47 mm common request can map below 47 mm uphill; an
uphill target of 210 mm cannot be sustained when the downhill leg needs to be
longer. These cases are recorded rather than silently clipped or dropped.

Relative to the middle range:

| Term | Short at/below 47 mm | Middle 90-165 mm | Long at/above 210 mm |
| --- | --- | --- | --- |
| Yaw tracking error | 2 | 1 | 1 |
| Signed spin progress | 2 | 1 | 1 |
| Translational speed and XY position cost | 1 | 1 | 1.5 |
| New peak drift cost | 1 | 1 | 1.5 |
| Roll angle/rate, weak pitch | unchanged | unchanged | unchanged |

Use smoothstep interpolation across 47-90 mm and 165-210 mm; no discrete weight
jump occurs at a band boundary. Yaw error has the V4 middle coefficient 0.8;
V4's symmetric end-margin multiplier is replaced, not multiplied again.

The weak height preference compares observed and requested standard lengths.
A new -0.8 mean squared reserve-deficit term, normalized by 20 mm and capped at
4 per direction, preserves uphill shortening AND downhill lengthening room in
the counterfactual pose. Needed reserve is 6 mm plus up to 6 mm during spin.
V4's measured per-leg directional margins and reserve cost remain in force.

## Isolation and Comparison

Actor and critic still receive 52 sensor/action features x 8 history frames
(416 float32 values), and output five residual actions. Standard lengths,
terrain parameters, world position and contact truth are not network inputs.
They do not generate native commands or alter the residual allocator. Native
PID/LQR training behavior is unchanged; the original native GUI does not execute
this Python spin policy. The deployment GUI below loads it explicitly.
A matched nonzero-action test checks V5 and V4 integration
states and actor observations are identical.

Use `--terrain standard` for V5 and `--terrain travel` for the retained V4.
Training defaults to V5; evaluation retains its explicit terrain selector.
The network is unchanged so V4 can be evaluated under the V5 reward, but a
fresh seed-0 PPO run is used for the new experiment. Native commands, spin
targets, randomization, curriculum, network, PPO settings and heldout cases
are matched to the previous 524,288-transition run.

`by_leg_band` in V5 uses the initial mapped requested length. That initial value
is only for reproducible trial grouping; live weights use the current mapping.
`by_nominal_common_leg_band` retains the old grouping for direct physical
comparison with V4. Each row also logs mean/final observed standard lengths,
mean task weights and the fraction of counterfactual poses outside either
physical limit. Reward totals across V4 and V5 are not directly comparable.

Validation covers plane, ramp interior, toe/crest offsets, all headings, both
uphill-side assignments, roll/pitch/swing projection, finite terrain boundaries,
unclipped infeasibility, smooth weights, actor isolation and unchanged physical
commands. Training checkpoints are scored on the same 240 heldout cases.

Verification: all 32 tests across `test_spin_rl.py`, `test_spin_mixed.py`,
`test_spin_travel.py` and `test_spin_standard.py` passed. A 512-transition PPO
pipeline run completed with finite parameters, observations and 416 -> 5
predictions; its checkpoint is `logs/rl_training/spin_standard_v5_pipeline_512.zip`.

The fresh 524,288-transition run and both 240-case reference evaluations started
at 2026-10-16 06:50:26 UTC. Exact commands, process IDs and log paths are recorded
in `logs/rl_training/spin_standard_v5_jobs.json`. The user subsequently requested
a pause at 393,216 transitions; see the resume instructions below. Passing
implementation checks is not evidence of improved spin performance.

## Interactive V5 Deployment

The final model is `logs/rl_training/spin_standard_v5_resume393k_to524k.zip`.
Open the dedicated Python GUI from the repository root:

```bash
.venv-mjbatch/bin/python scripts/run_spin_rl_gui.py \
  --slope 8 --length 0.117 --location ramp
```

This GUI runs the real 416-input, five-output actor at 50 Hz over the native
1 kHz controller. It checks the observation shape and native controller build
against checkpoint metadata. It is Python inference, not a C policy export.
Contact and terrain truth are not used to generate policy commands.

Spin starts enabled. Z/T toggles spin; WASD cancels spin and drives manually;
Q shortens and E lengthens the requested legs. W/S uses PID translation because
the current LQR suppressed forward displacement in deployment checks. RL residuals
are active for spin and fade out during manual driving.
Keys 1..5 reset onto lower flat, toe, ramp, crest and upper flat respectively.
Brackets change the slope, R resets, F toggles RL, P toggles LQR permission,
V changes the camera, Space pauses and Esc closes the window.

Deployment checks cover four seconds of matched training/deployment state and
observation evolution with nonzero actions, plus five-second W/S/A/D trials
checking actual displacement and signed yaw. All five tests pass. These checks
verify the deployment path; the final policy still has the drift and short-leg
rotation limitations reported by its heldout evaluation.

## Paused Run and Resume

The user requested continuation on 2026-10-16. Resumption started at 19:33:21
Asia/Shanghai from the checkpoint below, with the original reward-source hash
and native controller build verified unchanged. Training runs under the separate
`spin_standard_v5_resume393k_to524k` prefix; the interrupted checkpoint evaluation
is rerun as `spin_standard_v5_393216_recheck`. Commands, PIDs and logs are in
`logs/rl_training/spin_standard_v5_resume_jobs.json`.

Training PID 545855 was terminated at the user's request while the 393,216-step
evaluation had completed 128 of 240 cases. That evaluation has no complete
result file. The latest complete evaluation is still the 262,144-step result.

Verified resumable checkpoint:
`logs/rl_training/spin_standard_v5_524k_393216.zip`. It loads successfully with
393,216 transitions, 230 PPO optimizer updates, finite weights, optimizer state,
and the 416 -> 5 contract. Both completed reference evaluations are preserved.
The checkpoint was saved before evaluation, so terminating that evaluation did
not discard newer learned weights. Live environments and the pending rollout
are not serialized; resuming starts fresh environments from the saved policy
and optimizer, rather than reproducing the interrupted trajectory exactly.

To finish the original 524,288-transition target, add 131,072 transitions. Use
a separate output prefix to preserve the original metadata and logs:

```bash
.venv-mjbatch/bin/python -u scripts/train_spin_rl.py \
  --terrain standard --resume logs/rl_training/spin_standard_v5_524k_393216.zip \
  --steps 131072 --envs 32 --threads 4 --slope 8 \
  --rollout 512 --epochs 10 --eval-interval 131072 \
  --out logs/rl_training/spin_standard_v5_resume393k_to524k
```

Curriculum resumes at stage 2 from the saved transition count. To rerun the
interrupted evaluation separately:

```bash
.venv-mjbatch/bin/python -u scripts/eval_spin_rl.py \
  --terrain standard --callback-grid --slopes 3 8 --extensions -0.093 0 0.070 \
  --batch-size 32 --threads 4 \
  --model logs/rl_training/spin_standard_v5_524k_393216.zip \
  --out logs/rl_training/spin_standard_v5_393216_recheck.json
```

## Commands

```bash
.venv-mjbatch/bin/python -m pytest -q scripts/tools/test_spin_standard.py
.venv-mjbatch/bin/python -u scripts/train_spin_rl.py \
  --terrain standard --steps 524288 --envs 32 --threads 4 --slope 8 \
  --rollout 512 --epochs 10 --eval-interval 131072 \
  --out logs/rl_training/spin_standard_v5_524k
.venv-mjbatch/bin/python -u scripts/eval_spin_rl.py \
  --terrain standard --callback-grid --slopes 3 8 --extensions -0.093 0 0.070 \
  --batch-size 32 --threads 2 --model logs/rl_training/spin_travel_v4_524k.zip \
  --out logs/rl_training/spin_standard_v5_reference_v4.json
.venv-mjbatch/bin/python -u scripts/eval_spin_rl.py \
  --terrain standard --callback-grid --slopes 3 8 --extensions -0.093 0 0.070 \
  --batch-size 32 --threads 2 \
  --out logs/rl_training/spin_standard_v5_reference_zero.json
```
