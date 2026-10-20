# Mixed Terrain Spin RL (2026-10-16)

The revised directional-travel action, reward and curriculum are documented in
[SPIN_TRAVEL_RL_20261016.md](SPIN_TRAVEL_RL_20261016.md). Training now defaults to
`--terrain travel` (V4); use explicit `--terrain mixed` to reproduce V3 below.

## Correction to V2

The earlier task tilted the entire ground plane and did not randomize controller
handover. It could not establish performance at a real flat/ramp junction.
`slope_spin_mixed_v3` adds these domains. Old V2 policies are not evidence of
performance in this task. Native GUI inference for this spin policy is still
not connected; running `controller` alone does not run this policy.

## Terrain and Sampling

The original ground plane remains horizontal. Two fixed-size mocap box supports
create a 2 m ramp (surface length, 4 m wide) and a 4 m horizontal upper platform.
Mocap transforms are part of each simulation's integration state, including the
native C-controller warmup. The ramp toe and crest are continuous in height.

Each reset independently samples:

- One of lower flat, toe, ramp interior, crest, upper flat, uniformly.
- Toe/crest wheel-midpoint offset continuously in [-0.20, 0.20] m.
- Full-circle heading and both spin directions.
- Ramp inclination uniformly between zero and the configured maximum. Negative
  sampled inclinations are reflected positive; full-circle heading supplies both
  uphill and downhill orientations.
- Full nominal leg-length range 0.047-0.210 m. Each exact endpoint receives 10%
  probability; the remaining 80% is continuous in the full range.

The actual wheel geometry determines support height on each side. One wheel can
be on flat ground while the other is on the ramp, or on the ramp while the other
is on the upper platform. Height differences are consequences of geometry,
heading and position, not an independent artificial sensor perturbation.

The robot first establishes its requested height on flat ground for 2.5 s,
is placed above the real supports without initial wheel penetration, and settles
for 20 ms. The episode then runs without RL until its sampled activation time.
Initial settling and any pre-activation failures are retained; no failed
preparations are discarded. Ground spheres are moved away in this task.

Friction, mass, sensor noise and latency randomization are not implemented here.

## Handover Randomization

Four requested starting conditions are sampled uniformly: PID, LQR, PID to LQR,
and LQR to PID. A runtime permission controls whether LQR may enter; all normal
posture eligibility, fallback and actual supervisor blend dynamics remain active.
The permission defaults enabled in the native application.

RL activation is sampled at 0.8-4.0 s, with authority fading in over 0.1-0.8 s.
For transition cases the permission switches 0.02-0.35 s before activation.
Thirty percent of training episodes also fade RL authority out at 7-12 s, hold
it off for 0.2-1.0 s, then fade it back in. This samples re-entry during spin.

This remains the existing **residual RL** design: PID/LQR continues computing
baseline control, and RL authority multiplies its five residual actions.
Authority one means full allowed residual magnitude, not removal of the baseline.
The existing 40 ms residual filter, slew limit and actuator protection follow it.

Requested controller labels are not assumed to be realized. Each episode records
actual LQR mixture at first nonzero RL authority, actual left/right leg-length
difference, actual support-height difference and the two occupied surface zones.
Failures before activation are recorded as having no takeover. Posture safety
can legitimately prevent LQR entry at difficult terrain or leg lengths.

## Policy and Reward

The five actions and their physical scaling remain those of V2. Each frame keeps
the original 41 sensor/control features and appends:

| Index | Feature | Scale |
| --- | --- | --- |
| 41 | Current residual RL authority | 1 |
| 42 | Current LQR permission | 1 |

Eight chronological frames give **344 float32 inputs, 5 normalized outputs**.
True position, terrain location, slope and support height are not actor inputs.
The original actual LQR mixture remains at frame index 27. Reward still gives
roll stabilization priority, weak pitch shaping, yaw tracking and drift costs.
Network structure remains two 256-unit Tanh layers. V2's 328-input checkpoints
cannot directly resume this task; the new run starts fresh.

## Validation

The training callback uses 240 fixed cases: 3/8 degree inclinations, five
locations, four requested handovers, three endpoint/middle leg lengths, both
directions. Exact toe/crest cases use heading 90 degrees to straddle the edge.
Standalone evaluation also accepts heading and boundary-offset grids. Evaluation
is batched to limit memory use from independent native controllers and scenes.

Verification completed before training:

- Native build passed; Ruff and `git diff --check` passed.
- 24 Python regression tests passed, including four new mixed-terrain tests.
- Native MuJoCo rays match the piecewise surface at both junctions.
- A native warmup step and mjbatch produce matching physics with mocap terrain.
- Controlled activation measured actual LQR mixtures 0, 1, 0.2525, 0.7475.
- A 40-case, 20 s zero-residual baseline at 5 degrees and middle leg length
  completed 40/40. Actual takeover counts: 10 PID, 10 LQR, 20 blending;
  16 takeovers straddled surface zones. Mean trial roll RMS: 0.537 degrees;
  mean maximum drift: 0.509 m; mean integrated yaw tracking ratio: 0.935.

These baseline results verify the task and transitions at middle leg length.
They do not establish endpoint stability or a trained policy improvement, and
0.509 m drift is not acceptance of stationary spin.

A separate 240-case activation audit ran the first 2 s of the full callback grid
(3/8 degrees, all five locations, all four requests, endpoint/middle lengths,
both directions). It observed 62 PID, 58 LQR and 120 mixed activations; all 240
reached activation. There were 92 cross-zone activations. Maximum absolute
support-height difference was 36.43 mm; maximum measured leg-length difference
was 27.13 mm. These are short activation checks, not 20 s stability trials.
The two missing pure-LQR activations remained under normal supervisor safety
logic; requested labels are not substituted for these measured outcomes.
Raw records: `logs/rl_training/spin_mixed_v3_coverage_240.jsonl`.

The first 32,768-step checkpoint was loaded independently and ran four toe cases
for 2 s with actual policy inference, covering all four requested handovers.
Parameters, 344-input observations, five-action predictions and rewards were
finite; native filtered residuals were nonzero and no episode failed. This is a
pipeline check, not evidence of improved control. Raw output:
`logs/rl_training/spin_mixed_v3_policy_smoke.log`.

## Training Run

Fresh V3 training was started with the command below. It requests 524,288
transitions; this document does not claim that training or validation has finished.
Checkpoints are saved every 32,768 transitions, with deterministic validation
every 131,072 transitions and at completion. Evaluation saves the checkpoint
before running its trials, so a crash during evaluation does not lose that model.

```bash
.venv-mjbatch/bin/python scripts/train_spin_rl.py \
  --terrain mixed --steps 524288 --envs 32 --threads 8 --slope 8 \
  --rollout 512 --epochs 10 --eval-interval 131072 \
  --out logs/rl_training/spin_mixed_v3_524k

# Logs and observation/action metadata:
tail -f logs/rl_training/spin_mixed_v3_524k.train.log

# Expanded held-out grid, evaluated in batches of 16:
.venv-mjbatch/bin/python scripts/eval_spin_rl.py --terrain mixed \
  --slopes 3 8 --extensions -0.093 0 0.070 --headings 0 45 90 135 \
  --boundary-offsets -0.12 0 0.12 --batch-size 16 \
  --out logs/rl_training/spin_mixed_v3_baseline_grid.json

.venv-mjbatch/bin/python -m pytest scripts/tools/test_spin_mixed.py \
  scripts/tools/test_spin_rl.py scripts/tools/test_jump_rl_batch.py \
  scripts/tools/test_wheel_colliders_v3.py -q
```

For policy comparison, add `--model <V3-checkpoint.zip>` to the exact same
evaluation command and change the output path. Use identical seed, controller
build, MuJoCo version and case grid. Judge completion, actual rotation, drift and
roll together; low roll RMS after an early failure is not successful control.
