# mjbatch RL Training

## Run

The installed `.venv-mjbatch` environment uses the local
`/home/yeht/mjbatch-main` package (0.1.4), MuJoCo 3.14.0 and SB3 2.9.0.
It inherits the existing PyTorch installation. The GUI's native MuJoCo 3.10.0
and system Python MuJoCo 3.12.0 installations are unchanged.

```sh
RL_BACKEND=mjbatch N_ENVS=16 MJ_THREADS=8 N_STEPS=1024 GAMMA=0.999 \
  OUT=logs/rl_training/jump_mjbatch \
  .venv-mjbatch/bin/python scripts/train_jump_rl.py 500000
```

`N_STEPS` is rollout length PER environment. Total rollout samples are
`N_STEPS * N_ENVS`; no divisibility of N_STEPS by N_ENVS is required.
`BATCH_SIZE=256`, `N_EPOCHS=10`, `TORCH_THREADS=1`, `DEVICE=cpu` default.
`RESUME=path/to/model.zip` loads an existing SB3 PPO checkpoint. Its saved
training hyperparameters are retained on resume. The model architecture,
observation/action spaces and policy export format remain unchanged.

`RL_BACKEND=subproc N_ENVS=8` selects independent worker processes. The old
training entry used the default DummyVecEnv, which was sequential and shared
one controller library across environments. The process backend now explicitly
uses SubprocVecEnv when N_ENVS > 1 and builds its bridge from current source.

## Architecture

- `jump_rl_batch_env.py` implements SB3 VecEnv with mjbatch physics. IMU
  compensation, 34 observations, five actions, action filtering, expert
  mixing, ground/air wheel handling and reward terms follow JumpRLEnv.
- Every 1 ms RL step updates control and runs one physics step. No action
  repeat or changed policy rate is used for speed. `forward=False` retains
  ordinary mj_step sensor timing, matching the scalar environment.
- The deployed controller has static state across several translation units.
  Each environment loads a separate temporary copy of the controller shared
  library. `-Bsymbolic` keeps its internal symbol references local. This
  isolates PID integrals, filters, LQR states, supervisor, slip detection and
  leg servos without a refactor of the deployed controller.
- `rl_batch_bridge.c` runs selected controllers in native OpenMP threads.
  Episode warmup also runs the real controller and mj_step entirely in C,
  with one scratch mjData per worker. Integration states and final sensor
  values are transferred back to the mjbatch buffers. This removes a Python
  round trip for each warmup millisecond.
- Ended episodes reset only their own physics, controller, RNG sequence and
  reward/filter state. Terminal observation, episode return/length and
  TimeLimit.truncated are supplied to SB3 before autoreset observations.
- Reset clears complete integration state, including solver warm starts;
  the old scalar environment only overwrote pose, velocity and controls.
- Builds are cached under `logs/rl_training/<source-hash>` and temporary
  controller copies are unloaded and removed on close. Source/header and
  MuJoCo version/path changes invalidate the build cache.

This implementation targets Linux with GCC/OpenMP and the current v3 model.
The scalar fallback continues to be available. A controller library copy is
kept per simulation, so controller memory still grows with environment count.
Warmup currently uses the common template model; per-instance model domain
randomization would need to be applied to warmup as well as mjbatch physics.

## Installation

Reproduce the installed environment without replacing global dependencies:

```sh
python3 -m venv --system-site-packages .venv-mjbatch
.venv-mjbatch/bin/python -m pip install --upgrade pip packaging \
  'scikit-build-core>=1.0' 'nanobind>=2.2.0' 'cmake>=4.1' \
  'mujoco==3.14.0' 'opencv-python-headless>=4.10,<6'
.venv-mjbatch/bin/python -m pip install --no-build-isolation /home/yeht/mjbatch-main
.venv-mjbatch/bin/python -m pip install -r scripts/requirements-mjbatch.txt
```

The build tools are needed when installing the local mjbatch source. A fresh
machine also needs PyTorch/SB3 and a C compiler. OpenCV in this environment
supports the installed NumPy version; the older global OpenCV build did not.

## Verification

```sh
.venv-mjbatch/bin/python -m pytest scripts/tools/test_jump_rl_batch.py -q
.venv-mjbatch/bin/python scripts/tools/benchmark_jump_rl_batch.py \
  --envs 16 --threads 8 --steps 1600
```

Five tests cover scalar reset/ground-rollout parity, exact isolated C
controller output and subset reset, unaffected simulations during autoreset,
reward phases, and expert/airborne control. Both scalar and batched physics
are compared using MuJoCo 3.14.0. These checks do not establish equivalence
between the 3.14 training engine and the existing 3.10 GUI engine; deployment
still needs the existing policy evaluation and C inference checks.

Measured on this 22-logical-CPU machine, with existing GUI processes left
running. Each backend collects 25,600 transitions (16 x 1,600 vector steps).
Rollout timing includes automatic episode warmup/resets, but excludes initial
construction/reset, policy inference and PPO gradient updates.

| Backend | Transitions/s | Initial Reset | Completed Episodes |
| --- | ---: | ---: | ---: |
| Serial Python, isolated controllers | 2,581 | 3.084 s | 17 |
| SubprocVecEnv, 16 processes | 8,578 | 0.511 s | 17 |
| mjbatch, 16 environments / 8 threads | 8,202 | 0.521 s | 18 |

mjbatch gives about 3.18x the serial rollout rate in this measurement. The
correct process backend is slightly faster here; mjbatch does not imply an
automatic speedup over all process counts. Testing 16 mjbatch threads gave
7,141 transitions/s, so 8 threads is the current recommended setting.
Episode counts may diverge over long trajectories due to floating point and
complete-state reset differences. This is a throughput check, not evidence
of an improved learned policy or convergence.

Artifacts: `logs/rl_training/benchmark_16_threads8.json` and per-run logs.

An actual PPO smoke run completed 8,192 transitions with 16 environments,
128 steps/environment/rollout and two optimizer epochs. It saved
`logs/rl_training/mjbatch_smoke.zip` and reported about 3,944 transitions/s
including policy inference and updates. Reloading that checkpoint with four
environments completed another 1,024 transitions (9,216 cumulative) and saved
`mjbatch_resume_smoke.zip`. Logs use the same filename prefixes. These short
runs verify training/save/resume, not jump quality or policy convergence.
