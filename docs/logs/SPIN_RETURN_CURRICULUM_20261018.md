# Return Curriculum Experiment, 2026-10-18

## Purpose

The previous 32,768-transition history/exact-map pilot passed 11/30 strict
screening cases but returned successfully on 0/18 excursions. Its roll RMS
was 1.043 degrees and mean maximum drift was 0.530 m. This experiment tests
whether explicit return-velocity tracking and staged tasks improve recovery.

## Active Controller

Residual PID/LQR control, five residual outputs, sensor-only 448-value actor,
23 privileged critic features, history compression and exact encoder linkage
mapping are retained. Motor PI is disabled. The original ideal motor model
is used. Native controller gains and torque limits are unchanged.

`scripts/spin_return_env.py:ReturnTrainingSpinEnv` is opt-in through
`scripts/train_spin_rl.py --return-curriculum`. Existing evaluation and GUI
behavior do not select this environment automatically.

## Reward and Task

The reference velocity points toward the target with speed
`0.25 * tanh(distance / 0.25)` m/s. A Huber tracking cost gives a wide initial
tolerance and non-flat tails, then tightens with the course. This is a reward
reference, not an extra feedback controller or injected actuator command.

Position, outward-motion, signed-progress, roll, pitch, yaw, contact and action
smoothness terms remain. The generic total-speed penalty is removed in the
new training environment so intended return velocity is not separately
penalized. Failure penalties remain unchanged in this first experiment.

| Stage | Spin Speed Fraction | Return Deadline | Velocity Tolerance | Maximum Slope |
| --- | --- | --- | --- | --- |
| Return direction | 0.45 | 15 s | 0.20 m/s | 3 degrees |
| Return while spinning | 0.75 | 10 s | 0.13 m/s | 5 degrees |
| Bounded spin | 1.00 | 6 s | 0.08 m/s | 8 degrees |

All stages retain the full 47-210 mm leg range, both spin directions, all five
terrain locations and randomized PID/LQR handovers. Friction samples cover
0.3-1.2. No physical assistance is applied to make the robot return.

75%, 60% and 35% of episodes respectively receive a known relative target
offset of 0.20-0.38/0.40 m. Both the reward target and the sensor odometry
origin receive that same commanded offset. The robot is not teleported;
true position/velocity are not copied into actor observations. Other episodes
start at the normal zero-offset target. Physical push disturbances remain.

Stage changes affect new episodes only. At advanced stages, 20% of resets use
earlier stage yaw, tolerance, deadline and offset settings. Terrain sampling
retains the earlier slope range within the expanded range.

## Promotion and Evaluation

Every 32,768 transitions, after the PPO update, evaluate:

- Original strict 30-case screen: full spin speed, no initial offset, 6 s
  return deadline, 30 s duration, 8-degree scene, all five terrain locations,
  all three leg lengths, both directions, LQR handover, seed 1000.
- Fixed return gate: 30 cases at the stage slope, 0.35 m commanded target
  offset, stage yaw/deadline, no pushes, 30 s duration, seed 2000.

Promotion requires two consecutive gate passes. A gate pass requires at least
15 return attempts, at least 70% successful returns, at least 70% completed
episodes, mean yaw ratio >= 0.8 and roll RMS <= 3 degrees. Each mapped leg
band must also complete >= 50% of cases with mean yaw ratio >= 0.7.

Gate results are training feedback, not held-out acceptance. Strict screen
results are kept separately and are never relabeled as full 240-case success.
Reward totals from different training tasks are not directly comparable.

## Controlled Continuations

Both runs start from
`logs/rl_training/spin_residual_history_exact_pilot32k_20261018.zip`.
Actor weights, including the learned history branch, are preserved exactly.
Critic and optimizer are reset in both arms because the candidate changes the
reward. Each arm receives 65,536 new transitions, seed 20, 32 environments,
8 threads, 256-step rollouts, 3 PPO epochs, learning rate 3e-5, gamma 0.999,
GAE lambda 0.98, ideal motors and exact linkage mapping.

| Arm | Service | Output Prefix |
| --- | --- | --- |
| Existing reward | `spin-return-baseline-20261018-ab64k.service` | `logs/rl_training/spin_return_baseline_ab64k_20261018` |
| Return curriculum | `spin-return-curriculum-20261018-ab64k.service` | `logs/rl_training/spin_return_curriculum_ab64k_20261018` |

This compares the complete curriculum package with the prior training method;
it does not isolate the causal contribution of each changed component.

Checkpoints are saved every 16,384 transitions. Per-stage gate data is saved
as `.return_gate.json`; the latest promotion state is `.curriculum.json`.
For a future resume, use `--return-stage` to restore the recorded stage;
the consecutive validation streak starts fresh, conservatively requiring two
new passes. There is no automatic million-step continuation or GUI deployment.

## Verification

43 tests passed across return, asymmetric actor, mixed terrain, base spin,
history migration and anchor behavior. Tests cover inward/limited/braking
references, directional cost, validation gates, per-episode stage transitions,
actor isolation from simulator truth, and exact history-actor migration with
fresh critic weights. The default strict anchor deadline is regression tested.
