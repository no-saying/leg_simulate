# V6 Fixed-Origin Spin

## Acceptance

V5's no-fall completion metric does not satisfy position holding. The spin must
stay near its original start throughout motion and return after excursions.
The origin is fixed after terrain placement and warmup, and never follows drift.

Initial engineering thresholds (the same for every leg length):

- Working radius: 0.30 m from the original base XY position.
- Return: reenter 0.15 m and remain there for 0.5 s within 6 s of crossing 0.30 m.
- Escape: radius above 1.0 m or return timeout is a failed trial.
- A pass requires no fall/stall/escape, at least 90% of time inside 0.30 m,
  no pending return, final radius and the last five seconds inside 0.30 m,
  and signed cumulative rotation at least 80% of its reference.
- Training episodes last 30 s; heldout evaluation lasts 60 s. Spin continues
  through the end of the episode; there is no final stopped recovery phase.

Reports separate the mapped short/middle/long leg bands, and record return
attempts/successes, final/mean/peak distance, occupancy, unrecovered excursions,
and odometry error. No-fall completion remains available but is not acceptance.

## Observations and Reward

The V5 actor had no persistent start-relative position observation. Its eight
frames represent approximately 0.16 s, so increasing position penalties alone
does not tell it where to return after a longer excursion.

V6 appends four features to each frame: estimated start-relative planar position
and planar velocity, both expressed in the current heading frame. The contract
is 56 x 8 = 448 float32 inputs and the same five residual actions. Position is
integrated from yaw/leg/pitch-compensated wheel kinematics fused with IMU
acceleration. Both gyro yaw differential and off-center IMU centripetal
acceleration are removed. Native observation slot 19 is a controller reference,
not a measured velocity; the estimator does not treat it as a measurement.
World XY position/velocity and terrain/contact truth never enter these actor
features or generate residual commands.

Reward uses true start-relative distance at every 1 ms physics step:

- Position: `-4 * min((radius / 0.30)^2, 16)`.
- Outside boundary: `-12 * min((max(radius - 0.30, 0) / 0.30)^2, 16)`.
- Outward radial speed above 0.15 m: `-4 * min((max(radial_speed, 0) / 0.20)^2, 9)`.
- Signed radial progress: `-4 * min(radius / 0.30, 4) * radial_speed / 0.30`.
- Additional ongoing outside cost grows with the excursion's age.

At rest 0.60 m away, position plus boundary costs alone are -28 per second;
stopping there cannot remove the penalty. Inward motion receives signed progress
credit while the position cost remains. V5's speed penalty and peak charge are
replaced so return speed is not charged as severely as outward drift. Roll,
pitch, yaw, directional leg reserve and smoothness costs remain. All leg bands
use the same fixed-origin costs; V5's mapped yaw weighting is retained.
Premature failures cost `10 + 500 * remaining_seconds`. A uniform factor 0.05
scales the entire training reward, including failures, for critic conditioning;
episode reports retain raw returns and reward totals.

## Initialization and Run

The V5 actor is copied into V6. Each old 52-feature frame maps into the first
52 columns of its new 56-feature frame, and new position/velocity columns start
with zero weights. Predictions initially match V5 within numerical precision.
The critic and optimizer start fresh for the new observations/reward. This is
a new experiment, not resumption of the old V5 objective.

```bash
.venv-mjbatch/bin/python -u scripts/train_spin_rl.py \
  --terrain anchor --steps 1048576 --envs 32 --threads 4 --slope 8 \
  --no-curriculum --rollout 512 --epochs 10 --learning-rate 0.0001 \
  --warm-start logs/rl_training/spin_standard_v5_resume393k_to524k.zip \
  --eval-interval 131072 --checkpoint-interval 32768 \
  --out logs/rl_training/spin_anchor_v6_1m
```

Full leg travel, finite ramp/flat junctions, PID/LQR handovers and dropout
randomization remain. Half of training episodes additionally receive a random
0.10-0.25 m/s planar velocity impulse at 7-12 s. Its parameters do not enter the
actor. The matched initial-policy evaluation uses the same V6 task, thresholds,
disturbances, seeds and 240 heldout cases as subsequent checkpoints.

The 512-transition pipeline completed finite PPO updates and 448 -> 5 inference.
All 43 tests in the anchor, original spin, mixed terrain, leg travel, standard
mapping and live GUI suites passed. Launch commands, PIDs and logs are recorded
in `logs/rl_training/spin_anchor_v6_jobs.json`.
Three 8-degree ramp trials at 47/117/210 mm reduced observed peak odometry error
from 0.24/0.24/0.26 m with slower fusion to 0.086/0.053/0.050 m. This is a short
estimator check, not evidence of learned return behavior. The unchanged V5 actor
failed the new bounds in those trials.

Wheel/IMU odometry still cannot guarantee absolute localization during sustained
common slip. True position remains the acceptance measure, and estimator error
is reported. No trained V6 checkpoint is declared usable until the bounded spin
and return criteria pass across leg bands and terrain boundaries.
