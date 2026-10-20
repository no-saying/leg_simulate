# Spin optimization (2026-10-15)

This follow-up records the final deployed changes after
`LQR_VERIFICATION_20261015.md`. Values below come from headless simulation.

## Rotation and balance

Centripetal acceleration is proportional to yaw rate squared times distance
from the rotation axis. A displaced mass or IMU therefore experiences
acceleration during a spin even when the wheel axle midpoint stays nearly
stationary. Gyroscopic torques are another effect. These effects do not by
themselves provide vertical support, and turning off pitch feedback would
remove the response to real imbalance.

A separate, concrete control error was found: the controller treated mounted
body Y gyro as the derivative of Euler pitch while yawing with nonzero roll.
For ZYX Euler angles, the correct relation is:

```text
pitch_dot = cos(roll) * gyro_y - sin(roll) * gyro_z
```

Body Y gyro contains a yaw contribution when the body is tilted. Feeding this
contribution to pitch damping and axle-motion compensation can create common
wheel torque and a leg correction even without a corresponding pitch change.
The new portable `attitude_rates.h` removes that contribution during commanded
turning and the spin braking phase. Compensation increases continuously with
measured yaw gyro magnitude and is complete at 0.4 rad/s.

The existing accepted PID fit used body Y gyro under no yaw command. Its
original state is retained outside turning, preserving the stationary and
straight-driving plant and avoiding a regression observed when Euler rate
was substituted in every operating state. The LQR pitch state, PID frame,
supervisor, and identification bridge share the rotation compensation.
Pure PID identification collection keeps the original no-yaw rate state;
the bridge receives raw gyro and applies control conversion exactly once.

Spin intent is now exposed directly by the balance API. The parking swing
integral is frozen throughout a requested spin, including the reduced rate
at maximum leg length. Previously the freeze used a rate threshold; the
height-dependent reduction could incorrectly re-enable the integral.

No new identification data or centripetal force feedforward model was produced.
The accepted PID model and synthesized gains remain the same. The yaw channel
still uses the analytical scalar rate LQR described in the earlier record.

## Five-second comparison

Identical scripts settled each height to t=10 s, spun until t=15 s, then
released. The table compares positive spin maximum excursion from its entry
position. Reverse spin was also tested at every height.

| Actual length (m), approximately | Before (cm) | Final (cm) |
| --- | --- | --- |
| 0.151 | 16.2 | 11.9 |
| 0.131 | 20.2 | 10.5 |
| 0.111 | 13.9 | 8.6 |
| 0.091 | 18.3 | 11.5 |
| 0.071 | 13.8 | 8.9 |

All 10 final positive/reverse five-second cases passed. Largest excursion:
0.122 m. Largest pitch: 0.85 degrees. Mean LQR blend was 1.0 in each spin
window. Reverse-direction results are not uniformly better than baseline.

## Extended and regression tests

- All 10 positive/reverse 20-second spins passed. Largest excursion 0.131 m;
  largest final-second body path speed 0.043 m/s; no falls. This checks that
  a small endpoint displacement is not simply a large completed circle.
- All 18 stationary heights and 36 pitch pulse recoveries passed. Stationary
  length range approximately 0.0707-0.1505 m; final 5 s maximum net displacement
  0.002 m, maximum final-second path speed 0.013 m/s. Recovery maximum pitch
  0.20 degrees and maximum final-second path speed 0.016 m/s.
- Highest/shortest W/A/S/D/spin tests passed. The five-second W/S travel was
  0.796/3.320 m at the highest and 1.377/3.502 m at the shortest height.
- Requested sequence passed: W 1.871 m, A +195.1 degrees, S 2.863 m,
  D -206.2 degrees, E to 0.1505 m, Q to 0.0707 m, spin +374.8 degrees.
  Sequence spin maximum excursion 0.043 m; net displacement 0.021 m;
  release-window net displacement 0.002 m.
- During sequential driving, mean LQR fractions were
  W/A/S/D=0.907/0.497/0.677/0.470. The posture window still selects PID outside
  its limits; the movement test does not demonstrate uninterrupted LQR.
- Quaternion integration independently verified the Euler rate formula in
  567 cases, maximum error 0.000004096 rad/s. A body yawing at fixed tilted
  posture has zero Euler pitch derivative despite nonzero body Y gyro.
- Compilation, portable control-core build, Python syntax checks, and
  `git diff --check` passed. The pure PID bridge was rebuilt from current source.

## Rejected Trials and Remaining Limits

Holding the stationary leg trim alone increased drift at several heights.
Adding a pitch target correction did not resolve it. Replacing the spin
reference with bounded velocity or two-dimensional odometry correction also
failed the existing excursion gate in some cases. These trials were removed.
Independent left/right leg damping changed the identified plant and worsened
intermediate-height drift, so the existing servo schedule and damping were
restored.

Leg swing feedback remains active; the legs are not rigidly locked throughout
a spin. The logs now distinguish its reference from measured left/right
swing and distinguish raw gyro, exact Euler pitch derivative, and the rate
used by control. Reference travel can still reach about 0.025 m across a spin
startup window. Differential leg motion and some translational oscillation
remain. Spin excursion is reduced, not eliminated. Braking travel and W/S
speed asymmetry remain as described in the earlier record.

## Reproduction and Logs

```sh
make -s -C control -j4
make -s -C control check-portable
gcc -O2 -Wall -Wextra -Icontrol/include scripts/tools/test_attitude_rates.c -lm -o /tmp/lqr_attitude_rate_test
/tmp/lqr_attitude_rate_test
python3 scripts/tools/verify_manual_controls.py spin
python3 scripts/tools/verify_manual_controls.py spin-long
python3 scripts/tools/verify_manual_controls.py stability-all
python3 scripts/tools/verify_manual_controls.py range
python3 scripts/tools/verify_manual_controls.py sequence
```

Final logs use `spin_*`, `spin-long_*`, `static_*`, `pulse_*`, `u_*`, `d_*`,
and `sequence` in `logs/manual_verification/`. Baseline logs are named
`spin_before_*`. Other trial files document rejected candidates and do not
represent the final controller. CSV positions are rounded to millimetres;
reported small path speeds include quantization noise. No GUI validation
was performed in this follow-up.
