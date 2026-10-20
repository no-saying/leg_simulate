# Slope Spin Verification

## Setup

36 independent simulations: slopes 0, 3, 5, 8, 10 and 15 degrees; long,
middle and short manual heights (extension -0.04, 0 and +0.04 m); both spin
directions. Positive slope rises toward world +X. Gravity remains vertical.
Scattered balls are moved away. Floor sliding friction is 1.0.

Each trial starts on its slope, requests height at 3 s, spins from 10 to 15 s,
and releases the command from 15 to 20 s. Initialization rotates the robot
and floor about the original ground origin, avoiding artificial penetration.
MuJoCo's floor `geom_sameframe` optimization is disabled for these tests;
the actual world plane normal is asserted against the requested slope.
Slope changes are restricted to the test process. No control gains or user
GUI processes were changed.

Displacement below is maximum horizontal distance from the position at
spin-command onset, during the five-second command. It is neither total
path length nor displacement since startup. Results select the worse of
the two directions at each height. Falls include actual inversion during
startup, command or release, even if the robot subsequently self-rights.

## Results

| Slope | Long: max offset | Middle: max offset | Short: max offset | Falls |
| --- | --- | --- | --- | --- |
| 0 deg | 0.081 m | 0.130 m | 0.252 m | None |
| 3 deg | 0.091 m | 0.122 m | 0.079 m | None |
| 5 deg | 0.339 m | 0.135 m | 0.280 m | None |
| 8 deg | 5.777 m | 4.193 m | 3.651 m | Middle, both directions |
| 10 deg | 8.069 m | 1.489 m | 5.709 m | All heights, both directions |
| 15 deg | 1.713 m | 3.631 m | 11.314 m | All heights, both directions |

At 0-5 degrees, all 18 cases remain upright. Spin pitch/roll peaks remain
below 1.50 degrees, and recovery pitch/roll remains below 1.40 degrees.
Long height turns roughly 237-244 degrees per 5 s; middle/short heights
roughly 364-387 degrees, depending on direction and slope. All these
five-second spin phases report 5000 ms of full LQR control.

The harness's existing centered-spin limit is 0.20 m. Thus flat short
reverse, both 5-degree long cases, and 5-degree short positive spin fail
the centering gate despite not falling. Overall 14/36 cases pass every
gate; this must not be interpreted as 22 falls.

## Separating Slope Holding From Spin

The steeper cases are already unstable before the spin command. Speeds
below are ground-truth horizontal speed immediately before 10 s, obtained
from world XY samples. Positive and negative spin trials share identical
startup states.

| Slope | Long pre-spin speed | Middle pre-spin speed | Short pre-spin speed |
| --- | --- | --- | --- |
| 0 deg | 0.016 m/s | 0.011 m/s | 0.005 m/s |
| 3 deg | 0.009 m/s | 0.020 m/s | 0.039 m/s |
| 5 deg | 0.018 m/s | 0.006 m/s | 0.008 m/s |
| 8 deg | 2.444 m/s | 4.649 m/s | 1.873 m/s |
| 10 deg | 6.010 m/s | 2.817 m/s | 6.293 m/s |
| 15 deg | 1.375 m/s | 3.862 m/s | 4.492 m/s |

At 5 degrees, startup already produces 1.24-1.42 m maximum displacement
before settling. The spin offsets in the first table exclude that motion.
At 8 degrees, pre-spin displacement reaches 12.37-18.90 m, and the robot is
still sliding rapidly. These are not tests of spin from stationary balance.

First observed inversion occurs before spin for 10-degree middle (9.325 s),
15-degree long (7.340 s), and 15-degree short (8.250 s). At 8 degrees middle,
inversion occurs at about 10.260 s after arriving at the command with
4.649 m/s speed. Remaining 10/15-degree cases also invert after spin onset.
Wrong-direction or large yaw accumulated after inversion is not successful
commanded spin.

At 8 degrees, full LQR accounts for only 0-25 ms of the five-second long
trials and 239-293 ms of short trials; middle has none. The 10/15-degree
trials have none. Diagnosing those failures must include the fallback
controller and slope holding, rather than assuming continuous LQR control.

These measurements describe startup on the slope at one initial heading
and one friction setting. They do not establish a universal maximum slope
or behavior when entering a slope from flat ground.

## Reproduction and Artifacts

```sh
make -s -C control build/verify_slip_controls
python3 scripts/tools/verify_slope_spin.py
./control/build/verify_slip_controls spin 0 slope 5
./control/build/verify_slip_controls reverse -0.04 slope 8
```

Summary: `logs/manual_verification/slope_spin.json` and `slope_spin.csv`.
Each case preserves a `slope_spin_*.log` and a
`slip_check_{command}_{extension}_slope_{degrees}.csv` with world XY,
heading, controller mode, fall flag, IMU/encoder residuals and retraction.
The JSON includes pre-spin speed/attitude and first observed inversion time.
The runner returns nonzero when any case fails its gates.
