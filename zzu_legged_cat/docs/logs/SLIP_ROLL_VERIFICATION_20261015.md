# Slip Detection and Obstacle Roll Response

## Changes

- Slip detection compares IMU integration with per-wheel base velocity estimates.
  Base velocity estimates remove leg movement, body rotation and the expected yaw
  differential. IMU integration rotates acceleration into world heading coordinates
  and corrects the IMU mounting lever arm. Control uses sensors and encoders only.
- Linear and yaw slip flags are separate. Yaw-only slip does not replace the
  longitudinal speed estimate. Normal spinning is not classified as common slip.
- During linear slip, encoder updates and velocity-filter bias learning stop.
  Translational commands are softened; common balance torque remains available.
- Euler roll derivative is used for roll stabilization and LQR roll feedback.
  Raw X/Y/Z gyro measurements remain available for physical rotation compensation.
- A lateral support disturbance enables additional common leg retraction.
  On v3, positive extension means retraction; CAD Left is on negative control Y.
  Negative roll commands Left retraction, and positive roll commands Right retraction.
  Adding a positive common term favors retracting the raised side over extending
  the low side. Slew limits mean this is a priority, not a strict one-sided lock.
- Extra common retraction is limited to 20 mm and remaining manual leg travel.
  It rises at at most 0.12 m/s and restores at 0.02 m/s after a 0.30 s hold.
  The user's height target is preserved. Jump/self-right/reset clear the reflex.
  At minimum leg length, mechanical limits restrict further retraction.
- Slip prediction survives airborne/impact periods. After prolonged slip,
  acceleration/yaw agreement permits encoder reacquisition.
  Constant common slip cannot be independently confirmed with IMU and encoders
  alone; reacquisition uses an assumption about restored grip.
- Dashcam logs include inertial velocity, slip residuals, blend weights,
  obstacle retraction and Euler roll derivative.

## Verification

The physical test settles until 10 s, applies the command for 5 s, and records
release until 20 s. Balls are freely rolling 42 mm spheres, placed in wheel paths.
All other balls are moved away only in the test's in-memory model. Actual contact,
world velocity and displacement are used for validation, never for control.

Pass criteria: command pitch/roll below 10 degrees; recovery pitch/roll below
2 degrees during 17-20 s; measurable translation/rotation; spinning excursion
below 0.20 m. A cleared fallen flag alone does not constitute a pass.

| Test | Max Pitch (deg) | Max Roll (deg) | Common Retraction (mm) | Result |
| --- | ---: | ---: | ---: | --- |
| Middle W, Left ball | 7.10 | 4.35 | 8.68 | Pass |
| Middle W, Right ball | 7.40 | 4.04 | 13.33 | Pass |
| Middle S, Left ball | 6.03 | 2.98 | 13.54 | Pass |
| Middle S, Right ball | 6.19 | 3.15 | 13.70 | Pass |
| Highest W, Left ball | 7.47 | 5.18 | 11.98 | Fail: recovery pitch 3.82 deg |
| Highest W, Right ball | 7.31 | 5.60 | 12.32 | Fail: recovery pitch 5.19 deg |
| Shortest W, Left ball | 6.02 | 4.47 | 0.46 | Pass |
| Shortest W, Right ball | 6.08 | 4.56 | 0.50 | Pass |
| Middle spin, flat | 0.52 | 0.33 | 0.00 | Pass: 376.9 deg, 0.074 m excursion |
| Highest spin, flat | 0.36 | 0.24 | 0.00 | Pass: 237.8 deg, 0.086 m excursion |
| Shortest spin, flat | 0.36 | 0.58 | 0.00 | Pass: 381.4 deg, 0.098 m excursion |
| Middle spin, Left ball | 0.95 | 0.36 | 0.00 | Fail: 0.226 m excursion |
| Middle reverse spin, Right ball | 0.75 | 0.41 | 0.00 | Fail: 0.208 m excursion |
| Middle W, two balls | 40.67 | 3.62 | 8.54 | Fail: large pitch |
| Middle S, two balls | 16.52 | 4.51 | 12.31 | Fail: large pitch |
| Middle spin, friction 0.02 at 11-12 s | 2.93 | 0.36 | 0.00 | Fail: 0.781 m excursion |

For the low-friction spin, speed MAE is 0.1017 m/s with slip correction versus
0.0719 m/s with the original wheel/body estimate. Inertial substitution is still
worse in this case. Spin drift and simultaneous two-wheel obstacle traversal
are unresolved; this change must not be described as complete terrain recovery.

The scattered-ball W/A/S/D (5 s each), E/Q to the height endpoints, and spin (5 s)
sequence passed the existing motion checks. The spin rotated 372.9 degrees with
0.050 m excursion. See `logs/manual_verification/slip_roll_sequence.json`.

Observer tests pass at 1/2/5 ms steps, including common/yaw separation, progressive
wheel slip, airborne preservation, contact recovery, and filter reset/bias freeze.
Independent quaternion integration verifies pitch and roll derivatives over
567 poses/rates, with worst error below 0.000006 rad/s. Portable compilation and
the PID identification bridge rebuild pass. No LQR model was reidentified.

## Reproduction

```sh
make -C control all build/verify_slip_controls check-slip-observer check-attitude-rates check-portable
./control/build/verify_slip_controls W 0 leftball
./control/build/verify_slip_controls W -0.04 rightball
./control/build/verify_slip_controls spin 0 ice
```

Terrain options: `flat`, `ball`, `leftball`, `rightball`, `ice`. Height extension
-0.04 is highest, +0.04 shortest. CSV logs are under `logs/manual_verification/`.
Failed physical criteria return a nonzero exit code and print `VERDICT FAIL`.
