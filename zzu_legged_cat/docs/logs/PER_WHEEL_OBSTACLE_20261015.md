# Independent Wheel Obstacle Handling

This revision uses per-wheel encoder/IMU disagreement, without a DOB.
It supersedes the measurements in `SLIP_ROLL_VERIFICATION_20261015.md`.

## Detection and Response

- Left/right inputs already remove each leg's movement and measured body
  rotation, including the expected differential velocity during yaw.
- Each wheel has its own signed velocity residual, acceleration residual,
  entry timer, exit timer and active flag. Residuals are not compared directly
  between the two raw wheel speeds.
- Entry: absolute speed disagreement above 0.30 m/s, or disagreement above
  0.10 m/s with acceleration disagreement above 6 m/s^2, sustained for 20 ms.
  Above 0.70 m/s the existing fast threshold uses 12 ms.
- Exit: disagreement below 0.16 m/s for 150 ms. Invalid/airborne observations
  cannot start a new event or accumulate entry time.
- An affected wheel requests 12 mm additional retraction on that leg, held
  for 0.20 s after its flag clears. Both wheels can request retraction while
  roll is zero. Retraction does not wait for a roll threshold.
- Existing roll stabilization remains active. Its support correction and
  per-wheel requests are combined using the larger requested retraction.
  Each leg is limited to remaining travel; retract/restore slew rates remain
  0.12/0.02 m/s. Manual height targets are preserved.
- Common/differential conversion is performed once: common=(left+right)/2,
  differential=(left-right)/2. Full LQR receives these as pose references.
- An abnormal wheel cannot pull the velocity estimate toward its encoder.
  When only one wheel is flagged, the other non-suspect wheel can anchor the
  short-term inertial prediction. Common velocity disagreement switches
  feedback earlier after an individual wheel has confirmed the event.
- Jump, self-right, disabled control and reset clear retraction state.
- CSV logs expose both residuals, both flags, both retraction commands and,
  in the physical test only, per-wheel real ball contacts.

## Physical Tests

Commands run for 5 s after settling and height adjustment. Ground truth is
used only for verification. Per the user's updated ball-traversal requirement,
ball cases require no overturn, no controller fall flag or numerical warnings,
and recovery below 2 degrees. Overturn is checked throughout command and
release using pitch/roll reaching 90 degrees. Peak angles are still reported.
Flat cases retain the 10-degree command limit. Motion and spin excursion
<0.20 m checks remain. Two-ball cases also require overlapping real contacts.

| Case | Peak Pitch / Roll (deg) | Result |
| --- | --- | --- |
| Middle W, Left ball | 6.79 / 3.68 | Pass; Left triggers first, 76 ms after contact |
| Middle W, Right ball | 7.20 / 3.70 | Pass; Right triggers first, 62 ms after contact |
| Highest W, Left ball | 7.21 / 4.52 | Pass; recovery pitch 1.92 deg |
| Shortest W, Right ball | 5.95 / 4.81 | Pass; travel saturation limits affected-side retraction |
| Middle W, two balls | 38.63 / 4.35 | Pass no-overturn criterion; recovery pitch 0.57 deg |
| Middle S, two balls | 12.94 / 3.13 | Pass no-overturn criterion; recovery pitch 0.62 deg |
| Middle spin, flat | 0.52 / 0.33 | Pass; 376.9 deg, 0.074 m excursion, no flags/retraction |
| Middle reverse spin, flat | 0.78 / 0.39 | Pass; -379.5 deg, 0.136 m excursion, no flags/retraction |
| Highest spin, flat | 0.36 / 0.24 | Pass; 237.8 deg, 0.086 m excursion, no flags/retraction |
| Shortest spin, flat | 0.36 / 0.58 | Pass; 381.4 deg, 0.098 m excursion, no flags/retraction |
| Middle spin, balls | 0.91 / 0.37 | Pass; real contacts, 0.144 m excursion, residuals below trigger |
| Middle spin, friction 0.02 at 11-12 s | 0.91 / 0.54 | Pass; 372.5 deg, 0.134 m excursion |
| Middle moving turn, flat | 3.33 / 0.97 | Pass; actual tangential slip also recorded |

Single-side contact can subsequently disturb the other wheel and trigger that
wheel independently. First-trigger times in the table refer to the directly
contacting side. A physical contact without large disagreement need not trigger.

The low-friction spin velocity MAE is 0.0447 m/s, versus 0.0696 m/s for the
wheel/body estimate. This is one deterministic test, not proof of all-terrain
recovery. Simultaneous two-wheel traversal recovers in these middle-height
cases; the large W pitch transient remains visible in the measurements.

### Simultaneous Contact and Displacement

Two separate 42 mm balls are placed in the wheel paths. First-contact times
differ by 5 ms for W and 1 ms for S. Both wheels are simultaneously in ball
contact for 133 ms (W) and 192 ms (S). The updated harness records world XY and
heading, and measures displacement through the full 10-20 s event window.

| Middle-height case | Maximum lateral offset | Release maximum / net displacement | Final planar speed |
| --- | --- | --- | --- |
| W, two balls | 0.0446 m | 0.1749 / 0.1701 m | 0.0390 m/s |
| S, two balls | 0.0782 m | 1.2617 / 1.1816 m | 0.0014 m/s |
| W, flat reference | 0.0220 m | 1.0456 / 1.0079 m | 0.0450 m/s |
| S, flat reference | 0.1062 m | 1.4242 / 1.1801 m | 0.0021 m/s |

Lateral offset is perpendicular to the initial heading, measured from the
command-start position. Release displacement is measured from the position
when the command is released at 15 s, until 20 s; it includes braking and
balance recovery. The ball and flat cases have different release velocities,
so subtracting their stopping distances does not isolate obstacle-only drift.
The previously reported 1.172 m and 1.744 m are commanded travel during
10-15 s, not unintended drift. W still has 0.039 m/s planar speed at the end
of the measured window; upright recovery does not imply complete standstill.

Logs: `/tmp/perwheel_drift_{W,S}_{ball,flat}.log`; detailed CSVs are
`logs/manual_verification/slip_check_{W,S}_+0.000_{ball,flat}.csv`.

The W/A/S/D sequence (5 s each), full E/Q height adjustment and 5 s spin passed
existing motion checks. Sequence spin excursion was 0.056 m; all recorded
command pitch/roll peaks were below 4 degrees. See
`logs/manual_verification/per_wheel_sequence.json`.

Unit tests pass for independent side timers, transient rejection, common/yaw
separation, matched real acceleration/turning, airborne preservation, reset and
1/2/5 ms steps. Portable compilation, quaternion rate tests and PID bridge build
pass. No LQR gains or identified matrices were changed in this revision.

```sh
make -C control all build/verify_slip_controls check-slip-observer check-attitude-rates check-portable
./control/build/verify_slip_controls W 0 leftball
./control/build/verify_slip_controls W 0 rightball
./control/build/verify_slip_controls W 0 ball
./control/build/verify_slip_controls spin 0 ice
```

Failed physical criteria print `VERDICT FAIL` and return nonzero.
