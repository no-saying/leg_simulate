# LQR control verification (2026-10-15)

Baseline record. Subsequent spin changes and final follow-up results are in
`SPIN_OPTIMIZATION_20261015.md`.

## Control changes

- Full LQR owns the five output channels inside the posture window. Leg
  length selects and interpolates model parameters; it does not gate entry.
  PID takes over outside the window, with a 0.4 s blending transition and
  immediate fallback at the configured dangerous posture limits.
- Wheel speed references include leg and pitch axle motion. Releasing a
  command retains a valid zero body-speed request. Stop and heading anchors
  are updated during command transitions to avoid returning to stale targets.
- Full LQR uses the automatic leg servo gain schedule used during PID model
  collection. Pressing a movement key no longer switches this plant to the
  stiffer PID manual schedule. Deep short-leg swing freezing remains active.
- Parking swing uses the interpolated identified stationary input, with a
  bounded speed correction. Pitch reference includes the identified pitch
  trim associated with the stationary wheel input.
- The full wheel feedback row is synthesized again from the existing accepted
  pure PID fit, assuming no guaranteed free pitch-rate damping. This removes
  negative pitch-rate feedback found in intermediate leg-length segments.
- Yaw uses an analytical scalar yaw-rate LQR. It is not a newly identified
  coupled yaw model. No new PID identification data was collected for this
  verification.
- CSV files have unique names and include final wheel controls, measured
  wheel torques, command references, fall state, and LQR blend fraction.

## Model provenance

Source: `logs/three_loop_model_v3_20261007_213902.json`.
The report has `pid_only=true`, `accepted=true`, 18 segments, and a 0.005 s
model period. The generated damping header records the source SHA256.
The generator checks closed-loop eigenvalues under both the identified and
conservative reduced pitch models. These checks do not prove stability of
the complete nonlinear coupled robot; simulation results below are the
additional evidence.

## Completed simulation tests

All listed suites completed successfully. Motion is measured from world XY
coordinates and unwrapped yaw in the log. Encoder velocity alone is not used
as proof of translation. World XY is not supplied to the controller.

| Test | Result |
| --- | --- |
| 18 heights, each static for 20 s | Actual stationary lengths 0.0707-0.1505 m; largest net displacement in the final 5 s: 0.002 m; largest final-second path speed: 0.013 m/s |
| Positive and negative pitch pulses at every height | 36 cases, each 0.15 Nm for 0.15 s at t=10 s; no falls; largest pitch in t=10-13 s: 0.38 degrees |
| Recovery after pulses | In t=17-22 s: largest pitch 0.20 degrees, largest final-second path speed 0.016 m/s, largest net displacement 0.015 m |
| Static and disturbance ownership | Mean LQR blend fraction 1.0 in every measured window |
| Individual W/A/S/D, each 5 s | W net displacement 1.871 m, S 3.188 m; A +209.0 degrees, D -208.8 degrees; sustained signed velocity and yaw-rate gates passed |
| Highest and shortest heights, W/A/S/D/spin each 5 s | All 10 cases passed; sustained A/D yaw rates approximately 0.86-0.90 rad/s; no falls |
| Sequential W/A/S/D, E highest, Q shortest, spin 5 s | W 1.871 m, A +214.7 degrees, S 2.525 m, D -189.7 degrees; E 0.1506 m, Q 0.0706 m; spin +376.9 degrees and final-second yaw rate 2.109 rad/s |
| Sequence spin and release | Spin excursion about 0.09 m; release-window displacement 0.002 m |

Static gates are 0.08 m net displacement and 0.08 m/s final-second path speed.
Recovery gates are 0.08 m/s final-second path speed and 1 degree pitch.
Movement gates require signed final-second forward speed above 0.3 m/s and
path length above 0.5 m. Rotation gates require signed final-second yaw rate
above 0.4 rad/s and rotation above 60 degrees. Spin excursion limit is 0.2 m.
The recovery check verifies settling, not return to the original position.
CSV world positions are rounded to millimetres, so small path speeds include
quantization noise.

## Remaining limitations

- W/S speed is asymmetric, especially at the highest height: final-second
  speeds were approximately -0.576 and +1.365 m/s.
- Stopping travel is still substantial. Individual W/S release windows
  recorded approximately 1.12/1.69 m travel before settling. Sequential A/D
  displacement also includes braking after the preceding translation.
- Rotation is sustained, but it is not perfectly stationary: spin excursions
  across the tested cases reached approximately 0.16 m. Small idle yaw motion
  also remains (up to 1.8 degrees across a measured static 5 s window).
- Driving leaves the posture window at times. Sequential W/A/S/D mean LQR
  fractions were 0.907/0.551/0.819/0.475, so these results demonstrate the
  complete LQR/PID switching controller, not uninterrupted LQR driving.
- The GUI was not launched or validated in this verification.

## Reproduction

```sh
make -B -s -C control -j4
make -s -C control check-portable
python3 scripts/tools/verify_manual_controls.py
python3 scripts/tools/verify_manual_controls.py sequence
python3 scripts/tools/verify_manual_controls.py range
python3 scripts/tools/verify_manual_controls.py stability-all
```

Gain regeneration from the accepted PID report:

```sh
python3 scripts/tools/synthesize_lqr_pitch_damping.py logs/three_loop_model_v3_20261007_213902.json
```

Detailed CSV, stdout logs, and per-window JSON summaries are in
`logs/manual_verification/`. Final checks also passed Python syntax
compilation for the three tooling scripts and `git diff --check`.
