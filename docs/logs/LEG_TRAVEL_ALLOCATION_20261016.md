# Short-leg roll travel allocation (2026-10-16)

## Problem and Accepted Behavior

At the shortest height, independently clipping the two leg references removes
one side of a roll correction. For example, a requested 24 mm leg difference
becomes only 12 mm if one side is already at its short stop. The roll integral
can then accumulate against travel that the servos cannot provide.

The final stance allocator now moves the mean reference towards longer legs
when the requested roll difference reaches the short stop. This is enabled by
default in both the ordinary and full-range v3 control paths. It runs after
controller, obstacle and RL residual references have been combined.

The extension coordinate is positive towards the short stop. For common
extension `c`, differential `d`, and short-stop limit `hi`, the required common
offset is `min(0, hi - abs(d) - c)`. At `c = hi`, a differential of 12 mm
therefore raises the mean reference by 12 mm: one leg remains at the shortest
reference and the other becomes 24 mm longer. These are servo reference changes;
compliance and support loads determine the measured leg lengths.

- Create travel at up to 0.12 m/s; limit differential to the travel currently
  available during the transition.
- Retain attained short-end travel through turning and unstable recovery,
  including transient airborne estimates. Resetting on every contact loss was
  rejected because it abruptly collapsed the mean height.
- Restore the requested height at up to 0.020 m/s after 0.3 s continuously calm:
  no turn command, no airborne/impact flag, estimated speed below 0.08 m/s,
  roll below 0.02 rad, pitch below 0.035 rad, and each body gyro component below
  0.15 rad/s. Continuing roll demand still reserves the necessary travel.
- Feed requested minus applied differential back to the roll PID. Prevent
  integration farther into unavailable travel or the roll output limit, while
  permitting integral unwind.
- Explicit jump, self-right, fallen and inactive control states bypass this
  stance adjustment. The long stop retains ordinary independent clipping;
  automatic lowering at the longest height is outside this change.

Allocation uses controller commands and sensor estimates, with no access to
the simulated slope angle. Short-leg swing blending continues to use actual
measured length. A trial that retained the minimum-height swing strategy after
raising the body failed multiple slope-spin cases and was removed. LQR model
segments also continue to use actual length; no matrices or policies were
retrained or reidentified.

## Final Controller Measurements

Original production collision model, zero policy actions, shortest extension
`+0.070 m`, heading zero, seed 1000, both spin directions. Episodes last 24 s:
the command starts at 1 s and begins decelerating at 16 s. With 2.76 rad/s
and the 0.8 rad/s^2 reference limit, the command reaches zero near 19.45 s,
leaving about 4.55 s at zero reference. Completion alone does not establish
stationary rotation or successful tracking.

Pure PID, retained allocator and calm recovery gate:

| Slope | Direction | Duration (s) | Roll RMS / peak (deg) | Max drift (m) | End reason |
| --- | --- | --- | --- | --- | --- |
| 0 | - | 24.00 | 0.461 / 1.226 | 0.510 | Completed |
| 0 | + | 24.00 | 0.470 / 0.904 | 0.433 | Completed |
| 3 | - | 24.00 | 1.007 / 1.943 | 1.230 | Completed |
| 3 | + | 23.94 | 0.921 / 2.492 | 3.034 | Recovery drift |
| 5 | - | 24.00 | 1.756 / 2.933 | 1.749 | Completed |
| 5 | + | 10.68 | 1.499 / 2.913 | 0.849 | Rotation stall |
| 8 | - | 24.00 | 2.351 / 3.944 | 2.480 | Completed |
| 8 | + | 24.00 | 2.540 / 4.103 | 1.802 | Completed |

The final PID run completed 6/8 cases. In the matching six-case 0/3/8-degree
recovery comparison, completion improved from 2/6 without allocation to 5/6.
The 3-degree positive case regressed from completion to recovery drift. On
8 degrees, peaks fell from about 7.0/7.2 degrees to 3.9/4.1 degrees, but the
remaining 1.8-2.5 m excursion is not an acceptable in-place spin. Final flat
reference lift returned to about 0.37/0.15 mm; on 8 degrees about 22-24 mm
could remain reserved while roll authority was still needed.

Normal PID/LQR switching completed 5/8 extended cases. The remaining cases
were a 3-degree positive rotation stall and recovery drift on flat reverse
and 8-degree reverse. Roll peaks across the eight cases were 1.07-3.97 degrees;
drift and yaw tracking remain unresolved. This allocator does not repair the
wheel/calf collision or the longest-height wheel-estimator/obstacle feedback
documented in `SPIN_BOUNDARY_20261016.md`.

## Verification and Artifacts

- Native controller build and portable control compilation passed.
- `make -C control check-leg-allocation` passed at 1/2/5 ms steps, covering
  short-stop differential preservation, direction reversal, retained spin
  clearance, bounded restoration, long-stop clipping, jump bypass and range sweep.
- Spin/jump/collider regression: 20 tests passed. The new PID regression checks
  that unavailable travel stops further roll integral accumulation.
- Final native manual rerun passed: W/S each ran 5 s and translated
  1.718/2.671 m; A/D each ran 5 s and rotated +271.8/-273.0 degrees. A separate
  5 s spin rotated 484.6 degrees with 0.058 m maximum excursion and 0.41-degree
  peak roll. E/Q reached measured lengths 0.1506/0.0707 m. These ordinary-range
  tests do not validate the full-range shortest slope endpoint.
- Ruff and `git diff --check` passed. The existing GUI was not restarted.

Artifacts under `logs/rl_training/`:

- Retained PID: `spin_leg_allocation_v3_recovery_pid.*` and
  `spin_leg_allocation_final5_pid.*`.
- Retained switching controller: `spin_leg_allocation_final_lqr.*`.
- Baseline: `spin_leg_allocation_before_recovery_pid.*`.
- Build/tests: `spin_leg_allocation_accepted_build.log` and
  `spin_leg_allocation_accepted_tests.log`.
- Final native motion check: `spin_leg_allocation_accepted_native.log`.
- Trace columns `left_ext_applied`, `right_ext_applied`, `height_lift` and
  `roll_limited` expose the applied allocator outputs in metres.

The files named `spin_leg_allocation_final_pid.*` and
`spin_leg_allocation_final2_lqr.*` belong to the rejected minimum-height swing
strategy trial. Their names do not indicate accepted results.

Reproduce the final PID diagnostic from the project root:

```sh
.venv-mjbatch/bin/python scripts/tools/trace_spin_control.py --pid-only \
  --slopes 0 3 5 8 --extensions 0.070 --duration 24 --recovery-seconds 8 \
  --out logs/rl_training/leg_allocation_recheck
```

Omit `--pid-only` to check normal controller switching. To compare without the
allocator, add `--define LEG_ROLL_ALLOCATION_ENABLE=0`.
