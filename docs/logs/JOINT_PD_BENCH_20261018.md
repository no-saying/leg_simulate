# RL actuator PD baseline

## Configuration

`config/rl_joint_pd.json` records the simulated actuator baseline. Load with
`JointPDGains.load(path)` and execute `JointTargetPD.control` every 1 ms.
The new RL environment still needs to connect this tracker; no training was
started and no existing native PID/LQR gains were changed by this bench.

| Actuators | Physical torque request | Gains |
| --- | --- | --- |
| Four driven leg joints | `Kp * (q_ref - q) - Kd * qdot` | Kp = 120 Nm/rad; Kd = 1.6 Nm*s/rad |
| Two wheels | `Kv * (omega_ref - omega)` | Kv = 0.50 Nm*s/rad |

These are position/velocity feedback gains producing torque. There is no
measured torque-error PD, integral, gravity feedforward, or balance-controller
output in this tracker. Final physical torques are clipped and slew limited,
then divided by the model actuator gains (50 for legs, 10 for wheels).
Limits in actuator order are `[60,60,10,60,60,10]` Nm and
`[300,300,40,300,300,40]` Nm/s. These limits are simulation settings.

## Bench and coverage

The fixture removes the base free joint, retains real link inertias, passive
joints, joint bounds and closed-chain equalities, and disables contacts.
Native control is used to prepare poses only; neither RL nor native balance
acts during the servo tests. Encoder kinematics solve the commanded pose and
passive joints are projected back onto linkage constraints.

Actual initial lengths are 47, 60, 85, 117, 150, 180 and 198 mm. The requested
210 mm cannot be reached at the chosen fixed swing with these joint bounds.
The report marks this explicitly; it does not establish the mechanism's
global maximum or validate the 210 mm endpoint.

Leg tests apply upward wheel-axle loads of zero or half the complete robot's
weight (about 11.8995 N per side). Cartesian wrench moments shift MuJoCo's
COM-applied force to the axle. Each driven motor receives both signs of a
0.04 rad step. The other motors hold position, with wheels at zero speed.

## Results

Leg gain selection uses worst-case gates as well as average performance.
The selected leg gains pass all 112 combinations of length, load, motor
and step direction:

- Worst absolute steady position error: 0.005657 rad (0.324 degrees).
- Worst baseline-relative overshoot: 4.34 percent.
- Dynamic settling: 20-39 ms, requiring sustained tolerance entry.
- Worst final 150 ms position ripple: 0.000002327 rad.
- Peak applied leg torque: 3.015 Nm; no torque saturation.

Wheel Kv = 0.50 passes 56 short step cases and 168 extended cases.
Extended cases last 5 seconds and test zero, +/-2 and +/-10 rad/s targets
on either wheel, with zero or 0.05 Nm opposing disturbance torque.

| Target magnitude | Worst final mean speed error | Worst final 150 ms speed ripple |
| --- | --- | --- |
| 0 rad/s | 0.1374 rad/s | 0.000965 rad/s |
| 2 rad/s | 0.3334 rad/s | 0.01654 rad/s |
| 10 rad/s | 1.0001 rad/s | 0.07650 rad/s |

Worst sample-to-sample speed change in the final 2 seconds is 0.000554
rad/s; full speed variation in that interval is 0.12166 rad/s. Peak wheel
torque is 0.831 Nm and no torque saturation occurs.

Kv = 0.55 passes a short step sweep but exhibits alternating 1 ms speed
chatter during an extended short-leg zero-speed hold. Higher gains also
produce chatter. Kv = 0.50 is the nominal baseline below this boundary.
The screening report preserves the comparisons rather than selecting solely
by average step score.

The model wheel COM is approximately 8.5 mm off the axle, producing periodic
gravity torque as it rotates. High-speed tests therefore measure both slow
speed variation and sample-to-sample chatter. The report specifies separate
extended acceptance gates; the original short-step gates remain unchanged.
Wheel joint damping is 0.05 Nm*s/rad, so a pure P tracker has load-dependent
steady speed error. This baseline does not promise zero static error.

## Reproduction and limits

```bash
.venv-mjbatch/bin/python scripts/tools/tune_rl_joint_pd.py \
  --wheel-kv 0.5 --out logs/rl_training/joint_pd_bench_selected_20261018.json
.venv-mjbatch/bin/python -m pytest -q scripts/tools/test_spin_joint_pd.py
```

The main report includes gain rankings, actual lengths, every test case,
model/source hashes and acceptance gates. Software tests check physical
units, motor order, feedback sign, torque/slew clipping, sustained settling
and wheel-axle load application.

Fixed-base tracking does not prove free-base balance, ground traction,
large joint-target jumps, slope operation or hardware stability. RL targets
must subsequently be limited to feasible linkage configurations, and the
new flat-ground environment must validate the coupled robot with this
tracker as its only actuator output path.
