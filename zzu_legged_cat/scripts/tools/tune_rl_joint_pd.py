"""Measure target-tracking PD gains on a physically fixed-base linkage bench.

The full closed-chain linkage and rotor gains remain intact. The bench removes
base free motion and contacts, and applies known upward loads at wheel centers
to represent support reaction. It never evaluates whole-robot balance.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import mujoco
import numpy as np
from mjbatch import Batch
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from spin_joint_pd import (ACTUATOR_NAMES, LEG_CHANNELS, WHEEL_CHANNELS,
                           JointTargetPD, TORQUE_LIMIT_NM)
from spin_wrench_env import WrenchSpinEnv
from spin_leg_geometry import EncoderLegGeometry
from wheel_legged.provenance import implementation_digest

LENGTHS = (.047, .060, .085, .117, .150, .180, .210)
LEG_KP = (20., 40., 80., 120., 160., 200.)
LEG_KD = (.2, .4, .8, 1.2, 1.6, 2., 3.)
WHEEL_KV = (.1, .2, .4, .45, .48, .5, .52, .55, .6, .7, .8, .9, 1., 1.2, 1.6)


def prepare(lengths, threads):
    cases = [(0, 0, .117-length, 1) for length in lengths]
    terrain = [dict(location='lower_flat', handover='pid', takeover_time=0,
                    fade=.3, dropout=False)]*len(cases)
    env = WrenchSpinEnv(len(cases), threads, cases=cases, terrain_cases=terrain,
                        reference_mode='hold', disturbances=False)
    try:
        env.seed(3000)
        env.reset()
        snapshots = env.qpos.copy()
        actual = env._native_observation()[:, 11:13].copy()
        return env.provenance['model_xml'], snapshots, actual, env.robot_weight
    finally:
        env.close()


def fixed_bench(path, pose, target_length=None, permit_nearest=False):
    source = mujoco.MjModel.from_xml_path(path)
    spec = mujoco.MjSpec.from_file(path)
    for key in list(spec.keys):
        spec.delete(key)
    for body in list(spec.bodies):
        if body.name.startswith(('test_ball_', 'large_ball_')):
            spec.delete(body)
    base = spec.body('base_link')
    for joint in list(base.joints):
        if joint.type == mujoco.mjtJoint.mjJNT_FREE:
            spec.delete(joint)
    base.pos = pose[:3]+[0, 0, .5]
    base.quat = pose[3:7]
    model = spec.compile()
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    model.opt.timestep = .001
    initial = model.qpos0.copy()
    for index in range(model.njnt):
        joint = model.joint(index)
        original = source.joint(joint.name)
        initial[int(joint.qposadr[0])] = pose[int(original.qposadr[0])]
    if target_length is not None:
        servo = JointTargetPD(model)
        geometry = EncoderLegGeometry(model)
        driven = servo.qadr[LEG_CHANNELS]
        seed = initial[driven].copy()
        wheel = geometry.calculate(seed, np.zeros(4))[0]
        target = wheel.copy()
        target[:, 1] = -target_length
        limits = model.jnt_range[[model.joint(ACTUATOR_NAMES[k]).id for k in LEG_CHANNELS]]
        def endpoint_error(q):
            return (geometry.calculate(q, np.zeros(4))[0]-target).ravel()
        solved = least_squares(endpoint_error, seed, bounds=(limits[:, 0], limits[:, 1]),
                               xtol=1e-12, ftol=1e-12, gtol=1e-12)
        if np.max(np.abs(endpoint_error(solved.x))) > 1e-6 and not permit_nearest:
            raise RuntimeError(f'Cannot reach requested bench length {target_length}')
        initial[driven] = solved.x
        # Project passive joints back onto both closed-chain constraints.
        passive = np.array([i for i in range(model.nq) if i not in servo.qadr])
        data = mujoco.MjData(model)
        def closure_error(q):
            data.qpos[:] = initial
            data.qpos[passive] = q
            mujoco.mj_forward(model, data)
            return data.efc_pos[data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY].copy()
        lower = np.where(model.jnt_limited, model.jnt_range[:, 0], -np.inf)[passive]
        upper = np.where(model.jnt_limited, model.jnt_range[:, 1], np.inf)[passive]
        projected = least_squares(closure_error, np.clip(initial[passive], lower+1e-8, upper-1e-8),
                                  bounds=(lower, upper),
                                  xtol=1e-12, ftol=1e-12, gtol=1e-12)
        if np.max(np.abs(closure_error(projected.x))) > 1e-6:
            raise RuntimeError('Cannot close bench linkage at requested length')
        initial[passive] = projected.x
        joint_positions = initial[model.jnt_qposadr]
        limited = model.jnt_limited.astype(bool)
        if (np.any(joint_positions[limited] < model.jnt_range[limited, 0]-.001)
                or np.any(joint_positions[limited] > model.jnt_range[limited, 1]+.001)):
            raise RuntimeError(f'Requested bench length {target_length} exceeds joint limits')
    return model, initial


def response_metrics(time_axis, position, reference, baseline, amplitude, start):
    after = time_axis >= start
    tail = time_axis >= time_axis[-1]-.15
    error = reference-position
    signed_step = (position-baseline)/amplitude
    dynamic_error = np.abs(position-(baseline+amplitude))
    tolerance = max(abs(amplitude)*.1, .001)
    outside = np.flatnonzero(after & (dynamic_error > tolerance))
    last_outside = time_axis[outside[-1]] if len(outside) else start
    settle = last_outside-start+.001 if time_axis[-1]-last_outside >= .1 else None
    overshoot = max(0., float(np.max(signed_step[after])-1))
    steady = float(np.mean(np.abs(error[tail])))
    ripple = float(np.ptp(position[tail]))
    return dict(steady_error=steady, overshoot_fraction=overshoot,
                settle_seconds=settle, tail_peak_to_peak=ripple)


def wheel_center_load(model, forces, origins, centers_of_mass, magnitude):
    """MuJoCo Cartesian wrenches act at COM; shift support to the axle."""
    force = np.array([0., 0., magnitude])
    for name in ('Left_Wheel_link', 'Right_Wheel_link'):
        body = model.body(name).id
        forces[:, body, :3] = force
        forces[:, body, 3:] = np.cross(origins[:, body]-centers_of_mass[:, body], force)


def leg_sweep(model, initial, length, load_fraction, weight, threads, candidates):
    trials = [(kp, kd, motor, direction) for kp, kd in candidates
              for motor in range(4) for direction in (-1, 1)]
    n = len(trials)
    batch = Batch(model, n, num_threads=threads, forward=False)
    try:
        qpos, qvel, ctrl = (batch.bind(key) for key in ('qpos', 'qvel', 'ctrl'))
        forces = batch.bind('xfrc_applied')
        origins, centers_of_mass = (batch.bind(key) for key in ('xpos', 'xipos'))
        qpos[:] = initial
        qvel[:] = 0
        batch.forward()
        servo = JointTargetPD(model)
        gains = SimpleNamespace(leg_kp=np.array([t[0] for t in trials]),
                                leg_kd=np.array([t[1] for t in trials]), wheel_kv=np.full(n, .2))
        nominal = qpos[:, servo.qadr[LEG_CHANNELS]].copy()
        target = nominal.copy()
        motor = np.array([t[2] for t in trials])
        step = np.array([t[3]*.04 for t in trials])
        limits = model.jnt_range[[model.joint(ACTUATOR_NAMES[k]).id for k in LEG_CHANNELS]]
        wanted = target[np.arange(n), motor]+step
        wanted = np.clip(wanted, limits[motor, 0]+.002, limits[motor, 1]-.002)
        step = wanted-target[np.arange(n), motor]
        previous = np.zeros((n, 6))
        q_trace, torque_peak = [], np.zeros(n)
        saturated = np.zeros(n)
        baseline = np.zeros(n)
        total, onset = 1000, 200
        for tick in range(total):
            wheel_center_load(model, forces, origins, centers_of_mass, load_fraction*weight/2)
            if tick == onset:
                baseline[:] = qpos[np.arange(n), servo.qadr[LEG_CHANNELS[motor]]]
                target[np.arange(n), motor] += step
            ctrl[:], previous, requested = servo.control(
                qpos[:, servo.qadr], qvel[:, servo.vadr], target, np.zeros((n, 2)),
                gains, previous, model.opt.timestep)
            torque_peak = np.maximum(torque_peak, np.max(np.abs(previous[:, LEG_CHANNELS]), axis=1))
            saturated += (np.abs(requested) >= TORQUE_LIMIT_NM).any(axis=1)
            batch.step()
            q_trace.append(qpos[np.arange(n), servo.qadr[LEG_CHANNELS[motor]]].copy())
        trace = np.array(q_trace)
        time_axis = (np.arange(total)+1)*model.opt.timestep
        rows = []
        for i, (kp, kd, joint, direction) in enumerate(trials):
            if abs(step[i]) < .005:
                continue
            metrics = response_metrics(time_axis, trace[:, i], target[i, joint],
                                       baseline[i], step[i], onset*model.opt.timestep)
            rows.append(dict(kind='leg', length_m=length, load_fraction=load_fraction,
                             kp=kp, kd=kd, joint=ACTUATOR_NAMES[LEG_CHANNELS[joint]],
                             direction=direction, step_rad=float(step[i]),
                             torque_peak_nm=float(torque_peak[i]), saturation_fraction=float(saturated[i]/total),
                             **metrics))
        return rows
    finally:
        del batch


def wheel_sweep(model, initial, length, gains, threads, candidates=WHEEL_KV,
                speeds=(2.,), duration_s=1.):
    trials = [(kv, side, sign, load, speed) for kv in candidates for side in (0, 1)
              for sign in (-1, 1) for load in (0., .05) for speed in speeds]
    n = len(trials)
    batch = Batch(model, n, num_threads=threads, forward=False)
    try:
        qpos, qvel, ctrl = (batch.bind(key) for key in ('qpos', 'qvel', 'ctrl'))
        forces = batch.bind('qfrc_applied')
        qpos[:] = initial
        qvel[:] = 0
        batch.forward()
        servo = JointTargetPD(model)
        gain = SimpleNamespace(leg_kp=np.full(n, gains['kp']), leg_kd=np.full(n, gains['kd']),
                               wheel_kv=np.array([t[0] for t in trials]))
        leg_target = qpos[:, servo.qadr[LEG_CHANNELS]].copy()
        target = np.zeros((n, 2))
        previous = np.zeros((n, 6))
        speed_trace, peak, saturation = [], np.zeros(n), np.zeros(n)
        total = int(round(duration_s/model.opt.timestep))
        for tick in range(total):
            if tick == 200:
                for i, (_, side, sign, load, speed) in enumerate(trials):
                    target[i, side] = sign*speed
                    forces[i, servo.vadr[WHEEL_CHANNELS[side]]] = -sign*load
            ctrl[:], previous, request = servo.control(qpos[:, servo.qadr], qvel[:, servo.vadr],
                leg_target, target, gain, previous, model.opt.timestep)
            peak = np.maximum(peak, np.max(np.abs(previous[:, WHEEL_CHANNELS]), axis=1))
            saturation += (np.abs(request[:, WHEEL_CHANNELS]) >= 10).any(axis=1)
            batch.step()
            speed_trace.append(np.array([qvel[i, servo.vadr[WHEEL_CHANNELS[t[1]]]] for i, t in enumerate(trials)]))
        trace = np.array(speed_trace)
        time_axis = (np.arange(total)+1)*model.opt.timestep
        tail = trace[-min(total-200, 2000):]
        return [dict(kind='wheel', length_m=length, kv=kv, side=side, direction=sign,
                     speed_target_rad_s=sign*speed,
                     disturbance_torque_nm=load, torque_peak_nm=float(peak[i]),
                     saturation_fraction=float(saturation[i]/total),
                     final_2s_peak_to_peak_rad_s=float(np.ptp(tail[:, i])),
                     final_2s_max_sample_change_rad_s=float(np.max(np.abs(np.diff(tail[:, i])))),
                     **response_metrics(time_axis, trace[:, i], sign*speed, 0,
                                        sign*max(speed, .1), .2))
                for i, (kv, side, sign, load, speed) in enumerate(trials)]
    finally:
        del batch


def select_legs(rows):
    ranking = []
    for kp, kd in sorted({(r['kp'], r['kd']) for r in rows}):
        group = [r for r in rows if r['kp'] == kp and r['kd'] == kd]
        score = np.mean([r['steady_error']*100 + r['overshoot_fraction']*2
                         + (r['settle_seconds'] if r['settle_seconds'] is not None else 2)
                         + r['tail_peak_to_peak']*100 + r['saturation_fraction']*10 for r in group])
        ranking.append(dict(kp=kp, kd=kd, score=float(score), cases=len(group),
            worst_steady_error_rad=max(r['steady_error'] for r in group),
            worst_tail_peak_to_peak_rad=max(r['tail_peak_to_peak'] for r in group),
            worst_overshoot_fraction=max(r['overshoot_fraction'] for r in group),
            unsettled_cases=sum(r['settle_seconds'] is None for r in group),
            qualified=bool(max(r['steady_error'] for r in group) <= .01
                and max(r['overshoot_fraction'] for r in group) <= .10
                and max(r['tail_peak_to_peak'] for r in group) <= .0005
                and all(r['settle_seconds'] is not None for r in group)
                and max(r['saturation_fraction'] for r in group) == 0)))
    return sorted(ranking, key=lambda r: (not r['qualified'], r['score']))


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--threads', type=int, default=8)
    parser.add_argument('--quick', action='store_true')
    parser.add_argument('--wheel-kv', type=float, choices=WHEEL_KV, default=.5,
                        help='Nominal wheel gain, validated beyond the short sweep')
    parser.add_argument('--out', type=Path, default=Path('logs/rl_training/joint_pd_bench_20261018.json'))
    args = parser.parse_args()
    lengths = (.047, .117, .210) if args.quick else LENGTHS
    candidates = [(kp, kd) for kp in ((20., 40.) if args.quick else LEG_KP)
                  for kd in ((.2, .8) if args.quick else LEG_KD)]
    start = time.monotonic()
    path, poses, actual, weight = prepare(lengths, args.threads)
    rows, benches, bench_lengths = [], [], []
    for length, pose, measured in zip(lengths, poses, actual):
        fallback_reason = None
        try:
            model, initial = fixed_bench(path, pose, length)
        except RuntimeError as error:
            fallback_reason = str(error)
            # Explicitly report a locally unreachable request; never label a
            # different measured length as a successfully reached endpoint.
            for fallback in np.arange(length-.001, length-.025, -.001):
                try:
                    model, initial = fixed_bench(path, pose, float(fallback))
                    break
                except RuntimeError:
                    continue
            else:
                raise RuntimeError(f'No feasible bench pose near {length}: {error}') from error
        servo = JointTargetPD(model)
        actual_length = -EncoderLegGeometry(model).calculate(
            initial[servo.qadr[LEG_CHANNELS]], np.zeros(4))[0][:, 1]
        bench_lengths.append(actual_length.tolist())
        measured_length = float(np.mean(actual_length))
        benches.append((measured_length, model, initial))
        for load in (0., 1.):
            result = leg_sweep(model, initial, measured_length, load, weight, args.threads, candidates)
            rows.extend(result)
            print(json.dumps(dict(phase='leg', requested_length_m=length,
                measured_initial_lengths_m=actual_length.tolist(),
                target_reached=bool(np.max(np.abs(actual_length-length)) < 1e-6),
                fallback_reason=fallback_reason,
                load_fraction=load, cases=len(result))), flush=True)
    ranking = select_legs(rows)
    best = ranking[0]
    if not best['qualified'] and not args.quick:
        raise RuntimeError('No leg PD candidate met the worst-case acceptance gates')
    print('LEG_RANKING', json.dumps(ranking[:5]), flush=True)
    for length, model, initial in benches:
        rows.extend(wheel_sweep(model, initial, length, best, args.threads))
        print('WHEEL_DONE', length, flush=True)
    wheel_rank = []
    for kv in WHEEL_KV:
        group = [r for r in rows if r['kind'] == 'wheel' and r['kv'] == kv]
        score = np.mean([r['steady_error']+r['overshoot_fraction']*2+r['tail_peak_to_peak']
                         + (r['settle_seconds'] if r['settle_seconds'] is not None else 2) for r in group])
        wheel_rank.append(dict(kv=kv, score=float(score),
            worst_steady_error_rad_s=max(r['steady_error'] for r in group),
            worst_ripple_rad_s=max(r['tail_peak_to_peak'] for r in group),
            qualified=bool(max(r['tail_peak_to_peak'] for r in group) <= .05
                and max(r['steady_error'] for r in group) <= .35
                and max(r['saturation_fraction'] for r in group) == 0)))
    wheel_rank.sort(key=lambda r: (not r['qualified'], r['score']))
    print('WHEEL_RANKING', json.dumps(wheel_rank), flush=True)
    selected_wheel = next(row for row in wheel_rank if row['kv'] == args.wheel_kv)
    extended = []
    for length, model, initial in benches:
        extended.extend(wheel_sweep(model, initial, length, best, args.threads,
            candidates=(args.wheel_kv,), speeds=(0., 2., 10.), duration_s=5.))
    # High-speed tests also resolve the slow gravity disturbance of the model's
    # eccentric wheel COM. Sample-to-sample change separately detects chatter.
    extended_qualified = all(
        r['steady_error'] <= .35+.1*max(abs(r['speed_target_rad_s'])-2, 0)
        and r['tail_peak_to_peak'] <= .05+.005*max(abs(r['speed_target_rad_s'])-2, 0)
        and r['final_2s_peak_to_peak_rad_s'] <= .15
        and r['final_2s_max_sample_change_rad_s'] <= .005
        and r['saturation_fraction'] == 0 for r in extended)
    print('WHEEL_EXTENDED', json.dumps(dict(kv=args.wheel_kv,
        cases=len(extended), qualified=extended_qualified)), flush=True)
    report = dict(task='fixed_base_joint_target_pd', timestep_s=.001,
        qualified=best['qualified'] and selected_wheel['qualified'] and extended_qualified,
        gains=dict(leg_kp=best['kp'], leg_kd=best['kd'], wheel_kv=args.wheel_kv),
        wheel_selection='Explicit nominal gain; short-sweep ranking alone does not establish extended stability',
        gain_units=dict(leg_kp='Nm/rad', leg_kd='Nm*s/rad', wheel_kv='Nm*s/rad'),
        actor_or_balance_controller_used=False, native_reset_preparation_only=True,
        body_weight_n=weight, model_xml=path,
        model_sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),
        source_sha256=implementation_digest(__file__,
            Path(__file__).parents[1].joinpath('spin_joint_pd.py')),
        requested_lengths_m=list(lengths), measured_initial_lengths_m=bench_lengths,
        native_preparation_lengths_m=actual.tolist(),
        acceptance=dict(leg_steady_error_rad=.01, leg_overshoot_fraction=.10,
            leg_tail_peak_to_peak_rad=.0005, leg_all_settled=True,
            wheel_steady_error_rad_s=.35, wheel_tail_peak_to_peak_rad_s=.05,
            wheel_extended_duration_s=5., wheel_extended_speeds_rad_s=[0., 2., 10.],
            wheel_extended_error_bound='.35 + .1*max(abs(speed)-2, 0)',
            wheel_extended_tail_ripple_bound='.05 + .005*max(abs(speed)-2, 0)',
            wheel_extended_2s_peak_to_peak_rad_s=.15,
            wheel_extended_max_sample_change_rad_s=.005,
            allowed_saturation_fraction=0),
        limits='fixed-base simulated servo validation only; not whole-robot stability or hardware tuning',
        leg_ranking=ranking, wheel_ranking=wheel_rank, results=rows,
        wheel_extended_qualified=extended_qualified, wheel_extended_results=extended,
        elapsed_s=time.monotonic()-start)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2)+'\n')
    if not report['qualified'] and not args.quick:
        raise RuntimeError(f'No qualified PD set; diagnostic report saved to {args.out}')
    print('RECOMMENDATION', json.dumps({k: report[k] for k in ('gains', 'elapsed_s')}), flush=True)


if __name__ == '__main__':
    main()
