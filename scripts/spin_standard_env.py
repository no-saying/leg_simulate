"""V5 reward-side mapping to the uphill leg at an upslope wheel-line heading."""

import hashlib
from pathlib import Path

import mujoco
import numpy as np
from spin_rl_env import LEG_MID, attitude
from spin_travel_env import LENGTH_MAX, LENGTH_MIN, TravelSpinEnv

TASK_VERSION = "slope_spin_standard_v5"
NEUTRAL_SWING = -0.01596


def ramp_surface_height(points, ramp_length, ramp_height, floor_height):
    """Query the finite ramp, its upper flat and the surrounding lower floor."""
    x, y = np.asarray(points)[..., 0], np.asarray(points)[..., 1]
    length, height = np.asarray(ramp_length)[:, None], np.asarray(ramp_height)[:, None]
    inside = (x >= 0) & (x <= length+4) & (np.abs(y) <= 2)
    return floor_height+np.where(inside, np.clip(x/length, 0, 1)*height, 0)


def standard_pose_lengths(lengths, swings, roll, pitch, lateral,
                          current_heights, aligned_heights):
    """Keep reconstructed base height; level the body and align wheels upslope.

    L is body-down displacement, not a radial link length. The current vertical
    axle offset is -sin(pitch)*swing + cos(pitch)*sin(roll)*y
    - cos(pitch)*cos(roll)*L. Equal wheel radii cancel between poses.
    Outputs are uphill (short) and downhill (long) equivalent lengths. Values
    outside physical limits are retained, rather than hiding missing travel.
    """
    cp, sp = np.cos(pitch)[:, None], np.sin(pitch)[:, None]
    cr, sr = np.cos(roll)[:, None], np.sin(roll)[:, None]
    offset_z = -sp*swings+cp*sr*np.asarray(lateral)-cp*cr*lengths
    base_above_axle_radius = np.mean(current_heights-offset_z, axis=1)
    return np.column_stack((base_above_axle_radius-aligned_heights.max(axis=1),
                            base_above_axle_radius-aligned_heights.min(axis=1)))


def standard_reward_weights(requested_length):
    """Continuous task weights, independent of which physical leg is uphill."""
    def smoothstep(x):
        x = np.clip(x, 0, 1)
        return x*x*(3-2*x)

    short = smoothstep((0.090-np.asarray(requested_length))/(0.090-LENGTH_MIN))
    long = smoothstep((np.asarray(requested_length)-0.165)/(LENGTH_MAX-0.165))
    return 1+short, 1+0.5*long


class StandardSpinEnv(TravelSpinEnv):
    """Same sensor-only 416 -> 5 policy contract and native control as V4."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, data)
        base = self.model.body("base_link").id
        rotation = data.xmat[base].reshape(3, 3)
        # CAD +z is physical -y after the existing +90 degree IMU mount bake.
        self.axle_lateral = np.array([
            -(rotation.T@(data.xpos[self.model.body(name).id]-data.xpos[base]))[2]
            for name in ("Left_Wheel_link", "Right_Wheel_link")])
        self.axle_track = abs(np.diff(self.axle_lateral)[0])
        self.current_support = np.zeros((self.num_envs, 2))
        self.aligned_support = np.zeros((self.num_envs, 2))
        self.standard_requested = np.zeros(self.num_envs)
        self.standard_requested_initial = np.zeros(self.num_envs)
        self.standard_actual = np.zeros((self.num_envs, 2))
        self.standard_integral = np.zeros((self.num_envs, 5))
        self.reward_totals["standard_reserve"] = np.zeros(self.num_envs)
        self.provenance.update(
            task=TASK_VERSION, reward_version="standard_v5",
            length_standard="uphill leg with wheel line aligned upslope, level body, same base height",
            standard_geometry_hz=50, standard_inputs_to_actor=False,
            axle_lateral_m=self.axle_lateral.tolist(),
            yaw_weight_short_middle_long=[2, 1, 1],
            drift_weight_short_middle_long=[1, 1, 1.5],
            weight_basis="requested common length mapped at current position and heading",
            reward_source_sha256=hashlib.sha256(b"".join(
                Path(__file__).with_name(name).read_bytes() for name in (
                    "spin_standard_env.py", "spin_travel_env.py", "spin_mixed_env.py",
                    "spin_rl_env.py"))).hexdigest(),
            equivalent_lengths_clipped=False)

    def _surface(self, points):
        return ramp_surface_height(points, self.ramp_length, self.ramp_height,
                                   self.model.geom_pos[self.floor, 2])

    def _update_standard_geometry(self, ids):
        native = self._native_observation()
        q, gyro, _ = self._imu()
        roll, pitch, yaw, _, _ = attitude(q, gyro)
        w, x, y, z = q.T
        rotation = np.stack((
            1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
            2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
            2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)), axis=1).reshape(-1, 3, 3)
        local = np.stack((native[:, 15:17],
                          np.broadcast_to(self.axle_lateral, (self.num_envs, 2)),
                          -native[:, 11:13]), axis=2)
        offset = np.einsum("nij,nkj->nki", rotation, local)
        axle_xy = self.qpos[:, None, :2]+offset[:, :, :2]
        center = axle_xy.mean(axis=1)
        # Align the complete planar wheel line, including unequal swing offsets.
        half_span = 0.5*np.hypot(self.axle_track, native[:, 15]-native[:, 16])
        aligned = np.repeat(center[:, None, :], 2, axis=1)
        aligned[:, :, 0] += half_span[:, None]*np.array([-1, 1])
        self.current_support[ids] = self._surface(axle_xy)[ids]
        self.aligned_support[ids] = self._surface(aligned)[ids]

        # Map the original request independently of the policy's common/swing
        # actions. Level reference footprints retain the current yaw and position.
        c, s = np.cos(yaw), np.sin(yaw)
        nominal_xy = self.qpos[:, None, :2]+np.stack((
            c[:, None]*NEUTRAL_SWING-s[:, None]*self.axle_lateral,
            s[:, None]*NEUTRAL_SWING+c[:, None]*self.axle_lateral), axis=2)
        nominal_center = nominal_xy.mean(axis=1)
        nominal_aligned = np.repeat(nominal_center[:, None, :], 2, axis=1)
        nominal_aligned[:, :, 0] += self.axle_track/2*np.array([-1, 1])
        requested = (LEG_MID-self.ext+self._surface(nominal_xy).mean(axis=1)
                     -self._surface(nominal_aligned).max(axis=1))
        self.standard_requested[ids] = requested[ids]
        self.standard_actual[ids] = standard_pose_lengths(
            native[:, 11:13], native[:, 15:17], roll, pitch, self.axle_lateral,
            self.current_support, self.aligned_support)[ids]

    def _reset_context(self, ids):
        super()._reset_context(ids)
        self._update_standard_geometry(ids)
        self.standard_requested_initial[ids] = self.standard_requested[ids]
        self.standard_integral[ids] = 0

    def _before_policy_step(self):
        super()._before_policy_step()
        self._update_standard_geometry(np.arange(self.num_envs))

    def _reward(self, *args):
        _, terms = super()._reward(*args)
        roll, pitch, _, yaw_rate, yaw_target = args[:5]
        native = self._native_observation()
        self.standard_actual[:] = standard_pose_lengths(
            native[:, 11:13], native[:, 15:17], roll, pitch, self.axle_lateral,
            self.current_support, self.aligned_support)
        yaw_weight, drift_weight = standard_reward_weights(self.standard_requested)
        error = np.abs(yaw_rate-yaw_target)/0.6
        huber = np.where(error <= 1, 0.5*error**2, np.minimum(error, 6)-0.5)
        terms["yaw"] = 2-0.8*yaw_weight*huber
        terms["spin_progress"] *= yaw_weight
        terms["drift"] *= drift_weight
        terms["drift_peak"] *= drift_weight
        error_length = self.standard_actual[:, 0]-self.standard_requested
        terms["height"] = -0.03*np.minimum((error_length/0.040)**2, 4)
        # Both ends matter: uphill shortening AND downhill lengthening reserve.
        margin = np.column_stack((self.standard_actual[:, 0]-LENGTH_MIN,
                                  LENGTH_MAX-self.standard_actual[:, 1]))
        need = 0.006+0.006*np.clip(np.abs(yaw_target)/0.6, 0, 1)
        deficit = np.maximum(need[:, None]-margin, 0)
        terms["standard_reserve"] = -0.8*np.mean(np.minimum((deficit/0.020)**2, 4), axis=1)
        infeasible = (margin < 0).any(axis=1)
        self.standard_integral += np.column_stack((self.standard_requested,
                                                   self.standard_actual[:, 0],
                                                   yaw_weight, drift_weight, infeasible))*self.dt
        return 1.0+sum(terms.values()), terms

    def _extra_metrics(self, i):
        duration = max(self.steps[i]*self.policy_dt, self.dt)
        means = self.standard_integral[i]/duration
        return {**super()._extra_metrics(i),
                "standard_requested_initial_m": float(self.standard_requested_initial[i]),
                "standard_requested_mean_m": float(means[0]),
                "standard_actual_mean_m": float(means[1]),
                "standard_high_final_m": float(self.standard_actual[i, 0]),
                "standard_low_final_m": float(self.standard_actual[i, 1]),
                "standard_yaw_weight_mean": float(means[2]),
                "standard_drift_weight_mean": float(means[3]),
                "standard_infeasible_fraction": float(means[4])}
