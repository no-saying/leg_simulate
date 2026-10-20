"""V4: directional physical leg travel, common-height authority and revised reward."""

import hashlib
from pathlib import Path

import gymnasium as gym
import numpy as np
from spin_contacts import SpinContacts
from spin_mixed_env import MixedTerrainSpinEnv
from spin_rl_env import (
    ACTION_SCALE,
    EXT_LIMITS,
    HISTORY,
    LEG_MID,
    OBS_SCALE,
    spin_reward,
)

TASK_VERSION = "slope_spin_travel_v4"
FRAME_SIZE = 52
LENGTH_MIN, LENGTH_MAX = LEG_MID-EXT_LIMITS[1], LEG_MID-EXT_LIMITS[0]
LENGTH_SPAN = LENGTH_MAX-LENGTH_MIN
COMMON_OFFSET_MAX = 0.040
COMMON_RATE_MAX = 0.025
COMMON_ACCEL_MAX = 0.10
OBSERVATION_SCALE = np.r_[OBS_SCALE, [1, 1], [LENGTH_SPAN]*8, COMMON_RATE_MAX].tolist()
PHYSICAL_ACTION_SCALE = ACTION_SCALE.copy()
PHYSICAL_ACTION_SCALE[1] = COMMON_OFFSET_MAX


def length_margins(lengths):
    """Meters: L shorten/lengthen, R shorten/lengthen, common +/- and diff +/-.

    Positive differential lengthens the left leg and shortens the right.
    These are physical length directions, opposite to native extension commands.
    Negative entries retain evidence of a measured excursion past a limit.
    """
    lengths = np.asarray(lengths)
    shorten, lengthen = lengths-LENGTH_MIN, LENGTH_MAX-lengths
    return np.column_stack((shorten[:, 0], lengthen[:, 0], shorten[:, 1], lengthen[:, 1],
                            lengthen.min(axis=1), shorten.min(axis=1),
                            np.minimum(lengthen[:, 0], shorten[:, 1]),
                            np.minimum(shorten[:, 0], lengthen[:, 1])))


def differential_action(action, lengths):
    margins = length_margins(lengths)
    available = np.where(action >= 0, margins[:, 6], margins[:, 7])
    return action*np.clip(available/ACTION_SCALE[0], 0, 1)


def travel_reward(roll, pitch, roll_rate, yaw_rate, yaw_target, speed, drift,
                  actions, action_delta, length_error, lengths, contacts, common_rate,
                  robot_weight, previous_peak=None, dt=0.001):
    _, terms = spin_reward(roll, pitch, roll_rate, yaw_rate, yaw_target, speed, drift,
                           actions, action_delta, length_error)
    margins = length_margins(lengths)
    available = np.minimum(margins[:, [0, 2]], margins[:, [1, 3]])
    load = np.clip(contacts[:, :, 0]/(0.1*robot_weight), 0, 1)
    both = (load > 0.2).all(axis=1)
    height_difference = np.where(both, np.abs(contacts[:, 0, 2]-contacts[:, 1, 2]), 0)
    signed_height = contacts[:, 0, 2]-contacts[:, 1, 2]
    # Higher left support requires a shorter left leg for an upright chassis.
    geometry_error = lengths[:, 0]-lengths[:, 1]+signed_height
    terms["support_geometry"] = -0.2*both*np.minimum((geometry_error/0.020)**2, 4)
    turning = np.clip(np.abs(yaw_target)/0.6, 0, 1)
    need = np.minimum(0.040, 0.006+0.006*turning+0.5*height_difference)
    deficit = np.maximum(need[:, None]-available, 0)
    terms["leg_reserve"] = -0.8*np.mean(np.minimum((deficit/0.020)**2, 4), axis=1)
    # Keep the requested height as a weak preference while allowing useful travel.
    terms["height"] = -0.03*np.minimum((length_error/0.040)**2, 4)
    position_error = drift/0.3
    position_cost = np.where(position_error <= 1, position_error**2,
                              2*np.minimum(position_error, 11)-1)
    terms["drift"] = -(np.minimum((speed/0.2)**2, 9)+position_cost)
    # Charge new excursions once, so returning does not refund maximum drift.
    terms["drift_peak"] = (np.zeros_like(drift) if previous_peak is None else
                            -4*np.maximum(drift-previous_peak, 0)/(0.3*dt))
    extreme = np.clip((0.025-available.min(axis=1))/0.025, 0, 1)
    error = np.abs(yaw_rate-yaw_target)/0.6
    huber = np.where(error <= 1, 0.5*error**2, np.minimum(error, 6)-0.5)
    terms["yaw"] = 2-0.8*(1+0.5*extreme)*huber
    signed_ratio = yaw_rate*np.sign(yaw_target)/np.maximum(np.abs(yaw_target), 0.6)
    terms["spin_progress"] = 0.6*turning*np.clip(signed_ratio, 0, 1.0)
    # Relative velocity at the actual terrain contact already includes wheel spin
    # and body yaw. Legitimate rolling in place is not classified by encoder speed.
    terms["contact_slip"] = -0.6*np.mean(
        load*np.minimum((contacts[:, :, 1]/0.25)**2, 4), axis=1)
    terms["support"] = -0.3*np.mean(1-load, axis=1)
    terms["smooth"] = -0.15*np.mean(action_delta**2, axis=1)
    terms["height_rate"] = -0.05*np.minimum((common_rate/COMMON_RATE_MAX)**2, 1)
    return 1.0+sum(terms.values()), terms


class TravelSpinEnv(MixedTerrainSpinEnv):
    failure_cost_per_second = 85

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.common_offset = np.zeros(self.num_envs)
        self.common_rate = np.zeros(self.num_envs)
        self.common_peak = np.zeros(self.num_envs)
        self.minimum_margin = np.full(self.num_envs, np.inf)
        self.slip_integral = np.zeros(self.num_envs)
        self.reserve_deficit_integral = np.zeros(self.num_envs)
        self.contact_values = np.zeros((self.num_envs, 2, 4))
        self.contacts = SpinContacts(self.model, self.num_envs, self.batch.num_threads)
        self.robot_weight = float(self.model.body_subtreemass[self.model.body("base_link").id]*9.81)
        self.history = np.zeros((self.num_envs, HISTORY, FRAME_SIZE), np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (FRAME_SIZE*HISTORY,), np.float32)
        for name in ("leg_reserve", "spin_progress", "contact_slip", "support", "height_rate",
                     "support_geometry", "drift_peak"):
            self.reward_totals[name] = np.zeros(self.num_envs)
        self.provenance.update(
            task=TASK_VERSION, frame_size=FRAME_SIZE, reward_version="travel_v4",
            physical_length_limits_m=[LENGTH_MIN, LENGTH_MAX],
            margin_order=["left_shorten", "left_lengthen", "right_shorten", "right_lengthen",
                          "common_lengthen", "common_shorten", "diff_positive", "diff_negative"],
            common_offset_max_m=COMMON_OFFSET_MAX, common_rate_max_mps=COMMON_RATE_MAX,
            common_accel_max_mps2=COMMON_ACCEL_MAX, contact_hz=50,
            actor_contact_inputs=False, remaining_failure_cost_per_s=self.failure_cost_per_second,
            contact_source_sha256=hashlib.sha256(
                Path(__file__).with_name("spin_contact_bridge.c").read_bytes()).hexdigest())

    def _reset_context(self, ids):
        super()._reset_context(ids)
        for state in (self.common_offset, self.common_rate, self.common_peak,
                      self.slip_integral, self.reserve_deficit_integral):
            state[ids] = 0
        self.minimum_margin[ids] = np.inf
        self.contact_values[ids] = self.contacts.query(self.state)[ids]

    def _before_policy_step(self):
        self.contact_values[:] = self.contacts.query(self.state)

    def _frame_obs(self):
        frame = super()._frame_obs()
        native = self._native_observation()
        # Preserve original task height, expose applied common-height state.
        frame[:, 24] = (LEG_MID-self.ext)/OBS_SCALE[24]
        frame[:, 37] = self.common_offset/COMMON_OFFSET_MAX
        margins = length_margins(native[:, 11:13])/LENGTH_SPAN
        return np.column_stack((frame, np.clip(margins, -1, 1),
                                self.common_rate/COMMON_RATE_MAX)).astype(np.float32)

    def _apply_residual(self, actions, mask):
        nominal = LEG_MID-self.ext
        capacity = np.where(actions[:, 1] >= 0, LENGTH_MAX-nominal, nominal-LENGTH_MIN)
        target = actions[:, 1]*np.minimum(COMMON_OFFSET_MAX, np.maximum(capacity, 0))
        native = self._native_observation()
        obstacle = (native[:, 22:24] > 0.0001).any(axis=1)
        target = np.where(obstacle, np.minimum(target, 0), target)
        desired_rate = np.clip((target-self.common_offset)/0.2, -COMMON_RATE_MAX, COMMON_RATE_MAX)
        self.common_rate += np.clip(desired_rate-self.common_rate,
                                    -COMMON_ACCEL_MAX*self.dt, COMMON_ACCEL_MAX*self.dt)
        next_offset = self.common_offset+self.common_rate*self.dt
        crossed = (target-self.common_offset)*(target-next_offset) < 0
        self.common_offset[:] = np.where(crossed, target, next_offset)
        self.common_offset[:] = np.clip(self.common_offset,
                                        np.maximum(-COMMON_OFFSET_MAX, LENGTH_MIN-nominal),
                                        np.minimum(COMMON_OFFSET_MAX, LENGTH_MAX-nominal))
        self.common_rate[crossed] = 0
        self.common_peak[:] = np.maximum(self.common_peak, np.abs(self.common_offset))
        self.inputs[:, 0] = np.clip(self.ext-self.common_offset, *EXT_LIMITS)
        native_actions = actions.copy()
        native_actions[:, 1] = 0  # Common reference carries this action; do not add it twice.
        native_actions[:, 0] = differential_action(actions[:, 0], native[:, 11:13])
        super()._apply_residual(native_actions, mask)

    def _reward(self, *args):
        lengths = self._native_observation()[:, 11:13]
        margins = length_margins(lengths)
        self.minimum_margin[:] = np.minimum(self.minimum_margin, margins[:, :4].min(axis=1))
        self.slip_integral += np.mean(
            self.contact_values[:, :, 1]*(self.contact_values[:, :, 0] > 1), axis=1)*self.dt
        reward, terms = travel_reward(*args, lengths, self.contact_values,
                                      self.common_rate, self.robot_weight, self.drift_peak, self.dt)
        self.reserve_deficit_integral += -terms["leg_reserve"]*self.dt
        return reward, terms

    def _extra_metrics(self, i):
        return {**super()._extra_metrics(i), "common_offset_peak_m": float(self.common_peak[i]),
                "final_common_offset_m": float(self.common_offset[i]),
                "final_leg_lengths_m": self._native_observation()[i, 11:13].tolist(),
                "minimum_leg_margin_m": float(self.minimum_margin[i]),
                "mean_loaded_contact_slip_mps": float(self.slip_integral[i]/max(self.steps[i]*self.policy_dt, 1e-9)),
                "leg_reserve_cost": float(self.reserve_deficit_integral[i])}

    def close(self):
        if getattr(self, "contacts", None):
            self.contacts.close()
        super().close()
