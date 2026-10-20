"""Finite ramp/flat junctions with randomized residual-policy handovers."""

import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np
from jump_rl_env import XML, quat_mul
from spin_rl_env import HISTORY, SpinRLBatchEnv

TASK_VERSION = "slope_spin_mixed_v3"
FRAME_SIZE = 43
LOCATIONS = ("lower_flat", "toe", "ramp", "crest", "upper_flat")
HANDOVERS = ("pid", "lqr", "pid_to_lqr", "lqr_to_pid")


def validation_cases(slopes=(3, 8), extensions=(-0.093, 0, 0.070),
                     headings=None, boundary_offsets=(0,), handovers=HANDOVERS):
    physics, terrain = [], []
    for slope in slopes:
        for location in LOCATIONS:
            for mode in handovers:
                for ext in extensions:
                    angles = headings if headings is not None else (
                        (np.pi/2,) if location in ("toe", "crest") else (0.7,))
                    offsets = boundary_offsets if location in ("toe", "crest") else (0,)
                    for heading in angles:
                        for offset in offsets:
                            for direction in (-1, 1):
                                physics.append((slope, heading, ext, direction))
                                terrain.append({"location": location, "handover": mode,
                                                "takeover_time": 1.2, "fade": 0.3,
                                                "switch_lead": 0.1, "offset": offset,
                                                "dropout": False})
    return physics, terrain


def build_mixed_scene():
    source = Path(XML).resolve()
    root = ET.fromstring(source.read_bytes())
    for asset in root.findall("./asset/*[@file]"):
        asset.set("file", str((source.parent / asset.get("file")).resolve()))
    world = root.find("worldbody")
    for name, size, rgba in (
        ("spin_ramp", "1 2 0.08", "0.25 0.55 0.38 1"),
        ("spin_upper_flat", "2 2 0.08", "0.40 0.48 0.60 1"),
    ):
        # Fixed-size kinematic supports avoid stale bounds from resizing geoms.
        body = ET.SubElement(world, "body", name=name, mocap="true", pos="0 0 -10")
        ET.SubElement(body, "geom", name=name, type="box", size=size,
                      material="", rgba=rgba, friction="1 0.005 0.0001")
    content = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    digest = hashlib.sha256(content).hexdigest()[:16]
    path = source.parents[1].parent / "logs/rl_training" / f"spin_mixed_{digest}.xml"
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        path.write_bytes(content)
    return path


class MixedTerrainSpinEnv(SpinRLBatchEnv):
    def __init__(self, *args, terrain_cases=None, **kwargs):
        super().__init__(*args, model_path=build_mixed_scene(), **kwargs)
        if terrain_cases is not None and len(terrain_cases) != self.num_envs:
            self.close()
            raise ValueError("Expected one terrain case per environment")
        self.terrain_cases = terrain_cases
        self.mocap_pos = self.batch.bind("mocap_pos")
        self.mocap_quat = self.batch.bind("mocap_quat")
        self.geom_xpos = self.batch.bind("geom_xpos")
        self.ramp_mocap, self.top_mocap = (
            int(self.model.body_mocapid[self.model.body(name).id])
            for name in ("spin_ramp", "spin_upper_flat"))
        self.wheels = [self.model.geom(name).id for name in
                       ("L_Wheel_mesh_1", "R_Wheel_mesh_1")]
        self.ramp_length = np.zeros(self.num_envs)
        self.ramp_height = np.zeros(self.num_envs)
        self.location = [""]*self.num_envs
        self.handover = [""]*self.num_envs
        self.takeover_time = np.zeros(self.num_envs)
        self.takeover_fade = np.ones(self.num_envs)
        self.switch_time = np.zeros(self.num_envs)
        self.dropout_time = np.full(self.num_envs, np.inf)
        self.dropout_length = np.zeros(self.num_envs)
        self.authority = np.zeros(self.num_envs)
        self.lqr_allowed = np.ones(self.num_envs, dtype=np.uint8)
        self.actual_takeover_mix = np.full(self.num_envs, np.nan)
        self.takeover_leg_difference = np.full(self.num_envs, np.nan)
        self.takeover_support_difference = np.full(self.num_envs, np.nan)
        self.takeover_wheel_zones = np.full((self.num_envs, 2), -1, dtype=np.int32)
        self.initial_wheel_heights = np.zeros((self.num_envs, 2))
        self.history = np.zeros((self.num_envs, HISTORY, FRAME_SIZE), np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf,
                                               (FRAME_SIZE*HISTORY,), np.float32)
        self.provenance.update(task=TASK_VERSION, locations=list(LOCATIONS),
                               handovers=list(HANDOVERS), frame_size=FRAME_SIZE,
                               geometry="flat + finite 2m ramp + 4m upper flat",
                               ramp_width_m=4, boundary_offset_m=[-0.2, 0.2],
                               takeover_seconds=[0.8, 4.0], fade_seconds=[0.1, 0.8],
                               dropout_probability=0.3, dropout_seconds=[7, 12],
                               residual_control=True)

    def _case(self, i):
        return {} if self.terrain_cases is None else self.terrain_cases[i]

    def _reset_ids(self, ids):
        # The parent first warms the robot on flat ground; hide both supports.
        self.mocap_pos[ids, :, 2] = -10
        self.mocap_quat[ids] = [1, 0, 0, 0]
        super()._reset_ids(ids)

    def surface_height(self, i, x, y):
        height = np.clip(np.asarray(x)/self.ramp_length[i], 0, 1)*self.ramp_height[i]
        inside = (np.asarray(x) >= 0) & (np.asarray(x) <= self.ramp_length[i]+4)
        inside &= np.abs(y) <= 2
        return self.model.geom_pos[self.floor, 2] + np.where(inside, height, 0)

    def wheel_zones(self, i):
        wheel = self.geom_xpos[i, self.wheels]
        x, y = wheel[:, 0], wheel[:, 1]
        inside = (x >= 0) & (x <= self.ramp_length[i]+4) & (np.abs(y) <= 2)
        return np.where(inside, np.where(x <= self.ramp_length[i], 1, 2), 0)

    def _place_on_terrain(self, ids):
        z0 = self.model.geom_pos[self.floor, 2]
        for i in ids:
            r, case = self.rng[i], self._case(i)
            self.location[i] = case.get("location", str(r.choice(LOCATIONS)))
            self.handover[i] = case.get("handover", str(r.choice(HANDOVERS)))
            if self.location[i] not in LOCATIONS or self.handover[i] not in HANDOVERS:
                raise ValueError("Unknown terrain location or handover")
            angle = np.deg2rad(abs(self.slope_deg[i]))
            self.slope_deg[i] = abs(self.slope_deg[i])
            length, height = 2*np.cos(angle), 2*np.sin(angle)
            self.ramp_length[i], self.ramp_height[i] = length, height
            self.mocap_pos[i, self.ramp_mocap] = [
                length/2+0.08*np.sin(angle), 0, z0+height/2-0.08*np.cos(angle)]
            self.mocap_quat[i, self.ramp_mocap] = [np.cos(angle/2), 0, -np.sin(angle/2), 0]
            self.mocap_pos[i, self.top_mocap] = [length+2, 0, z0+height-0.08]
            self.mocap_quat[i, self.top_mocap] = [1, 0, 0, 0]
            spawn = {
                "lower_flat": -r.uniform(0.4, 1.2),
                "toe": r.uniform(-0.20, 0.20),
                "ramp": r.uniform(0.3, length-0.3),
                "crest": length+r.uniform(-0.20, 0.20),
                "upper_flat": length+r.uniform(0.4, 1.5),
            }[self.location[i]]
            if "offset" in case and self.location[i] in ("toe", "crest"):
                spawn = (0 if self.location[i] == "toe" else length)+case["offset"]
            h = self.heading[i]/2
            rotation = np.array([np.cos(h), 0, 0, np.sin(h)])
            self.qpos[i, 3:7] = quat_mul(rotation, self.qpos[i, 3:7])
            mat = np.empty(9)
            mujoco.mju_quat2Mat(mat, rotation)
            self.qvel[i, :3] = mat.reshape(3, 3)@self.qvel[i, :3]
            self.takeover_time[i] = case.get("takeover_time", r.uniform(0.8, 4.0))
            self.takeover_fade[i] = case.get("fade", r.uniform(0.1, 0.8))
            self.switch_time[i] = max(0, self.takeover_time[i]-case.get(
                "switch_lead", r.uniform(0.02, 0.35)))
            if case.get("dropout", r.random() < 0.3):
                self.dropout_time[i] = r.uniform(7, 12)
                self.dropout_length[i] = r.uniform(0.2, 1.0)
            else:
                self.dropout_time[i] = np.inf
                self.dropout_length[i] = 0
            # Forward first, then place the actual wheel midpoint at the sample.
            self.qpos[i, 0] = spawn
            self.qpos[i, 1] = r.uniform(-0.5, 0.5)
        self.batch.forward(ids=ids)
        for i in ids:
            wheel = self.geom_xpos[i, self.wheels]
            requested = self.qpos[i, 0]
            self.qpos[i, 0] += requested-wheel[:, 0].mean()
        self.batch.forward(ids=ids)
        for i in ids:
            wheel = self.geom_xpos[i, self.wheels]
            heights = self.surface_height(i, wheel[:, 0], wheel[:, 1])
            self.initial_wheel_heights[i] = heights
            self.qpos[i, 2] += np.max(heights-(wheel[:, 2]-0.043))+0.002
        self.batch.forward(ids=ids)

    def _assert_floor(self, ids):
        # Both pieces retain their exact shared top edge; the base plane stays flat.
        super_normal = self.geom_xmat[ids, self.floor].reshape(-1, 3, 3)[:, :, 2]
        if not np.allclose(super_normal, [0, 0, 1], atol=1e-8):
            raise RuntimeError("Mixed terrain base plane must remain horizontal")

    def _reset_context(self, ids):
        self.authority[ids] = 0
        self.actual_takeover_mix[ids] = np.nan
        self.takeover_leg_difference[ids] = np.nan
        self.takeover_support_difference[ids] = np.nan
        self.takeover_wheel_zones[ids] = -1
        for i in ids:
            self.lqr_allowed[i] = self.handover[i] in ("lqr", "lqr_to_pid")
        self.control.set_lqr_enabled(self.lqr_allowed)

    def _frame_obs(self):
        return np.column_stack((super()._frame_obs(), self.authority,
                                self.lqr_allowed)).astype(np.float32)

    def _prepare_control(self, elapsed, actions, mask):
        self.authority[:] = np.clip((elapsed-self.takeover_time)/self.takeover_fade, 0, 1)
        dropping = elapsed >= self.dropout_time
        for i in np.flatnonzero(dropping):
            start = self.dropout_time[i]
            end = start+self.takeover_fade[i]+self.dropout_length[i]
            self.authority[i] *= (1-np.clip((elapsed[i]-start)/self.takeover_fade[i], 0, 1)
                                  + np.clip((elapsed[i]-end)/self.takeover_fade[i], 0, 1))
        for i, mode in enumerate(self.handover):
            if mode in ("pid_to_lqr", "lqr_to_pid"):
                after = elapsed[i] >= self.switch_time[i]
                self.lqr_allowed[i] = after if mode == "pid_to_lqr" else not after
        self.control.set_lqr_enabled(self.lqr_allowed)
        native = self._native_observation()
        first = np.isnan(self.actual_takeover_mix) & (self.authority > 0)
        self.actual_takeover_mix[first] = native[first, 27]
        self.takeover_leg_difference[first] = native[first, 11]-native[first, 12]
        for i in np.flatnonzero(first):
            wheel = self.geom_xpos[i, self.wheels]
            support = self.surface_height(i, wheel[:, 0], wheel[:, 1])
            self.takeover_support_difference[i] = support[0]-support[1]
            self.takeover_wheel_zones[i] = self.wheel_zones(i)
        self._apply_residual(actions*self.authority[:, None], mask)

    def _apply_residual(self, actions, mask):
        self.inputs[:, 2:] = actions
        self.control.spin_command(self.inputs, mask)

    def _extra_metrics(self, i):
        return {
            "terrain_location": self.location[i], "handover_requested": self.handover[i],
            "takeover_time_s": float(self.takeover_time[i]),
            "takeover_fade_s": float(self.takeover_fade[i]),
            "takeover_actual_lqr_mix": (float(self.actual_takeover_mix[i])
                                        if np.isfinite(self.actual_takeover_mix[i]) else None),
            "takeover_leg_difference_m": (float(self.takeover_leg_difference[i])
                                          if np.isfinite(self.takeover_leg_difference[i]) else None),
            "takeover_support_difference_m": (float(self.takeover_support_difference[i])
                                              if np.isfinite(self.takeover_support_difference[i]) else None),
            "takeover_wheel_zones": self.takeover_wheel_zones[i].tolist(),
            "dropout_time_s": float(self.dropout_time[i]) if np.isfinite(self.dropout_time[i]) else None,
            "dropout_length_s": float(self.dropout_length[i]),
            "initial_wheel_height_difference_m": float(
                self.initial_wheel_heights[i, 0]-self.initial_wheel_heights[i, 1]),
            "initial_position_xy_m": self.start_xy[i].tolist(),
        }
