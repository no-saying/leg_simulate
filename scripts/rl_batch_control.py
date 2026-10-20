"""Build and run isolated copies of the deployed C controller in one process."""

import ctypes
import hashlib
import json
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

import mujoco
import numpy as np
from jump_rl_env import ROOT, IdentFrame

ROOT = Path(ROOT)
SOURCES = [
    "scripts/ident_bridge.c",
    "control/balance/balance.c",
    "control/balance/balance_api.c",
    "control/balance/balance_util.c",
    "control/balance/balance_core.c",
    "control/balance/leg_kin_v3.c",
    "control/balance/vel_kf.c",
    "control/controllers/lqr/lqr_controller.c",
    "control/controllers/energy/energy_controller.c",
    "control/controllers/supervisor/controller_supervisor.c",
    "control/controllers/rl_jump/rl_jump_net.c",
    "control/controllers/rl_jump/rl_jump_controller.c",
    "control/runtime/robot_control.c",
    "control/lowlevel/pid.c",
    "control/lowlevel/wheel_t.c",
    "control/lowlevel/leg_t.c",
]


def build_bridges(pid_only=False, controller_defines=None):
    controller_defines = dict(controller_defines or {})
    for name, value in controller_defines.items():
        if not re.fullmatch(r"[A-Z][A-Z0-9_]*", name) or not re.fullmatch(
            r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?[fF]?", str(value)
        ):
            raise ValueError("Controller overrides must be macro names and numeric constants")
    if pid_only and any(name in controller_defines for name in (
        "CTRL_LQR_FULL_ENABLE", "CTRL_LQR_MIX_GAIN"
    )):
        raise ValueError("PID-only mode cannot override its LQR disable flags")
    inputs = sorted(
        set(
            [ROOT / s for s in SOURCES]
            + list((ROOT / "control").rglob("*.h"))
            + list((ROOT / "control/balance").glob("*.inc"))
            + [ROOT / "scripts/ident_frame.h", ROOT / "scripts/rl_batch_bridge.c"]
        )
    )
    digest = hashlib.sha256()
    digest.update(b"isolated-controller-quiet-v2")
    digest.update(b"pid-only" if pid_only else b"pid-lqr")
    if controller_defines:
        digest.update(json.dumps(controller_defines, sort_keys=True).encode())
    mujoco_dir = Path(mujoco.__file__).parent
    digest.update(mujoco.__version__.encode())
    digest.update(str(mujoco_dir).encode())
    for path in inputs:
        digest.update(str(path.relative_to(ROOT)).encode())
        digest.update(path.read_bytes())
    build = ROOT / "logs/rl_training" / digest.hexdigest()[:16]
    build.mkdir(parents=True, exist_ok=True)
    controller, batch = build / "controller.so", build / "batch.so"
    includes = [
        "-I" + str(ROOT / "control" / s)
        for s in ("include", "", "balance", "lowlevel", "runtime", "controllers")
    ]
    commands = [
        (
            controller,
            ["gcc", "-O2", "-fPIC", "-shared", "-Wl,-Bsymbolic", "-DCTRL_DEBUG=0"]
            + [f"-D{name}={value}" for name, value in sorted(controller_defines.items())]
            + (["-DCTRL_LQR_FULL_ENABLE=0", "-DCTRL_LQR_MIX_GAIN=0"] if pid_only else [])
            + [str(ROOT / s) for s in SOURCES]
            + includes
            + ["-lm"],
        ),
        (
            batch,
            [
                "gcc",
                "-O2",
                "-Wall",
                "-Wextra",
                "-fPIC",
                "-shared",
                "-fopenmp",
                str(ROOT / "scripts/rl_batch_bridge.c"),
                "-I" + str(mujoco_dir / "include"),
                str(next(mujoco_dir.glob("libmujoco.so.*"))),
                "-Wl,-rpath," + str(mujoco_dir),
                "-ldl",
                "-lm",
            ],
        ),
    ]
    for target, command in commands:
        if not target.exists():
            with tempfile.TemporaryDirectory(dir=build) as staging:
                output = Path(staging) / target.name
                subprocess.run(command + ["-o", str(output)], check=True)
                output.replace(target)
    return controller, batch


class BatchControl:
    def __init__(self, count, threads, pid_only=False, controller_defines=None):
        controller, batch = build_bridges(pid_only, controller_defines)
        self.build_id, self.pid_only = controller.parent.name, pid_only
        self._directory = tempfile.TemporaryDirectory(prefix="rl_controllers_")
        self._context = None
        self.lib = ctypes.CDLL(str(batch))
        self.lib.rl_batch_frame_size.restype = ctypes.c_size_t
        if self.lib.rl_batch_frame_size() != ctypes.sizeof(IdentFrame):
            raise RuntimeError("IdentFrame C/Python ABI mismatch")
        paths = []
        for i in range(count):
            path = Path(self._directory.name) / f"controller_{i}.so"
            shutil.copyfile(controller, path)
            paths.append(str(path).encode())
        ptr = ctypes.c_void_p
        self.lib.rl_batch_create.argtypes = [
            ctypes.POINTER(ctypes.c_char_p),
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_int,
        ]
        self.lib.rl_batch_create.restype = ptr
        self.lib.rl_batch_destroy.argtypes = [ptr]
        self.lib.rl_batch_destroy.restype = None
        self.lib.rl_batch_reset.argtypes = [ptr, ptr]
        self.lib.rl_batch_reset.restype = None
        self.lib.rl_batch_set_active.argtypes = [ptr, ptr, ctypes.c_int]
        self.lib.rl_batch_set_active.restype = None
        self.lib.rl_batch_step.argtypes = [ptr, ptr, ptr, ctypes.c_double, ptr]
        self.lib.rl_batch_step.restype = None
        self.lib.rl_batch_warmup.argtypes = [ptr, ptr, ptr, ptr, ptr, ptr, ptr]
        self.lib.rl_batch_warmup.restype = ctypes.c_int
        self.lib.rl_batch_warmup_terrain.argtypes = [ptr] * 9
        self.lib.rl_batch_warmup_terrain.restype = ctypes.c_int
        self.lib.rl_batch_warmup_terrain_imu.argtypes = [ptr] * 10
        self.lib.rl_batch_warmup_terrain_imu.restype = ctypes.c_int
        self.lib.rl_batch_lqr_enabled.argtypes = [ptr, ptr]
        self.lib.rl_batch_lqr_enabled.restype = None
        self.lib.rl_batch_spin_command.argtypes = [ptr, ptr, ptr]
        self.lib.rl_batch_spin_command.restype = None
        self.lib.rl_batch_spin_observation.argtypes = [ptr, ptr]
        self.lib.rl_batch_spin_observation.restype = None
        self.lib.rl_batch_spin_actions.argtypes = [ptr, ptr]
        self.lib.rl_batch_spin_actions.restype = None
        self.lib.rl_batch_spin_mix_audit.argtypes = [ptr, ptr]
        self.lib.rl_batch_spin_mix_audit.restype = None
        self.lib.rl_batch_leg_jacobians.argtypes = [ptr, ptr, ptr]
        self.lib.rl_batch_leg_jacobians.restype = None
        error = ctypes.create_string_buffer(1024)
        self._context = self.lib.rl_batch_create(
            (ctypes.c_char_p * count)(*paths), count, threads, error, len(error)
        )
        if not self._context:
            self.close()
            raise RuntimeError(error.value.decode() or "Cannot allocate controller batch")
        self._frames = (IdentFrame * count)()
        self.frames = np.ndarray((count,), dtype=np.dtype(IdentFrame), buffer=self._frames)
        self.outputs = np.zeros((count, 6), dtype=np.float32)
        self.spin_obs = np.zeros((count, 28), dtype=np.float32)
        self.filtered_actions = np.zeros((count, 5), dtype=np.float32)
        self.spin_mix_audit = np.zeros((count, 8), dtype=np.float32)
        self.leg_jacobian_values = np.zeros((count, 2, 2, 2), dtype=np.float32)

    def leg_jacobians(self):
        self.lib.rl_batch_leg_jacobians(self._context, ctypes.addressof(self._frames),
                                       self.leg_jacobian_values.ctypes.data)
        return self.leg_jacobian_values

    def wheel_mix_audit(self):
        self.lib.rl_batch_spin_mix_audit(self._context, self.spin_mix_audit.ctypes.data)
        return self.spin_mix_audit

    def spin_command(self, inputs, mask):
        inputs = np.ascontiguousarray(inputs, dtype=np.float32)
        mask = np.ascontiguousarray(mask, dtype=np.uint8)
        if inputs.shape != (len(self.frames), 7) or not np.isfinite(inputs).all():
            raise ValueError("Expected finite spin inputs with shape (num_envs, 7)")
        self.lib.rl_batch_spin_command(self._context, inputs.ctypes.data, mask.ctypes.data)

    def spin_observation(self):
        self.lib.rl_batch_spin_observation(self._context, self.spin_obs.ctypes.data)
        return self.spin_obs

    def set_lqr_enabled(self, enabled):
        enabled = np.ascontiguousarray(enabled, dtype=np.uint8)
        if enabled.shape != (len(self.frames),):
            raise ValueError("Expected one LQR permission per controller")
        self.lib.rl_batch_lqr_enabled(self._context, enabled.ctypes.data)

    def spin_actions(self):
        self.lib.rl_batch_spin_actions(self._context, self.filtered_actions.ctypes.data)
        return self.filtered_actions

    def reset(self, mask):
        mask = np.ascontiguousarray(mask, dtype=np.uint8)
        self.lib.rl_batch_reset(self._context, mask.ctypes.data)

    def set_active(self, mask, active):
        mask = np.ascontiguousarray(mask, dtype=np.uint8)
        self.lib.rl_batch_set_active(self._context, mask.ctypes.data, int(active))

    def step(self, mask, dt):
        mask = np.ascontiguousarray(mask, dtype=np.uint8)
        self.lib.rl_batch_step(
            self._context,
            ctypes.addressof(self._frames),
            mask.ctypes.data,
            dt,
            self.outputs.ctypes.data,
        )
        return self.outputs

    def close(self):
        if self._context:
            self.lib.rl_batch_destroy(self._context)
            self._context = None
        self._directory.cleanup()

    def warmup(self, model, states, steps, mask, sensors, forces, terrain=None, imu_delta=None):
        mask = np.ascontiguousarray(mask, dtype=np.uint8)
        steps = np.ascontiguousarray(steps, dtype=np.int32)
        extra = [] if terrain is None else [
            np.ascontiguousarray(value, dtype=np.float64) for value in terrain
        ]
        function = self.lib.rl_batch_warmup if terrain is None else self.lib.rl_batch_warmup_terrain
        if imu_delta is not None:
            if (terrain is None or imu_delta.dtype != np.float64 or not imu_delta.flags.c_contiguous
                    or imu_delta.shape != (len(self.frames), 3)):
                raise ValueError('IMU warmup requires terrain and a contiguous float64 (num_envs,3) output')
            function = self.lib.rl_batch_warmup_terrain_imu
            extra.append(imu_delta)
        ok = function(
            self._context,
            model._address,
            states.ctypes.data,
            steps.ctypes.data,
            mask.ctypes.data,
            sensors.ctypes.data,
            forces.ctypes.data,
            *(value.ctypes.data for value in extra),
        )
        if not ok:
            raise RuntimeError("Unsupported model or allocation failure in C warmup")

    def __del__(self):
        if getattr(self, "_context", None):
            self.close()
