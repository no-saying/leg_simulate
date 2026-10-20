"""Check collision routing and rigid ownership without changing the inertial plant."""

import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

trimesh = pytest.importorskip("trimesh")
sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_wheel_colliders_v3 import ROOT, assemble


def pair_enabled(model, a, b):
    a, b = model.geom(a).id, model.geom(b).id
    return bool((model.geom_contype[a] & model.geom_conaffinity[b])
                | (model.geom_contype[b] & model.geom_conaffinity[a]))


def test_fixed_motor_and_collision_routing(tmp_path):
    for side in ("left", "right"):
        subprocess.run([sys.executable, str(ROOT / "scripts/tools/build_wheel_colliders_v3.py"),
                        "--side", side, "--out", str(tmp_path)], check=True, capture_output=True)
    xml = tmp_path / "candidate.xml"
    assemble(tmp_path, xml, preserve_terrain=True, fixed_motor=True)
    original = mujoco.MjModel.from_xml_path(str(ROOT / "model/v3/COD-2026RoboMaster-Balance-v3.xml"))
    candidate = mujoco.MjModel.from_xml_path(str(xml))
    for key in ("body_mass", "body_inertia", "body_ipos", "body_iquat", "jnt_axis",
                "dof_damping", "dof_armature", "actuator_gear"):
        np.testing.assert_array_equal(getattr(original, key), getattr(candidate, key), err_msg=key)
    first, second = mujoco.MjData(original), mujoco.MjData(candidate)
    first.qpos[2] = second.qpos[2] = -0.03
    first.qpos[3:7] = second.qpos[3:7] = [2**-0.5, 2**-0.5, 0, 0]
    mujoco.mj_forward(original, first)
    mujoco.mj_forward(candidate, second)
    initial_motor = []
    for side, wheel, link, joint in (
        ("left", "L_Wheel_mesh_1", "L_claf_link_1_mesh_1", "Left_Wheel_joint"),
        ("right", "R_Wheel_mesh_1", "R_Claf_link_1_mesh_1", "Right_Wheel_joint"),
    ):
        assert pair_enabled(candidate, "floor", wheel)
        assert not pair_enabled(candidate, wheel, link)
        assert pair_enabled(candidate, f"wheel_{side}_collision_00", link)
        assert not pair_enabled(candidate, "floor", f"wheel_{side}_collision_00")
        assert pair_enabled(candidate, "floor", "test_ball_01_geom")
        assert pair_enabled(candidate, "floor", "base_link_mesh_1")
        for key in ("geom_pos", "geom_quat", "geom_friction", "geom_solref", "geom_solimp"):
            np.testing.assert_array_equal(getattr(original, key)[original.geom(wheel).id],
                                          getattr(candidate, key)[candidate.geom(wheel).id])
        # Every source triangle has the same world pose at wheel angle zero.
        for i in range(22):
            name = f"wheel_{side}_visual_{i:02d}"
            gid = candidate.geom(name).id
            mesh = candidate.geom_dataid[gid]
            a, n = candidate.mesh_vertadr[mesh], candidate.mesh_vertnum[mesh]
            vertices = candidate.mesh_vert[a:a+n]
            world = vertices @ second.geom_xmat[gid].reshape(3, 3).T + second.geom_xpos[gid]
            # Raw OBJ is in the original STL frame, before MuJoCo recenters it.
            raw = trimesh.load_mesh(tmp_path / f"wheel_{side}_solid_{i:02d}.obj").vertices*0.001
            # Use the original wheel body transform and its XML geom offset.
            offset = np.array([-0.0017462873, 0.039292568,
                               -0.1195 if side == "left" else 0.132875])
            body = candidate.geom(wheel).bodyid[0]
            expected = (raw+offset) @ second.xmat[body].reshape(3, 3).T + second.xpos[body]
            np.testing.assert_allclose(np.sort(world, axis=0), np.sort(expected, axis=0), atol=2e-8)
        motor = candidate.geom(f"wheel_{side}_visual_19").id
        rim = candidate.geom(f"wheel_{side}_visual_00").id
        initial_motor.append((motor, second.geom_xpos[motor].copy(), rim, second.geom_xpos[rim].copy()))
        second.qpos[candidate.joint(joint).qposadr[0]] = 1.2
    mujoco.mj_forward(candidate, second)
    for motor, stationary, rim, rotating in initial_motor:
        np.testing.assert_allclose(second.geom_xpos[motor], stationary, atol=1e-12)
        assert np.linalg.norm(second.geom_xpos[rim]-rotating) > 0.001
