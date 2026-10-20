"""Generate convex wheel collision pieces from the original CAD mesh.

Requires trimesh and scipy in the tooling environment. Explicit body inertias
are retained for isolated contact experiments. Fixed-motor mode splits visual
meshes as well; generated pieces use the STL's millimetre frame.
"""

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import trimesh
from scipy.sparse.csgraph import connected_components

ROOT = Path(__file__).resolve().parents[2]
WHEEL_SHA256 = {
    "left": "8338b4deecd01a4ddf4800855d9ab96042227730af312814d620d8cfdf679745",
    "right": "6b38b97178354b78ba00c226e7a2c13ff521cbd35c44ea1ff51ddd31acbb7756",
}


def pose(element):
    position = np.fromstring(element.get("pos", "0 0 0"), sep=" ")
    quaternion = np.fromstring(element.get("quat", "1 0 0 0"), sep=" ")
    if element.get("euler"):
        mujoco.mju_euler2Quat(quaternion, np.fromstring(element.get("euler"), sep=" "), "xyz")
    return position, quaternion


def assemble(directory, output, preserve_terrain=False, fixed_motor=False):
    tree = ET.parse(ROOT / "model/v3/COD-2026RoboMaster-Balance-v3.xml")
    root = tree.getroot()
    root.find("compiler").set("meshdir", str(ROOT / "model/v3"))
    if preserve_terrain:
        # MuJoCo accepts a pair if EITHER directional bitmask test succeeds.
        # Terrain=16, robot=2, original wheel hull=8, detailed wheel=4.
        for geom in root.find("worldbody").iter("geom"):
            terrain = geom.get("name", "").startswith(("floor", "test_ball_", "large_ball_"))
            geom.set("contype", "16" if terrain else "2")
            geom.set("conaffinity", "26" if terrain else "22")
    for side, name in (("left", "L_Wheel_mesh_1"), ("right", "R_Wheel_mesh_1")):
        record = json.loads((directory / f"wheel_{side}.json").read_text())
        if fixed_motor and (record["source_sha256"] != WHEEL_SHA256[side] or record["pieces"] != 22):
            raise ValueError("Wheel CAD changed; recheck fixed motor solid ownership")
        parent = next(body for body in root.iter("body")
                      if any(g.get("name") == name for g in body.findall("geom")))
        carrier = next(body for body in root.iter("body") if parent in list(body))
        original = next(g for g in parent.findall("geom") if g.get("name") == name)
        original.set("contype", "8" if preserve_terrain else "0")
        original.set("conaffinity", "16" if preserve_terrain else "0")
        if fixed_motor:
            original.set("rgba", "0 0 0 0")
            original.set("material", "")
        for i in range(record["pieces"]):
            mesh_name = f"wheel_{side}_collision_{i:02d}"
            ET.SubElement(root.find("asset"), "mesh", name=mesh_name,
                          file=str((directory / f"wheel_{side}_{i:02d}.obj").resolve()),
                          scale="0.001 0.001 0.001")
            attrs = {k: original.get(k) for k in ("pos", "quat", "euler") if original.get(k)}
            destination = parent
            if fixed_motor and i >= 13:
                # CAD solids 13..21 are the motor can, PCB, connector and phase leads.
                # Their zero-angle pose is preserved in the fixed wheel carrier.
                position, quaternion = np.empty(3), np.empty(4)
                mujoco.mju_mulPose(position, quaternion, *pose(parent), *pose(original))
                attrs = {"pos": " ".join(map(str, position)),
                         "quat": " ".join(map(str, quaternion))}
                destination = carrier
            ET.SubElement(destination, "geom", name=mesh_name, type="mesh", mesh=mesh_name,
                          group="3", rgba="0 0 0 0", material="",
                          **({"contype": "4", "conaffinity": "2"} if preserve_terrain else {}),
                          **attrs)
            if fixed_motor:
                visual = f"wheel_{side}_visual_{i:02d}"
                ET.SubElement(root.find("asset"), "mesh", name=visual,
                              file=str((directory / f"wheel_{side}_solid_{i:02d}.obj").resolve()),
                              scale="0.001 0.001 0.001")
                ET.SubElement(destination, "geom", name=visual, type="mesh", mesh=visual,
                              group="1", contype="0", conaffinity="0", **attrs)
    ET.indent(tree)
    tree.write(output, encoding="utf-8", xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--side", choices=["left", "right"])
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--model-out", type=Path)
    parser.add_argument("--preserve-terrain", action="store_true",
                        help="Keep original wheel hulls for terrain; detailed pieces for self contact")
    parser.add_argument("--fixed-motor", action="store_true",
                        help="Attach motor can, electronics and phase leads to the fixed carrier")
    args = parser.parse_args()
    if args.side is None:
        if args.model_out is None:
            parser.error("--side or --model-out is required")
        assemble(args.out, args.model_out, args.preserve_terrain, args.fixed_motor)
        return
    source = ROOT / "model/v3/meshes" / (
        "part_009_v3.stl" if args.side == "left" else "part_008_v3.stl")
    mesh = trimesh.load_mesh(source)
    # CAD solids share vertices at nonmanifold seams; face adjacency splits them.
    count, labels = connected_components(mesh.edges_sparse, directed=False)
    groups = labels[mesh.faces[:, 0]]
    parts = [mesh.submesh([np.flatnonzero(groups == i)], append=True)
             for i in range(count)]
    args.out.mkdir(parents=True, exist_ok=True)
    for i, part in enumerate(parts):
        part.convex_hull.export(args.out / f"wheel_{args.side}_{i:02d}.obj")
        part.export(args.out / f"wheel_{args.side}_solid_{i:02d}.obj")
    record = {"source": str(source.relative_to(ROOT)),
              "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
              "algorithm": "convex_hull_per_vertex_connected_cad_solid", "pieces": len(parts)}
    (args.out / f"wheel_{args.side}.json").write_text(json.dumps(record, indent=2)+"\n")
    print(json.dumps(record), flush=True)


if __name__ == "__main__":
    main()
