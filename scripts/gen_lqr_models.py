#!/usr/bin/env python3
"""从 logs/three_loop_model_latest.json 生成 control/controllers/lqr/lqr_models.h。

模型来源: scripts/identify_three_loop.py(三环基线+腿长全行程辨识)。
输出: 12 状态 x 5 输入分段 LQR 模型(按腿长分段), 每个分段含 A/B/K/x_trim。
"""

import json
import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(PROJECT_ROOT, "logs", "three_loop_model_latest.json")
DST = os.path.join(PROJECT_ROOT, "control", "controllers", "lqr", "lqr_models.h")

with open(SRC) as f:
    d = json.load(f)

segs = []
for k in sorted(d["piecewise"].keys()):
    item = d["piecewise"][k]
    if item["model"] is None:
        continue
    lo, hi = item["range"]
    m = item["model"]
    segs.append((lo, hi, m))

if not segs:
    raise SystemExit("no piecewise segments in json")

def mat(name, rows):
    lines = []
    for r in rows:
        lines.append("      {" + ",".join(f"{v:.8e}f" for v in r) + "},")
    return f"    {{ /* {name} */\n" + "\n".join(lines) + "\n    },\n"

out = []
out.append("/* lqr_models.h - 由 scripts/gen_lqr_models.py 生成, 勿手改 */")
out.append("#ifndef LQR_MODELS_H")
out.append("#define LQR_MODELS_H")
out.append("")
out.append(f"#define LQR_N_SEG {len(segs)}")
out.append("#define LQR_N_STATE 12")
out.append("#define LQR_N_INPUT 5")
out.append("")
out.append("typedef struct {")
out.append("    float lo, hi;")
out.append("    float A[12][12];")
out.append("    float B[12][5];")
out.append("    float K[5][12];")
out.append("    float x_trim[12];")
out.append("} LqrSegment;")
out.append("")
out.append(f"static const LqrSegment lqr_segments[{len(segs)}] = {{")
for lo, hi, m in segs:
    out.append("  {")
    out.append(f"    /* [{lo:+.3f},{hi:+.3f}] */")
    out.append(f"    {lo:.6f}f, {hi:.6f}f,")
    out.append(mat("A", m["A"]))
    out.append(mat("B", m["B"]))
    out.append(mat("K", m["K_lqr"]))
    xt = ",".join(f"{v:.6e}f" for v in m["x_trim"])
    out.append(f"    {{ {xt} }},")
    out.append("  },")
out.append("};")
out.append("")
out.append("#endif /* LQR_MODELS_H */")

os.makedirs(os.path.dirname(DST), exist_ok=True)
with open(DST, "w") as f:
    f.write("\n".join(out) + "\n")

print(f"wrote {DST}: {len(segs)} segments")
