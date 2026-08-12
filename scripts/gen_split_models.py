#!/usr/bin/env python3
"""从 logs/split_models_latest.json 生成拆分后的 C 模型头文件。

输出:
  control/controllers/lqr/lqr_pitch_models.h  (7 状态 x 3 输入, 按腿长分段)
  control/controllers/lqr/lqr_roll_models.h   (5 状态 x 2 输入, 按腿长分段)
"""

import json
import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(PROJECT_ROOT, "logs", "split_models_latest.json")
OUT_PITCH = os.path.join(PROJECT_ROOT, "control", "controllers", "lqr",
                         "lqr_pitch_models.h")
OUT_ROLL = os.path.join(PROJECT_ROOT, "control", "controllers", "lqr",
                        "lqr_roll_models.h")

with open(SRC) as f:
    d = json.load(f)


def mat(name, rows):
    lines = []
    for r in rows:
        lines.append("      {" + ",".join(f"{v:.8e}f" for v in r) + "},")
    return f"    {{ /* {name} */\n" + "\n".join(lines) + "\n    },\n"


def gen(header, guard, seg_type, nstate, ninput, state_names, input_names,
        segments):
    out = []
    out.append(f"/* {header} - 由 scripts/gen_split_models.py 生成, 勿手改 */")
    out.append(f"#ifndef {guard}")
    out.append(f"#define {guard}")
    out.append("")
    out.append(f"#define LQR_N_SEG {len(segments)}")
    nstate_macro = "LQR_PITCH_N_STATE" if seg_type=="LqrPitchSegment" else "LQR_ROLL_N_STATE"
    out.append(f"#define {nstate_macro} {nstate}")
    ninput_macro = "LQR_PITCH_N_INPUT" if seg_type=="LqrPitchSegment" else "LQR_ROLL_N_INPUT"
    out.append(f"#define {ninput_macro} {ninput}")
    out.append("")
    out.append(f"typedef struct {{")
    out.append("    float lo, hi;")
    out.append(f"    float A[{nstate}][{nstate}];")
    out.append(f"    float B[{nstate}][{ninput}];")
    out.append(f"    float K[{ninput}][{nstate}];")
    out.append(f"    float x_trim[{nstate}];")
    out.append(f"}} {seg_type};")
    out.append("")
    arr = "lqr_pitch_segments" if seg_type == "LqrPitchSegment" else "lqr_roll_segments"
    out.append(f"static const {seg_type} {arr}[{len(segments)}] = {{")
    for lo, hi, m in segments:
        if m is None:
            continue
        out.append("  {")
        out.append(f"    /* [{lo:+.3f},{hi:+.3f}] */")
        out.append(f"    {lo:.6f}f, {hi:.6f}f,")
        out.append(mat("A", m["A"]))
        out.append(mat("B", m["B"]))
        out.append(mat("K", m["K"]))
        xt = ",".join(f"{v:.6e}f" for v in m["x_trim"])
        out.append(f"    {{ {xt} }},")
        out.append("  },")
    out.append("};")
    out.append("")
    out.append(f"/* 状态: {state_names} */")
    out.append(f"/* 输入: {input_names} */")
    out.append("")
    out.append(f"#endif /* {guard} */")
    return "\n".join(out) + "\n"


segs = []
for key in sorted(d["piecewise"].keys()):
    item = d["piecewise"][key]
    lo, hi = item["range"]
    segs.append((lo, hi, item["pitch"]))
with open(OUT_PITCH, "w") as f:
    f.write(gen("lqr_pitch_models.h", "LQR_PITCH_MODELS_H",
                "LqrPitchSegment", 7, 3,
                "pitch,pitch_rate,leg_swing,leg_swing_rate,leg_len,"
                "leg_len_rate,wheel_v",
                "wheel_common_tau,leg_swing_ref,leg_len_ref", segs))

segs = []
for key in sorted(d["piecewise"].keys()):
    item = d["piecewise"][key]
    lo, hi = item["range"]
    segs.append((lo, hi, item["roll"]))
with open(OUT_ROLL, "w") as f:
    f.write(gen("lqr_roll_models.h", "LQR_ROLL_MODELS_H",
                "LqrRollSegment", 5, 2,
                "roll,roll_rate,leg_len_diff,leg_len_diff_rate,wheel_diff_v",
                "wheel_diff_tau,leg_len_diff_ref", segs))

print(f"wrote {OUT_PITCH}")
print(f"wrote {OUT_ROLL}")
