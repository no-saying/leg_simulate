"""Run five-second spin/reverse trials on independently initialized slopes."""

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import csv
import itertools
import json
import math
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "logs/manual_verification"


def add_start_state(row):
    path = OUT / ("slip_check_%s_%+.3f_slope_%+.1f.csv" %
                  (row["command"], row["extension"], row["degrees"]))
    with path.open() as source:
        samples = list(csv.DictReader(source))
    before = [s for s in samples if float(s["time"]) < 10]
    a, b = before[-2:]
    dt = float(b["time"]) - float(a["time"])
    row["pre_spin_speed"] = math.hypot(float(b["world_x"])-float(a["world_x"]),
                                        float(b["world_y"])-float(a["world_y"]))/dt
    row["pre_spin_pitch"] = float(b["pitch"])
    row["pre_spin_roll"] = float(b["roll"])
    row["first_overturn_time"] = next(
        (float(s["time"]) for s in samples
         if abs(float(s["pitch"])) >= 90 or abs(float(s["roll"])) >= 90), -1)
    return row


def run_case(case):
    degrees, extension, command = case
    name = f"slope_spin_{degrees:+.1f}_{extension:+.3f}_{command}"
    result = subprocess.run(
        [str(ROOT / "control/build/verify_slip_controls"), command,
         str(extension), "slope", str(degrees)],
        cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        timeout=120,
    )
    (OUT / f"{name}.log").write_text(result.stdout)
    row = {"degrees": degrees, "extension": extension, "command": command,
           "exit_code": result.returncode}
    for line in result.stdout.splitlines():
        if line.startswith(("RESULT ", "DRIFT ", "SLOPE ")):
            for token in line.split():
                if "=" in token:
                    key, value = token.split("=", 1)
                    row[key] = float(value)
        elif line.startswith("VERDICT "):
            row["verdict"] = line.split()[1]
    if "verdict" not in row:
        raise RuntimeError(f"Incomplete trial: {OUT / (name + '.log')}")
    return add_start_state(row)


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--degrees", type=float, nargs="+", default=[0, 3, 5, 8, 10, 15])
    parser.add_argument("--summarize-only", action="store_true",
                        help="Add start-state diagnostics to existing JSON/CSV results.")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    cases = list(itertools.product(args.degrees, [-0.04, 0, 0.04], ["spin", "reverse"]))
    if args.summarize_only:
        rows = [add_start_state(row) for row in json.loads((OUT / "slope_spin.json").read_text())]
    else:
        rows = []
        with ThreadPoolExecutor(max_workers=4) as pool:
            for future in as_completed([pool.submit(run_case, case) for case in cases]):
                row = future.result()
                rows.append(row)
                print(f"{row['degrees']:g}deg ext={row['extension']:+.2f} {row['command']}: "
                      f"{row['verdict']} yaw={row['yaw']:.1f}deg "
                      f"excursion={row['excursion']:.3f}m "
                      f"pitch/roll={row['pitch']:.2f}/{row['roll']:.2f}deg "
                      f"fallen={row['fallen_time']:.3f}", flush=True)
    rows.sort(key=lambda row: (row["degrees"], row["extension"], row["command"]))
    (OUT / "slope_spin.json").write_text(json.dumps(rows, indent=2) + "\n")
    with (OUT / "slope_spin.csv").open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"Completed {len(rows)} trials; {sum(row['verdict'] == 'PASS' for row in rows)} passed.")
    return int(any(row["exit_code"] != 0 for row in rows))


if __name__ == "__main__":
    raise SystemExit(main())
