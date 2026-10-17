import csv
from collections import Counter
from pathlib import Path
from statistics import mean


ROOT = Path(__file__).resolve().parent
INPUT_FILE = ROOT / "results" / "webots_layout_trials.csv"
REPORT_FILE = ROOT / "WEBOTS_LAYOUT_RESULTS.md"

LAYOUTS = ("Centre", "Right-up", "Left-down")
ALGORITHMS = ("A-star", "Dijkstra", "BFS")


def is_true(value):
    return str(value).strip().lower() == "true"


def number(row, field):
    return float(row[field])


def main():
    if not INPUT_FILE.is_file():
        raise SystemExit(
            f"Results file not found:\n{INPUT_FILE}\n"
            "Run the nine-layout Webots experiment first."
        )

    with INPUT_FILE.open(encoding="utf-8-sig", newline="") as file:
        reader = csv.DictReader(file)
        required = {
            "run_id",
            "layout",
            "algorithm",
            "outcome",
            "box_collision_detected",
            "planned_distance_m",
            "travelled_distance_m",
            "simulation_time_s",
            "route_segments",
        }
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise SystemExit(
                "Missing CSV columns: " + ", ".join(sorted(missing))
            )
        all_rows = list(reader)

    if not all_rows:
        raise SystemExit("The results file contains no trials.")

    latest_run = all_rows[-1]["run_id"]
    rows = [row for row in all_rows if row["run_id"] == latest_run]

    expected = {
        (layout, algorithm)
        for layout in LAYOUTS
        for algorithm in ALGORITHMS
    }
    counts = Counter((row["layout"], row["algorithm"]) for row in rows)

    if set(counts) != expected or any(count != 1 for count in counts.values()):
        raise SystemExit(
            "The latest run must contain exactly one trial for each "
            "of the three algorithms on each of the three layouts.\n"
            "Complete all nine Webots trials before analyzing this run."
        )

    lookup = {
        (row["layout"], row["algorithm"]): row
        for row in rows
    }

    common_successful = [
        layout
        for layout in LAYOUTS
        if all(
            lookup[(layout, algorithm)]["outcome"] == "GOAL REACHED"
            and not is_true(
                lookup[(layout, algorithm)]["box_collision_detected"]
            )
            for algorithm in ALGORITHMS
        )
    ]

    equal_distance_layouts = []
    for layout in LAYOUTS:
        planned = [
            lookup[(layout, algorithm)]["planned_distance_m"]
            for algorithm in ALGORITHMS
        ]
        if all(value.strip() for value in planned):
            distances = [float(value) for value in planned]
            if max(distances) - min(distances) <= 1e-9:
                equal_distance_layouts.append(layout)

    print(f"Run: {latest_run}")
    print(f"Trials analyzed: {len(rows)}")
    print(
        "Layouts with collision-free success for all planners: "
        f"{len(common_successful)}/{len(LAYOUTS)}"
    )
    print(
        "Layouts with equal planned distances for all planners: "
        f"{len(equal_distance_layouts)}/{len(LAYOUTS)}"
    )

    report = [
        "# RoboBench: Three-layout Webots experiment",
        "",
        f"Run: `{latest_run}`",
        "",
        "Three planners were tested on three box placements, giving nine "
        "trials. Each planner used the same route-following controller. "
        "Obstacle footprints were calculated from the Webots box geometry.",
        "",
        "## Individual trials",
        "",
        "| Layout | Planner | Outcome | Box collision | Planned (m) | "
        "Travelled (m) | Segments | Execution (simulation s) |",
        "|---|---|---|---|---:|---:|---:|---:|",
    ]

    for layout in LAYOUTS:
        for algorithm in ALGORITHMS:
            row = lookup[(layout, algorithm)]
            planned = row["planned_distance_m"].strip()
            planned_text = f"{float(planned):.3f}" if planned else "N/A"
            collision = row["box_collision_detected"].strip()
            collision_text = (
                str(is_true(collision)) if collision else "Not assessed"
            )

            report.append(
                f"| {layout} | {algorithm} | {row['outcome']} | "
                f"{collision_text} | {planned_text} | "
                f"{number(row, 'travelled_distance_m'):.3f} | "
                f"{row['route_segments']} | "
                f"{number(row, 'simulation_time_s'):.2f} |"
            )

    report.extend([
        "",
        "## Comparison on common successful layouts",
        "",
        "Means below include only layouts where all three planners "
        "reached the goal without a detected box collision.",
        "",
        "| Planner | Goals reached / trials | Detected box collisions | "
        "Common layouts | Mean execution (simulation s) |",
        "|---|---:|---:|---:|---:|",
    ])

    print()
    for algorithm in ALGORITHMS:
        algorithm_rows = [
            row for row in rows if row["algorithm"] == algorithm
        ]
        successes = sum(
            row["outcome"] == "GOAL REACHED" for row in algorithm_rows
        )
        collisions = sum(
            is_true(row["box_collision_detected"])
            for row in algorithm_rows
        )

        times = [
            number(lookup[(layout, algorithm)], "simulation_time_s")
            for layout in common_successful
        ]
        mean_text = f"{mean(times):.2f}" if times else "N/A"

        print(
            f"{algorithm:<8} | Goals: {successes}/3 | "
            f"Box collisions: {collisions} | "
            f"Mean execution: {mean_text} simulation seconds"
        )

        report.append(
            f"| {algorithm} | {successes}/3 | {collisions} | "
            f"{len(times)} | {mean_text} |"
        )

    report.extend([
        "",
        "## Interpretation and limitations",
        "",
        f"All three planners had equal planned distances on "
        f"{len(equal_distance_layouts)} of the three layouts.",
        "",
        "Equal planned distance does not imply equal execution time. "
        "Planners can select different shortest routes, with different "
        "segment lengths and turn requirements. The controller slows "
        "near waypoints, so route structure can affect execution time.",
        "",
        "Execution times measure simulated robot motion, not planning "
        "computation time or elapsed time on the computer.",
        "",
        "The robot uses perfect position and orientation feedback from "
        "the Webots Supervisor. This experiment adds no motion noise. "
        "Collision monitoring covers the named box.",
        "",
        "There is one trial per planner per layout. These results describe "
        "this small experiment and do not establish general performance "
        "rankings or reliability.",
        "",
        "## Reproduce the analysis",
        "",
        "Run the nine-layout experiment using the "
        "`robobench_layout_trials` controller, then run:",
        "",
        "```powershell",
        r".\.venv\Scripts\python.exe analyze_webots_layouts.py",
        "```",
        "",
        "This report summarizes the latest run in "
        "`results/webots_layout_trials.csv`.",
        "",
    ])

    REPORT_FILE.write_text("\n".join(report), encoding="utf-8")
    print(f"\nReport saved to: {REPORT_FILE}")


if __name__ == "__main__":
    main()