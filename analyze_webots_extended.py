import csv
import math
from collections import Counter
from pathlib import Path
from statistics import mean, median


ROOT = Path(__file__).resolve().parent
INPUT = ROOT / "data" / "webots_extended_trials.csv"
REPORT = ROOT / "WEBOTS_EXTENDED_RESULTS.md"

ALGORITHMS = ("A-star", "Dijkstra", "BFS")
LAYOUTS = (
    "Centre",
    "Right",
    "Left",
    "Up",
    "Down",
    "Right-up",
    "Right-down",
    "Left-up",
    "Left-down",
)
REPETITIONS = (1, 2, 3)


def successful(row):
    return (
        row["outcome"] == "GOAL REACHED"
        and row["box_collision_detected"].strip().lower() == "false"
    )


def numeric(row, field):
    value = float(row[field])
    if not math.isfinite(value):
        raise ValueError(f"Invalid {field}: {row[field]}")
    return value


def main():
    if not INPUT.is_file():
        raise ValueError(f"Missing archived data: {INPUT}")

    with INPUT.open(encoding="utf-8-sig", newline="") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise ValueError("The CSV contains no trials.")

    latest_run = rows[-1]["run_id"]
    rows = [row for row in rows if row["run_id"] == latest_run]

    expected = {
        (layout, algorithm, repetition)
        for layout in LAYOUTS
        for algorithm in ALGORITHMS
        for repetition in REPETITIONS
    }
    counts = Counter(
        (row["layout"], row["algorithm"], int(row["repetition"]))
        for row in rows
    )

    if set(counts) != expected or any(n != 1 for n in counts.values()):
        raise ValueError(
            "The latest run must contain exactly 81 trials: "
            "nine layouts, three planners, and three repetitions."
        )

    groups = {
        (layout, algorithm): [
            row for row in rows
            if row["layout"] == layout and row["algorithm"] == algorithm
        ]
        for layout in LAYOUTS
        for algorithm in ALGORITHMS
    }

    common_layouts = [
        layout for layout in LAYOUTS
        if all(
            successful(row)
            for algorithm in ALGORITHMS
            for row in groups[(layout, algorithm)]
        )
    ]

    equal_distance_layouts = []
    for layout in LAYOUTS:
        layout_rows = [row for row in rows if row["layout"] == layout]
        if all(row["planned_distance_m"].strip() for row in layout_rows):
            distances = [
                numeric(row, "planned_distance_m") for row in layout_rows
            ]
            if max(distances) - min(distances) <= 1e-9:
                equal_distance_layouts.append(layout)

    print(f"Run: {latest_run}")
    print(f"Trials analyzed: {len(rows)}/81")
    print(
        "Layouts with collision-free success on every trial: "
        f"{len(common_layouts)}/9"
    )
    print(
        "Layouts with equal planned distances across all trials: "
        f"{len(equal_distance_layouts)}/9"
    )

    report = [
        "# RoboBench: Extended Webots Results",
        "",
        f"Run: `{latest_run}`",
        "",
        "The experiment tested nine placements of one static box, "
        "three planners, and three repetitions per combination: 81 trials.",
        "",
        "Repetitions used the same nominal starting pose and controller. "
        "No motion noise was deliberately added. They assess consistency "
        "and are not independent environment samples.",
        "",
        "## Outcomes",
        "",
        "| Planner | Goal reached | Box contact | Obstacle stop | "
        "Timeout | No route |",
        "|---|---:|---:|---:|---:|---:|",
    ]

    print()
    for algorithm in ALGORITHMS:
        selected = [row for row in rows if row["algorithm"] == algorithm]
        outcomes = Counter(row["outcome"] for row in selected)
        collisions = sum(
            row["box_collision_detected"].strip().lower() == "true"
            for row in selected
        )
        print(
            f"{algorithm:<8} | Goals: {outcomes['GOAL REACHED']}/27 | "
            f"Box contacts: {collisions} | "
            f"Other outcomes: {len(selected) - outcomes['GOAL REACHED']}"
        )
        report.append(
            f"| {algorithm} | {outcomes['GOAL REACHED']}/27 | "
            f"{collisions} | {outcomes['OBSTACLE STOP']} | "
            f"{outcomes['TIMEOUT']} | {outcomes['NO ROUTE']} |"
        )

    report.extend([
        "",
        "## Execution by layout",
        "",
        "Each cell reports median execution duration and the full "
        "minimum-to-maximum range across three repetitions. "
        "Only layouts where every trial succeeded without detected "
        "box contact are included.",
        "",
        "| Layout | A-star: median [min, max] s | "
        "Dijkstra: median [min, max] s | BFS: median [min, max] s |",
        "|---|---:|---:|---:|",
    ])

    for layout in common_layouts:
        cells = []
        for algorithm in ALGORITHMS:
            times = [
                numeric(row, "simulation_time_s")
                for row in groups[(layout, algorithm)]
            ]
            cells.append(
                f"{median(times):.3f} "
                f"[{min(times):.3f}, {max(times):.3f}]"
            )
        report.append(f"| {layout} | " + " | ".join(cells) + " |")

    report.extend([
        "",
        "## Paired-layout summary",
        "",
        f"Common successful layouts: {len(common_layouts)}/9.",
        "",
        "Each layout receives equal weight. The reported mean averages "
        "the three-repetition median from each common successful layout.",
        "",
        "| Planner | Mean layout median (s) | "
        "Largest within-layout repeat range (s) |",
        "|---|---:|---:|",
    ])

    print("\nExecution on common successful layouts:")
    for algorithm in ALGORITHMS:
        medians = []
        ranges = []
        for layout in common_layouts:
            times = [
                numeric(row, "simulation_time_s")
                for row in groups[(layout, algorithm)]
            ]
            medians.append(median(times))
            ranges.append(max(times) - min(times))

        if medians:
            average = mean(medians)
            largest_range = max(ranges)
            print(
                f"{algorithm:<8} | Mean layout median: {average:.3f} s | "
                f"Largest repeat range: {largest_range:.6f} s"
            )
            report.append(
                f"| {algorithm} | {average:.3f} | {largest_range:.6f} |"
            )
        else:
            print(f"{algorithm}: no common successful layouts.")
            report.append(f"| {algorithm} | N/A | N/A |")

    report.extend([
        "",
        "## Interpretation and limitations",
        "",
        f"Planned distances matched across planners and repetitions on "
        f"{len(equal_distance_layouts)}/9 layouts.",
        "",
        "Execution times are simulation durations, not planning "
        "computation times or wall-clock durations.",
        "",
        "A small repeat range indicates consistency under the tested "
        "reset procedure. It does not establish robustness to uncertainty.",
        "",
        "The layouts differ only in the position of one box. The study "
        "does not represent diverse obstacle shapes, multiple obstacles, "
        "sensor uncertainty, or physical robot trials.",
        "",
        "The controller uses perfect Supervisor pose feedback. Contact "
        "monitoring covers the named obstacle box. Proximity-triggered "
        "stops are recorded separately from physical box contacts.",
        "",
        "Execution comparisons conditioned on common successful layouts "
        "must be read alongside the all-trial outcome counts.",
        "",
        "No confidence intervals or significance tests are reported.",
        "",
        "## Reproduce",
        "",
        "Read archived trials and regenerate this report:",
        "",
        "```powershell",
        r".\.venv\Scripts\python.exe analyze_webots_extended.py",
        "```",
        "",
        "Input: `data/webots_extended_trials.csv`.",
        "",
        "This report describes a separate experiment from the earlier "
        "nine-trial study; it does not overwrite those historical results.",
        "",
    ])

    REPORT.write_text("\n".join(report), encoding="utf-8")
    print(f"\nReport saved to: {REPORT}")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, KeyError) as error:
        raise SystemExit(f"Analysis stopped: {error}")