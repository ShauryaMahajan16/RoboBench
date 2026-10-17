import csv
from collections import Counter
from pathlib import Path
from statistics import mean

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
FIGURES = ROOT / "figures"

ALGORITHMS = ("A-star", "Dijkstra", "BFS")
COLORS = ("#0072B2", "#D55E00", "#009E73")

plt.rcParams.update({
    "font.size": 10,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "pdf.fonttype": 42,
})


def read_csv(name):
    path = RESULTS / name
    if not path.is_file():
        raise ValueError(f"Missing file: {path}")

    with path.open(encoding="utf-8-sig", newline="") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise ValueError(f"No data in {path}")

    return rows


def true(value):
    return str(value).strip().lower() == "true"


def save(fig, name):
    fig.tight_layout()
    for extension in ("png", "pdf"):
        fig.savefig(
            FIGURES / f"{name}.{extension}",
            dpi=300,
            bbox_inches="tight",
        )
    plt.close(fig)
    print(f"Saved: {name}.png and {name}.pdf")


def check_unique(rows, fields, description):
    keys = [
        tuple(row[field] for field in fields)
        for row in rows
    ]
    if len(set(keys)) != len(keys):
        raise ValueError(f"Duplicate records in {description}.")


def plot_resolution():
    rows = read_csv("resolution_summary.csv")
    check_unique(
        rows, ("seed", "grid_size_px", "algorithm"),
        "resolution summary",
    )

    grids = (10, 20, 40)
    groups = {}
    for algorithm in ALGORITHMS:
        for grid in grids:
            group = [
                row for row in rows
                if row["algorithm"] == algorithm
                and int(row["grid_size_px"]) == grid
            ]
            if len(group) != 100:
                raise ValueError(
                    f"Expected 100 resolution rows for {algorithm}, {grid}px."
                )
            groups[(algorithm, grid)] = group

    for grid in grids:
        reference = {
            row["seed"]: true(row["route_found"])
            for row in groups[("Dijkstra", grid)]
        }
        for algorithm in ALGORITHMS:
            found = {
                row["seed"]: true(row["route_found"])
                for row in groups[(algorithm, grid)]
            }
            if found != reference:
                raise ValueError("Resolution reachability disagreement.")

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))

    counts = [
        sum(true(row["route_found"])
            for row in groups[("Dijkstra", grid)])
        for grid in grids
    ]
    axes[0].bar(
        [str(grid) for grid in grids], counts, color=COLORS[0]
    )
    axes[0].set(
        xlabel="Grid spacing (pixels)",
        ylabel="Maps with a route",
        title="Route availability",
        ylim=(0, 105),
    )
    for index, count in enumerate(counts):
        axes[0].text(index, count + 1, f"{count}/100", ha="center")

    for algorithm, color in zip(ALGORITHMS, COLORS):
        timings = [
            mean(float(row["median_planning_ms"])
                 for row in groups[(algorithm, grid)])
            for grid in grids
        ]
        axes[1].plot(
            grids, timings, marker="o", label=algorithm, color=color
        )

    axes[1].set(
        xlabel="Grid spacing (pixels)",
        ylabel="Mean of map median planning times (ms)",
        title="Planning time, including no-route maps",
        xticks=grids,
    )
    axes[1].legend()
    save(fig, "01_grid_resolution")


def plot_adaptive():
    rows = read_csv("adaptive_summary.csv")
    check_unique(
        rows, ("passage_width_px", "passage_centre_y", "method"),
        "adaptive summary",
    )
    methods = list(dict.fromkeys(row["method"] for row in rows))
    if len(methods) != 2:
        raise ValueError("Expected two adaptive-experiment methods.")

    fig, ax = plt.subplots(figsize=(8, 4))
    positions = (0, 1)
    width = 0.34

    for index, method in enumerate(methods):
        method_rows = [row for row in rows if row["method"] == method]
        if len(method_rows) != 244:
            raise ValueError(f"Expected 244 cases for {method}.")

        timings = []
        for reachable in (True, False):
            group = [
                row for row in method_rows
                if true(row["route_found"]) == reachable
            ]
            if not group:
                raise ValueError("Missing adaptive outcome group.")
            timings.append(
                mean(float(row["median_planning_ms"]) for row in group)
            )

        locations = [
            position + (index - 0.5) * width
            for position in positions
        ]
        ax.bar(
            locations, timings, width,
            label=method, color=COLORS[index],
        )

    ax.set(
        xticks=positions,
        xticklabels=("Route found", "No route"),
        ylabel="Mean of case median planning times (ms)",
        title="Fixed fine grid versus resolution retry",
    )
    ax.legend()
    save(fig, "02_resolution_retry")


def plot_motion():
    rows = sorted(
        read_csv("motion_summary.csv"),
        key=lambda row: float(row["noise_sigma_px"]),
    )
    check_unique(rows, ("noise_sigma_px",), "motion summary")

    noise = [float(row["noise_sigma_px"]) for row in rows]
    rates = [
        100 * int(row["successes"]) / int(row["trials"])
        for row in rows
    ]
    distances = [
        float(row["mean_success_distance_px"]) for row in rows
    ]

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].plot(noise, rates, "o-", color=COLORS[0])
    axes[0].set(
        xlabel="Noise sigma (pixels per axis per step)",
        ylabel="Successful trials (%)",
        title="Motion success on 19 planned maps",
        ylim=(0, 105),
    )
    axes[1].plot(noise, distances, "o-", color=COLORS[1])
    axes[1].set(
        xlabel="Noise sigma (pixels per axis per step)",
        ylabel="Mean travelled distance (pixels)",
        title="Successful trials only",
    )
    save(fig, "03_motion_noise")


def plot_clearance():
    rows = sorted(
        read_csv("clearance_summary.csv"),
        key=lambda row: float(row["extra_clearance_px"]),
    )
    check_unique(rows, ("extra_clearance_px",), "clearance summary")

    labels = [row["extra_clearance_px"] for row in rows]
    positions = list(range(len(rows)))
    bottom = [0.0] * len(rows)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    outcomes = (
        ("successes", "Success", "#009E73"),
        ("collisions", "Collision", "#D55E00"),
        ("no_route", "No route", "#999999"),
        ("timeouts", "Timeout", "#CC79A7"),
    )

    for row in rows:
        total = sum(int(row[field]) for field, _, _ in outcomes)
        if total != int(row["total_trials"]):
            raise ValueError("Clearance outcome counts do not sum to total.")

    for field, label, color in outcomes:
        percentages = [
            100 * int(row[field]) / int(row["total_trials"])
            for row in rows
        ]
        axes[0].bar(
            positions, percentages, bottom=bottom,
            label=label, color=color,
        )
        bottom = [
            a + b for a, b in zip(bottom, percentages)
        ]

    axes[0].set(
        xticks=positions,
        xticklabels=labels,
        xlabel="Extra planning clearance (pixels)",
        ylabel="All recorded cases (%)",
        title="Completion versus collision and route availability",
        ylim=(0, 100),
    )
    axes[0].legend(fontsize=8, loc="lower left")

    common_counts = {int(row["common_maps"]) for row in rows}
    if len(common_counts) != 1:
        raise ValueError("Inconsistent common-map counts.")

    axes[1].plot(
        [float(value) for value in labels],
        [float(row["success_rate_common_percent"]) for row in rows],
        "o-", color=COLORS[0],
    )
    axes[1].set(
        xlabel="Extra planning clearance (pixels)",
        ylabel="Successful trials (%)",
        title=f"Matched subset: {common_counts.pop()} common maps",
        ylim=(0, 105),
    )
    save(fig, "04_planning_clearance")


def plot_webots():
    rows = read_csv("webots_layout_trials.csv")
    latest_run = rows[-1]["run_id"]
    rows = [row for row in rows if row["run_id"] == latest_run]

    layouts = ("Centre", "Right-up", "Left-down")
    expected = {
        (layout, algorithm)
        for layout in layouts
        for algorithm in ALGORITHMS
    }
    counts = Counter(
        (row["layout"], row["algorithm"]) for row in rows
    )
    if set(counts) != expected or any(value != 1 for value in counts.values()):
        raise ValueError("Latest Webots run must contain all nine trials.")

    if any(
        row["outcome"] != "GOAL REACHED"
        or row["box_collision_detected"].strip().lower() != "false"
        for row in rows
    ):
        raise ValueError(
            "This execution comparison expects nine collision-free successes."
        )

    lookup = {
        (row["layout"], row["algorithm"]): row
        for row in rows
    }
    fig, ax = plt.subplots(figsize=(9, 4))
    width = 0.24

    for index, (algorithm, color) in enumerate(zip(ALGORITHMS, COLORS)):
        locations = [
            position + (index - 1) * width
            for position in range(len(layouts))
        ]
        times = [
            float(lookup[(layout, algorithm)]["simulation_time_s"])
            for layout in layouts
        ]
        ax.bar(locations, times, width, label=algorithm, color=color)

    ax.set(
        xticks=range(len(layouts)),
        xticklabels=layouts,
        ylabel="Execution duration (simulation seconds)",
        title="Webots: one trial per planner per layout",
    )
    ax.legend()
    print(f"Webots run plotted: {latest_run}")
    save(fig, "05_webots_execution")


def main():
    FIGURES.mkdir(exist_ok=True)
    plot_resolution()
    plot_adaptive()
    plot_motion()
    plot_clearance()
    plot_webots()
    print(f"\nFigures saved in: {FIGURES}")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, KeyError) as error:
        raise SystemExit(f"Cannot finish plotting: {error}")