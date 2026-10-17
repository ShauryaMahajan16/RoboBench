import csv
import math
from collections import Counter
from pathlib import Path
from statistics import mean

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parent
INPUT = ROOT / "data" / "webots_waypoint_test.csv"
OUTPUT = ROOT / "figures"

CONDITIONS = ("Original waypoints", "Extra midpoints")
COMPONENTS = (
    ("move_command_time_s", "Moving", "#0072B2"),
    ("turn_command_time_s", "Turning", "#D55E00"),
    ("idle_command_time_s", "Idle", "#999999"),
)


def number(row, field):
    value = float(row[field])
    if not math.isfinite(value):
        raise ValueError(f"Invalid number in {field}.")
    return value


def main():
    with INPUT.open(encoding="utf-8-sig", newline="") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise ValueError("The CSV contains no trials.")

    run_id = rows[-1]["run_id"]
    rows = [row for row in rows if row["run_id"] == run_id]

    expected = {
        (condition, repetition)
        for condition in CONDITIONS
        for repetition in (1, 2, 3)
    }
    counts = Counter(
        (row["condition"], int(row["repetition"])) for row in rows
    )

    if set(counts) != expected or any(n != 1 for n in counts.values()):
        raise ValueError(
            "Latest run must contain exactly three trials per condition."
        )

    for row in rows:
        if (
            row["outcome"] != "GOAL REACHED"
            or row["box_collision_detected"].strip().lower() != "false"
        ):
            raise ValueError(
                "Inspect failed trials before plotting execution comparisons."
            )

        component_total = sum(
            number(row, field) for field, _, _ in COMPONENTS
        )
        if not math.isclose(
            component_total,
            number(row, "simulation_time_s"),
            abs_tol=1e-6,
        ):
            raise ValueError("Execution components do not sum to total.")

        if not math.isclose(
            number(row, "planned_distance_m"), 0.820, abs_tol=1e-9
        ):
            raise ValueError("Expected the same 0.820 m planned route.")

    groups = {
        condition: [
            row for row in rows if row["condition"] == condition
        ]
        for condition in CONDITIONS
    }

    plt.rcParams.update({
        "font.size": 11,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "pdf.fonttype": 42,
    })

    fig, ax = plt.subplots(figsize=(8, 5))
    positions = (0, 1)
    bottom = [0.0, 0.0]

    for field, label, color in COMPONENTS:
        values = [
            mean(number(row, field) for row in groups[condition])
            for condition in CONDITIONS
        ]
        ax.bar(
            positions,
            values,
            bottom=bottom,
            width=0.55,
            label=label,
            color=color,
        )
        bottom = [a + b for a, b in zip(bottom, values)]

    totals = [
        mean(number(row, "simulation_time_s") for row in groups[condition])
        for condition in CONDITIONS
    ]

    for position, total in zip(positions, totals):
        ax.text(
            position,
            total + 2,
            f"{total:.3f} s",
            ha="center",
        )

    ax.set(
        xticks=positions,
        xticklabels=("Original\n5 segments", "Extra midpoints\n10 segments"),
        ylabel="Mean execution duration (simulation seconds)",
        title="Same planned route, different waypoint counts",
        ylim=(0, max(totals) * 1.16),
    )

    handles, labels = ax.get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        ncol=3,
        frameon=False,
    )

    fig.text(
        0.5,
        0.02,
        "Planned length: 0.820 m in both conditions. "
        "Three repetitions per condition.",
        ha="center",
        fontsize=9,
    )
    fig.tight_layout(rect=(0, 0.07, 1, 0.88))

    OUTPUT.mkdir(exist_ok=True)
    for extension in ("png", "pdf"):
        path = OUTPUT / f"06_waypoint_control.{extension}"
        fig.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")

    plt.close(fig)

    difference = totals[1] - totals[0]
    print(f"Run: {run_id}")
    print(f"Extra execution time: {difference:.3f} s")
    print(f"Increase: {100 * difference / totals[0]:.1f}%")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, KeyError, OSError) as error:
        raise SystemExit(f"Plotting stopped: {error}")