import csv
import statistics
from pathlib import Path


def main():
    source = (
        Path(__file__).resolve().parent
        / "results"
        / "resolution_summary.csv"
    )

    if not source.exists():
        raise SystemExit("Run resolution_experiment.py first.")

    maps = {}

    # Use Dijkstra's results once per map and grid.
    # The experiment already checked agreement with A-star and BFS.
    with source.open(newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            if row["algorithm"] != "Dijkstra":
                continue

            seed = int(row["seed"])
            grid = int(row["grid_size_px"])

            entry = maps.setdefault(seed, {})

            if grid in entry:
                raise ValueError(
                    f"Duplicate result for seed {seed}, grid {grid}."
                )

            found = row["route_found"].lower() == "true"

            entry[grid] = {
                "found": found,
                "distance": (
                    float(row["distance_px"]) if found else None
                ),
            }

    if not maps:
        raise ValueError("No Dijkstra results found.")

    for seed, results in maps.items():
        if set(results) != {10, 20, 40}:
            raise ValueError(f"Incomplete grid results for seed {seed}.")

    missed = [
        seed
        for seed, results in sorted(maps.items())
        if results[20]["found"] and not results[40]["found"]
    ]

    print("Routes found at 20 pixels but missed at 40 pixels:")
    print(", ".join(str(seed) for seed in missed) or "None")
    print(f"Number of affected maps: {len(missed)}")

    for fine, coarse in [(10, 20), (20, 40)]:
        common = [
            (seed, results)
            for seed, results in sorted(maps.items())
            if results[fine]["found"] and results[coarse]["found"]
        ]

        print(f"\nDistance comparison: {fine}px versus {coarse}px")
        print(f"Maps with routes at both resolutions: {len(common)}")

        if not common:
            continue

        reductions = []
        improved_seeds = []

        for seed, results in common:
            fine_distance = results[fine]["distance"]
            coarse_distance = results[coarse]["distance"]
            reduction = coarse_distance - fine_distance

            if reduction < -0.000001:
                raise ValueError(
                    f"Unexpected longer fine-grid route on seed {seed}."
                )

            reductions.append(max(0, reduction))

            if reduction > 0.000001:
                improved_seeds.append(seed)

        print(f"Finer grid found a shorter route: {len(improved_seeds)}")
        print(f"Same distance: {len(common) - len(improved_seeds)}")
        print(
            "Mean distance reduction across these paired maps: "
            f"{statistics.mean(reductions):.1f} pixels"
        )
        print(
            "Largest distance reduction: "
            f"{max(reductions):.1f} pixels"
        )
        print(
            "Seeds with shorter fine-grid routes: "
            + (
                ", ".join(str(seed) for seed in improved_seeds)
                or "None"
            )
        )


if __name__ == "__main__":
    main()