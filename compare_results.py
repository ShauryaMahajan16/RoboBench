import csv
import statistics
from pathlib import Path

ALGORITHMS = ["A-star", "Dijkstra", "BFS"]


def main():
    folder = Path(__file__).resolve().parent / "results"
    source = folder / "map_summary.csv"

    if not source.exists():
        raise SystemExit(
            "Run benchmark_maps.py first to create map_summary.csv."
        )

    maps = {}

    with source.open(newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            seed = int(row["seed"])
            algorithm = row["algorithm"]

            if algorithm not in ALGORITHMS:
                raise ValueError(f"Unknown algorithm: {algorithm}")

            entry = maps.setdefault(seed, {})

            if algorithm in entry:
                raise ValueError(
                    f"Duplicate result: seed {seed}, {algorithm}"
                )

            entry[algorithm] = {
                "reachable": row["route_found"].lower() == "true",
                "time_ms": float(row["median_planning_ms"]),
            }

    comparison_rows = []

    for seed, results in sorted(maps.items()):
        if set(results) != set(ALGORITHMS):
            raise ValueError(f"Seed {seed} is missing an algorithm.")

        reachable_values = {
            result["reachable"] for result in results.values()
        }

        if len(reachable_values) != 1:
            raise ValueError(f"Reachability mismatch on seed {seed}.")

        astar = results["A-star"]["time_ms"]
        dijkstra = results["Dijkstra"]["time_ms"]
        bfs = results["BFS"]["time_ms"]

        if min(astar, dijkstra, bfs) <= 0:
            raise ValueError(f"Invalid timing on seed {seed}.")

        comparison_rows.append({
            "seed": seed,
            "route_found": results["A-star"]["reachable"],
            "astar_ms": astar,
            "dijkstra_ms": dijkstra,
            "bfs_ms": bfs,
            # Above 1 means BFS had the lower median time.
            "astar_time_divided_by_bfs": astar / bfs,
        })

    for reachable, title in [
        (True, "Maps with a route"),
        (False, "Maps with no route"),
    ]:
        selected = [
            row for row in comparison_rows
            if row["route_found"] == reachable
        ]

        if not selected:
            continue

        print(f"\n{title}: {len(selected)}")

        astar_faster = sum(
            row["astar_ms"] < row["bfs_ms"] for row in selected
        )
        bfs_faster = sum(
            row["bfs_ms"] < row["astar_ms"] for row in selected
        )
        equal = len(selected) - astar_faster - bfs_faster

        ratios = [
            row["astar_time_divided_by_bfs"] for row in selected
        ]

        print(f"A-star lower median than BFS: {astar_faster} maps")
        print(f"BFS lower median than A-star: {bfs_faster} maps")
        print(f"Exactly equal medians: {equal} maps")
        print(
            "Median per-map A-star/BFS time ratio: "
            f"{statistics.median(ratios):.3f}"
        )

        for algorithm, key in [
            ("A-star", "astar_ms"),
            ("Dijkstra", "dijkstra_ms"),
            ("BFS", "bfs_ms"),
        ]:
            wins = sum(
                row[key] < min(
                    row[other_key]
                    for other_key in ["astar_ms", "dijkstra_ms", "bfs_ms"]
                    if other_key != key
                )
                for row in selected
            )

            print(f"{algorithm} strictly fastest: {wins} maps")

    destination = folder / "paired_comparison.csv"

    with destination.open(
        "w", newline="", encoding="utf-8"
    ) as file:
        writer = csv.DictWriter(
            file, fieldnames=list(comparison_rows[0])
        )
        writer.writeheader()
        writer.writerows(comparison_rows)

    print("\nRatios above 1 favour BFS; below 1 favour A-star.")
    print("Small timing differences may reflect measurement noise.")
    print(f"Saved to: {destination}")


if __name__ == "__main__":
    main()