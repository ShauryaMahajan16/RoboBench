import csv
import statistics
from collections import Counter
from pathlib import Path

from motion import simulate_motion
from planner import find_path
from test_maps import START, GOAL, RADIUS, generate_obstacles

MAP_COUNT = 20
TRIALS_PER_MAP = 20
NOISE_LEVELS = [0.0, 0.25, 0.5, 1.0, 2.0]


def main():
    planned_maps = []
    unavailable = []

    for map_seed in range(MAP_COUNT):
        obstacles = generate_obstacles(map_seed)

        path = find_path(
            START, GOAL, obstacles, RADIUS, 20
        )

        if path:
            planned_maps.append((map_seed, obstacles, path))
        else:
            unavailable.append(map_seed)

    if not planned_maps:
        raise SystemExit("No maps have a planned route.")

    print(f"Maps with planned routes: {len(planned_maps)}/{MAP_COUNT}")
    print(f"Excluded no-route map seeds: {unavailable}")
    print("Each noise level uses the same maps and trial seeds.")

    rows = []
    summary_rows = []

    for sigma in NOISE_LEVELS:
        outcomes = Counter()
        successful_distances = []

        for map_seed, obstacles, path in planned_maps:
            for trial in range(TRIALS_PER_MAP):
                motion_seed = map_seed * 1000 + trial

                result = simulate_motion(
                    path,
                    obstacles,
                    RADIUS,
                    noise_sigma=sigma,
                    seed=motion_seed,
                    speed=2.0,
                    max_steps=10000,
                )

                outcome = result["outcome"]
                outcomes[outcome] += 1

                if sigma == 0 and outcome != "success":
                    raise RuntimeError(
                        f"Zero-noise failure on map {map_seed}."
                    )

                if outcome == "success":
                    successful_distances.append(result["distance_px"])

                rows.append({
                    "map_seed": map_seed,
                    "trial": trial,
                    "motion_seed": motion_seed,
                    "noise_sigma_px": sigma,
                    "outcome": outcome,
                    "distance_px": result["distance_px"],
                    "steps": result["steps"],
                })

        total = len(planned_maps) * TRIALS_PER_MAP
        success_rate = 100 * outcomes["success"] / total

        mean_distance = (
            statistics.mean(successful_distances)
            if successful_distances else None
        )

        print(f"\nMovement noise sigma: {sigma:.2f} px per axis per step")
        print(
            f"Success: {outcomes['success']}/{total} "
            f"({success_rate:.1f}%)"
        )
        print(f"Collisions: {outcomes['collision']}")
        print(f"Timeouts: {outcomes['timeout']}")

        if mean_distance is not None:
            print(
                "Mean travelled distance among successes: "
                f"{mean_distance:.1f} px"
            )

        summary_rows.append({
            "noise_sigma_px": sigma,
            "maps_with_routes": len(planned_maps),
            "trials": total,
            "successes": outcomes["success"],
            "collisions": outcomes["collision"],
            "timeouts": outcomes["timeout"],
            "success_rate_percent": success_rate,
            "mean_success_distance_px": (
                mean_distance if mean_distance is not None else ""
            ),
        })

    folder = Path(__file__).resolve().parent / "results"
    folder.mkdir(exist_ok=True)

    for filename, data in [
        ("motion_trials.csv", rows),
        ("motion_summary.csv", summary_rows),
    ]:
        with (folder / filename).open(
            "w", newline="", encoding="utf-8"
        ) as file:
            writer = csv.DictWriter(file, fieldnames=list(data[0]))
            writer.writeheader()
            writer.writerows(data)

    print(f"\nRecorded motion trials: {len(rows)}")
    print(f"Results saved in: {folder}")


if __name__ == "__main__":
    main()