import csv
import statistics
from collections import Counter
from pathlib import Path

from motion import simulate_motion
from planner import find_path
from scenarios import validate_geometry
from test_maps import START, GOAL, RADIUS, generate_obstacles, route_distance

MAP_COUNT = 20
TRIALS_PER_MAP = 20
GRID_SIZE = 20
NOISE_SIGMA = 2.0
EXTRA_CLEARANCES = [0, 4, 8, 12]


def main():
    maps = {
        seed: generate_obstacles(seed)
        for seed in range(MAP_COUNT)
    }

    plans = {}
    common_maps = set(range(MAP_COUNT))

    for extra in EXTRA_CLEARANCES:
        planning_radius = RADIUS + extra
        available = set()

        for map_seed, obstacles in maps.items():
            path = find_path(
                START,
                GOAL,
                obstacles,
                planning_radius,
                GRID_SIZE,
            )

            # Validate against the real robot's body.
            validate_geometry(path, obstacles, RADIUS, GRID_SIZE)
            plans[(extra, map_seed)] = path

            if path:
                available.add(map_seed)

        common_maps &= available

    print(f"Physical robot radius: {RADIUS}px")
    print(f"Movement noise sigma: {NOISE_SIGMA}px per axis per step")
    print(f"Maps: {MAP_COUNT}; motion trials per map: {TRIALS_PER_MAP}")
    print(
        "Maps with routes at every clearance: "
        f"{len(common_maps)}/{MAP_COUNT}"
    )
    print(f"Common map seeds: {sorted(common_maps)}")

    rows = []
    summary_rows = []

    for extra in EXTRA_CLEARANCES:
        outcomes = Counter()
        common_outcomes = Counter()
        successful_distances = []
        maps_with_routes = 0

        for map_seed, obstacles in maps.items():
            path = plans[(extra, map_seed)]

            if path:
                maps_with_routes += 1

            for trial in range(TRIALS_PER_MAP):
                motion_seed = map_seed * 1000 + trial

                result = simulate_motion(
                    path,
                    obstacles,
                    RADIUS,
                    noise_sigma=NOISE_SIGMA,
                    seed=motion_seed,
                    speed=2.0,
                    max_steps=10000,
                )

                outcome = result["outcome"]
                outcomes[outcome] += 1

                if map_seed in common_maps:
                    common_outcomes[outcome] += 1

                if outcome == "success":
                    successful_distances.append(result["distance_px"])

                rows.append({
                    "extra_clearance_px": extra,
                    "planning_radius_px": RADIUS + extra,
                    "physical_radius_px": RADIUS,
                    "map_seed": map_seed,
                    "trial": trial,
                    "motion_seed": motion_seed,
                    "noise_sigma_px": NOISE_SIGMA,
                    "route_available": bool(path),
                    "common_map": map_seed in common_maps,
                    "outcome": outcome,
                    "planned_distance_px": (
                        route_distance(path) if path else ""
                    ),
                    "travelled_distance_px": result["distance_px"],
                    "steps": result["steps"],
                })

        total = MAP_COUNT * TRIALS_PER_MAP
        common_total = len(common_maps) * TRIALS_PER_MAP
        success_rate = 100 * outcomes["success"] / total

        common_rate = (
            100 * common_outcomes["success"] / common_total
            if common_total else None
        )

        mean_common_plan_distance = (
            statistics.mean(
                route_distance(plans[(extra, seed)])
                for seed in common_maps
            )
            if common_maps else None
        )

        print(f"\nExtra planning clearance: {extra}px")
        print(f"Maps with a route: {maps_with_routes}/{MAP_COUNT}")
        print(
            f"Success across all cases: {outcomes['success']}/{total} "
            f"({success_rate:.1f}%)"
        )
        print(
            f"Collisions: {outcomes['collision']} | "
            f"Timeouts: {outcomes['timeout']} | "
            f"No route: {outcomes['no_route']}"
        )

        if common_total:
            print(
                f"Success on common maps: "
                f"{common_outcomes['success']}/{common_total} "
                f"({common_rate:.1f}%)"
            )
            print(
                "Mean planned distance on common maps: "
                f"{mean_common_plan_distance:.1f}px"
            )

        summary_rows.append({
            "extra_clearance_px": extra,
            "maps_with_routes": maps_with_routes,
            "total_trials": total,
            "successes": outcomes["success"],
            "collisions": outcomes["collision"],
            "timeouts": outcomes["timeout"],
            "no_route": outcomes["no_route"],
            "success_rate_all_percent": success_rate,
            "common_maps": len(common_maps),
            "success_rate_common_percent": (
                common_rate if common_rate is not None else ""
            ),
            "mean_common_planned_distance_px": (
                mean_common_plan_distance
                if mean_common_plan_distance is not None else ""
            ),
            "mean_success_travelled_distance_px": (
                statistics.mean(successful_distances)
                if successful_distances else ""
            ),
        })

    folder = Path(__file__).resolve().parent / "results"
    folder.mkdir(exist_ok=True)

    for filename, data in [
        ("clearance_trials.csv", rows),
        ("clearance_summary.csv", summary_rows),
    ]:
        with (folder / filename).open(
            "w", newline="", encoding="utf-8"
        ) as file:
            writer = csv.DictWriter(file, fieldnames=list(data[0]))
            writer.writeheader()
            writer.writerows(data)

    print(f"\nRecorded motion cases: {len(rows)}")
    print(f"Results saved in: {folder}")


if __name__ == "__main__":
    main()