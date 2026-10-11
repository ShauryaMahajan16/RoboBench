from motion import movement_is_safe, simulate_motion
from planner import find_path

RADIUS = 12
OBSTACLES = [
    (200, 100, 260, 280),
    (360, 220, 440, 300),
]


def main():
    path = find_path(
        (60, 200), (540, 200), OBSTACLES, RADIUS, 20
    )
    assert path, "Expected a planned route."

    result = simulate_motion(path, OBSTACLES, RADIUS)
    assert result["outcome"] == "success"
    print("PASS: Zero-noise robot reaches the goal")

    assert not movement_is_safe(
        (100, 200), (300, 200), OBSTACLES, RADIUS
    )
    print("PASS: Detects obstacle crossing between endpoints")

    assert not movement_is_safe(
        (60, 200), (5, 200), [], RADIUS
    )
    print("PASS: Detects map-boundary collision")

    assert not movement_is_safe(
        (188, 200), (188, 200), OBSTACLES, RADIUS
    )
    print("PASS: Touching an obstacle counts as collision")

    assert movement_is_safe(
        (60, 200), (80, 200), OBSTACLES, RADIUS
    )
    print("PASS: Accepts clear movement")

    first = simulate_motion(
        path, OBSTACLES, RADIUS, noise_sigma=0.5, seed=42
    )
    second = simulate_motion(
        path, OBSTACLES, RADIUS, noise_sigma=0.5, seed=42
    )
    assert first == second
    print("PASS: Same seed reproduces the same result")

    assert simulate_motion(
        [], OBSTACLES, RADIUS
    )["outcome"] == "no_route"
    print("PASS: Handles unavailable route")

    assert simulate_motion(
        path, OBSTACLES, RADIUS, max_steps=1
    )["outcome"] == "timeout"
    print("PASS: Handles step limit")

    print("\nAll eight motion checks passed.")


if __name__ == "__main__":
    main()