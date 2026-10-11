import math
import random

WIDTH = 600
HEIGHT = 400


def point_segment_distance(point, a, b):
    px, py = point
    ax, ay = a
    bx, by = b

    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy

    if length_squared == 0:
        return math.hypot(px - ax, py - ay)

    fraction = (
        (px - ax) * dx + (py - ay) * dy
    ) / length_squared

    fraction = max(0, min(1, fraction))
    nearest_x = ax + fraction * dx
    nearest_y = ay + fraction * dy

    return math.hypot(px - nearest_x, py - nearest_y)


def segment_rectangle_distance(a, b, rectangle):
    left, top, right, bottom = rectangle

    # Check whether the segment intersects the rectangle.
    low, high = 0.0, 1.0
    intersects = True

    for start, end, minimum, maximum in [
        (a[0], b[0], left, right),
        (a[1], b[1], top, bottom),
    ]:
        change = end - start

        if change == 0:
            if not minimum <= start <= maximum:
                intersects = False
                break
        else:
            first = (minimum - start) / change
            second = (maximum - start) / change
            entry, exit_point = sorted((first, second))

            low = max(low, entry)
            high = min(high, exit_point)

            if low > high:
                intersects = False
                break

    if intersects:
        return 0.0

    def point_rectangle_distance(point):
        x, y = point
        gap_x = max(left - x, x - right, 0)
        gap_y = max(top - y, y - bottom, 0)
        return math.hypot(gap_x, gap_y)

    distances = [
        point_rectangle_distance(a),
        point_rectangle_distance(b),
    ]

    for corner in [
        (left, top), (right, top),
        (left, bottom), (right, bottom),
    ]:
        distances.append(point_segment_distance(corner, a, b))

    return min(distances)


def movement_is_safe(a, b, obstacles, radius):
    for x, y in (a, b):
        if not (
            radius <= x <= WIDTH - radius
            and radius <= y <= HEIGHT - radius
        ):
            return False

    # Check the entire movement, not just its final position.
    for obstacle in obstacles:
        if segment_rectangle_distance(a, b, obstacle) <= radius:
            return False

    return True


def simulate_motion(
    path, obstacles, radius,
    noise_sigma=0.0, seed=0, speed=2.0, max_steps=10000,
):
    if (
        not math.isfinite(noise_sigma)
        or noise_sigma < 0
        or not math.isfinite(speed)
        or speed <= 0
        or not math.isfinite(radius)
        or radius <= 0
        or type(max_steps) is not int
        or max_steps <= 0
    ):
        raise ValueError("Invalid motion settings.")

    def result(outcome, distance, steps, position):
        return {
            "outcome": outcome,
            "distance_px": distance,
            "steps": steps,
            "final_position": position,
        }

    if not path:
        return result("no_route", 0.0, 0, None)

    position = tuple(path[0])
    distance = 0.0
    waypoint = 1
    rng = random.Random(seed)

    if not movement_is_safe(position, position, obstacles, radius):
        return result("collision", distance, 0, position)

    for step_number in range(1, max_steps + 1):
        # Within one pixel counts as reaching a waypoint.
        while waypoint < len(path):
            target = path[waypoint]
            remaining = math.dist(position, target)

            if remaining > 1.0:
                break

            waypoint += 1

        if waypoint >= len(path):
            return result(
                "success", distance, step_number - 1, position
            )

        target_x, target_y = path[waypoint]
        x, y = position
        dx, dy = target_x - x, target_y - y
        remaining = math.hypot(dx, dy)
        commanded_step = min(speed, remaining)

        # Independent Gaussian displacement error on each axis.
        next_position = (
            x + commanded_step * dx / remaining
            + rng.gauss(0, noise_sigma),
            y + commanded_step * dy / remaining
            + rng.gauss(0, noise_sigma),
        )

        distance += math.dist(position, next_position)

        if not movement_is_safe(
            position, next_position, obstacles, radius
        ):
            return result(
                "collision", distance, step_number, next_position
            )

        position = next_position

    return result("timeout", distance, max_steps, position)