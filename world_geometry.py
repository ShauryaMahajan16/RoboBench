import math
from itertools import product


def box_to_grid_rectangle(position, size, orientation):
    """Convert a Webots box's world footprint to planner pixels."""

    for name, values, length in [
        ("position", position, 3),
        ("size", size, 3),
        ("orientation", orientation, 9),
    ]:
        if (
            len(values) != length
            or any(
                type(value) not in (int, float)
                or not math.isfinite(value)
                for value in values
            )
        ):
            raise ValueError(f"Invalid {name}.")

    if any(value <= 0 for value in size):
        raise ValueError("Box dimensions must be positive.")

    corners = []

    # Transform all eight box corners into world coordinates.
    for sx, sy, sz in product([-1, 1], repeat=3):
        local = (
            sx * size[0] / 2,
            sy * size[1] / 2,
            sz * size[2] / 2,
        )

        world_x = position[0] + sum(
            orientation[index] * local[index]
            for index in range(3)
        )
        world_y = position[1] + sum(
            orientation[3 + index] * local[index]
            for index in range(3)
        )

        corners.append((world_x, world_y))

    # One planner pixel represents one millimetre.
    # Round outward so the rectangle never understates the footprint.
    left = math.floor(min(x for x, _ in corners) * 1000 + 300)
    right = math.ceil(max(x for x, _ in corners) * 1000 + 300)
    top = math.floor(min(y for _, y in corners) * 1000 + 200)
    bottom = math.ceil(max(y for _, y in corners) * 1000 + 200)

    if not (0 <= left < right <= 600 and 0 <= top < bottom <= 400):
        raise ValueError("Box footprint extends outside the planner map.")

    return left, top, right, bottom