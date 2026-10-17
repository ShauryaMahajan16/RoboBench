import csv
import json
import math
import random
import sys
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

from controller import Supervisor


PROJECT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT))

from planner import find_path
from bfs import find_path_bfs
from world_geometry import box_to_grid_rectangle


START = (-0.25, 0.0)
GOAL = (0.25, 0.0)

ALGORITHMS = ("A-star", "Dijkstra", "BFS")
REPETITIONS = 3
ORDER_SEED = 42

LAYOUTS = [
    ("Centre", [0.0, 0.0, 0.1]),
    ("Right", [0.04, 0.0, 0.1]),
    ("Left", [-0.04, 0.0, 0.1]),
    ("Up", [0.0, 0.03, 0.1]),
    ("Down", [0.0, -0.03, 0.1]),
    ("Right-up", [0.04, 0.03, 0.1]),
    ("Right-down", [0.04, -0.03, 0.1]),
    ("Left-up", [-0.04, 0.03, 0.1]),
    ("Left-down", [-0.04, -0.03, 0.1]),
]

GRID_SIZE = 20
PLANNING_RADIUS = 40
WAYPOINT_TOLERANCE = 0.008
FORWARD_SPEED = 1.5
MAX_WHEEL_SPEED = 2.0
TURN_GAIN = 1.5
TURN_THRESHOLD = 0.15
MAX_TRIAL_SECONDS = 180.0

RUN_ID = datetime.now(timezone.utc).isoformat()
OUTPUT = PROJECT / "results" / "webots_extended_trials.csv"

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
robot_node = robot.getSelf()
box_node = robot.getFromDef("ROBOBENCH_OBSTACLE")

if robot_node is None or box_node is None:
    raise RuntimeError(
        "Enable supervisor TRUE and set the box DEF "
        "to ROBOBENCH_OBSTACLE."
    )

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

for motor in (left_motor, right_motor):
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)

front_sensors = []
for name in ("ps0", "ps1", "ps6", "ps7"):
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    front_sensors.append(sensor)

robot_node.enableContactPointsTracking(timestep, True)
box_node.enableContactPointsTracking(timestep, False)

root_children = robot.getRoot().getField("children")


def stop():
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


def step():
    if robot.step(timestep) == -1:
        stop()
        raise SystemExit("Simulation ended before the experiment finished.")


def heading():
    orientation = robot_node.getOrientation()
    return math.atan2(orientation[3], orientation[0])


def to_pixels(point):
    return (
        round(point[0] * 1000 + 300),
        round(point[1] * 1000 + 200),
    )


def to_metres(point):
    return (
        (point[0] - 300) * 0.001,
        (point[1] - 200) * 0.001,
    )


def simplify_path(path):
    if len(path) <= 2:
        return path[:]

    result = [path[0]]
    for index in range(1, len(path) - 1):
        a, b, c = path[index - 1:index + 2]
        incoming = (b[0] - a[0], b[1] - a[1])
        outgoing = (c[0] - b[0], c[1] - b[1])
        if incoming != outgoing:
            result.append(b)

    result.append(path[-1])
    return result


def touching_box():
    robot_contacts = robot_node.getContactPoints(True)
    box_contacts = box_node.getContactPoints(False)
    return any(
        math.dist(a.getPoint(), b.getPoint()) <= 0.00001
        for a in robot_contacts
        for b in box_contacts
    )


def prepare_trial(box_position):
    stop()

    # Relocate both bodies before stepping, so the moved box
    # does not overlap the robot's previous finishing position.
    robot_node.getField("translation").setSFVec3f(
        [START[0], START[1], 0.0]
    )
    robot_node.getField("rotation").setSFRotation([0, 0, 1, 0])
    robot_node.resetPhysics()

    box_node.getField("translation").setSFVec3f(box_position)
    box_node.getField("rotation").setSFRotation([0, 0, 1, 0])

    started = robot.getTime()
    while robot.getTime() - started < 0.5:
        stop()
        step()

    position = robot_node.getPosition()
    if (
        math.dist(position[:2], START) > 0.01
        or abs(heading()) > 0.1
        or touching_box()
    ):
        raise RuntimeError(
            "Trial setup failed: invalid starting pose or box contact."
        )


def draw_route(route):
    for name in ("ROBOBENCH_ROUTE", "ROBOBENCH_GOAL"):
        node = robot.getFromDef(name)
        if node is not None:
            node.remove()

    root_children.importMFNodeFromString(
        -1,
        """
        DEF ROBOBENCH_GOAL Pose {
          translation 0.25 0 0.015
          children [
            Shape {
              appearance PBRAppearance {
                baseColor 0 1 0
                roughness 1
                metalness 0
                emissiveColor 0 0.3 0
              }
              geometry Sphere { radius 0.015 }
            }
          ]
        }
        """,
    )

    if route:
        coordinates = " ".join(
            f"{x:.6f} {y:.6f} 0.004" for x, y in route
        )
        indices = " ".join(str(i) for i in range(len(route)))

        root_children.importMFNodeFromString(
            -1,
            "DEF ROBOBENCH_ROUTE Shape { "
            "appearance Appearance { material Material { "
            "diffuseColor 1 0.5 0 emissiveColor 1 0.5 0 "
            "} } geometry IndexedLineSet { "
            "coord Coordinate { point [ " + coordinates + " ] } "
            "coordIndex [ " + indices + " -1 ] } }",
        )


def clamp(value):
    return max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, value))


def execute_route(route):
    started = robot.getTime()
    previous_time = started
    previous_position = robot_node.getPosition()[:2]

    travelled = 0.0
    waypoint = 1
    obstacle_count = 0
    last_print = started
    previous_command = "IDLE"
    command_times = {"TURNING": 0.0, "MOVING": 0.0, "IDLE": 0.0}
    turn_episodes = 0

    while True:
        step()
        now = robot.getTime()
        command_times[previous_command] += now - previous_time
        previous_time = now
        elapsed = now - started

        position = robot_node.getPosition()
        current = (position[0], position[1])
        travelled += math.dist(current, previous_position)
        previous_position = current
        collision = touching_box()

        while (
            waypoint < len(route)
            and math.dist(current, route[waypoint]) <= WAYPOINT_TOLERANCE
        ):
            waypoint += 1

        outcome = None
        if collision:
            outcome = "COLLISION"
        elif waypoint >= len(route):
            outcome = "GOAL REACHED"
        elif elapsed >= MAX_TRIAL_SECONDS:
            outcome = "TIMEOUT"

        if outcome is None:
            front = max(sensor.getValue() for sensor in front_sensors)
            obstacle_count = obstacle_count + 1 if front >= 120 else 0
            if obstacle_count >= 2:
                outcome = "OBSTACLE STOP"

        if outcome is not None:
            stop()
            if not math.isclose(
                sum(command_times.values()), elapsed, abs_tol=0.000001
            ):
                raise RuntimeError("Command-time accounting mismatch.")

            return {
                "outcome": outcome,
                "box_collision_detected": collision,
                "travelled_distance_m": travelled,
                "simulation_time_s": elapsed,
                "turn_command_time_s": command_times["TURNING"],
                "move_command_time_s": command_times["MOVING"],
                "idle_command_time_s": command_times["IDLE"],
                "turn_episodes": turn_episodes,
                "goal_error_m": math.dist(current, GOAL),
            }

        dx = route[waypoint][0] - current[0]
        dy = route[waypoint][1] - current[1]
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        error = math.atan2(
            math.sin(target_heading - heading()),
            math.cos(target_heading - heading()),
        )
        turn = clamp(TURN_GAIN * error)

        if abs(error) > TURN_THRESHOLD:
            command = "TURNING"
            left_speed, right_speed = -turn, turn
            if previous_command != command:
                turn_episodes += 1
        else:
            command = "MOVING"
            forward = min(FORWARD_SPEED, distance * 10.0)
            left_speed = clamp(forward - turn)
            right_speed = clamp(forward + turn)

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        previous_command = command

        if now - last_print >= 15.0:
            print(
                f"{command} | Waypoint {waypoint}/{len(route) - 1}"
            )
            last_print = now


def no_route_result():
    return {
        "outcome": "NO ROUTE",
        "box_collision_detected": "",
        "travelled_distance_m": 0.0,
        "simulation_time_s": 0.0,
        "turn_command_time_s": 0.0,
        "move_command_time_s": 0.0,
        "idle_command_time_s": 0.0,
        "turn_episodes": 0,
        "goal_error_m": "",
    }


def save_result(row):
    OUTPUT.parent.mkdir(exist_ok=True)
    needs_header = not OUTPUT.exists() or OUTPUT.stat().st_size == 0

    if not needs_header:
        with OUTPUT.open(newline="", encoding="utf-8") as file:
            existing_fields = next(csv.reader(file))
        if existing_fields != list(row):
            raise RuntimeError(
                "Existing extended CSV has a different schema. "
                "Keep that file and choose a new output filename."
            )

    with OUTPUT.open("a", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(row))
        if needs_header:
            writer.writeheader()
        writer.writerow(row)


def main():
    stop()
    box_size = list(box_node.getField("size").getSFVec3f())
    if any(abs(a - 0.2) > 0.00001 for a in box_size):
        raise RuntimeError("Set box size to 0.2 0.2 0.2.")
    if box_node.getField("mass").getSFFloat() > 0:
        raise RuntimeError("Set box mass to 0.")

    # Shuffle all layout/repetition blocks, then planner order
    # within each block. This is separate from simulation randomness.
    rng = random.Random(ORDER_SEED)
    blocks = [
        (layout, position, repetition)
        for layout, position in LAYOUTS
        for repetition in range(1, REPETITIONS + 1)
    ]
    rng.shuffle(blocks)

    total = len(blocks) * len(ALGORITHMS)
    results = []
    reference_distances = {}

    print(f"Run: {RUN_ID}")
    print(f"Starting {total} trials.")
    print("No added motion noise; repetitions check consistency.")

    for layout, requested_position, repetition in blocks:
        order = list(ALGORITHMS)
        rng.shuffle(order)

        for algorithm in order:
            trial_number = len(results) + 1
            print(
                f"\n--- Trial {trial_number}/{total}: {layout} | "
                f"Repeat {repetition}/{REPETITIONS} | {algorithm} ---"
            )

            prepare_trial(requested_position)
            actual_position = list(box_node.getPosition())
            actual_orientation = list(box_node.getOrientation())
            rectangle = box_to_grid_rectangle(
                actual_position, box_size, actual_orientation
            )

            stats = {}
            planning_started = time.perf_counter_ns()
            if algorithm == "BFS":
                path = find_path_bfs(
                    to_pixels(START), to_pixels(GOAL),
                    [rectangle], PLANNING_RADIUS, GRID_SIZE,
                    stats=stats,
                )
            else:
                path = find_path(
                    to_pixels(START), to_pixels(GOAL),
                    [rectangle], PLANNING_RADIUS, GRID_SIZE,
                    use_heuristic=(algorithm == "A-star"),
                    stats=stats,
                )
            planning_ms = (
                time.perf_counter_ns() - planning_started
            ) / 1_000_000

            distance_px = (
                sum(math.dist(a, b) for a, b in zip(path, path[1:]))
                if path else None
            )

            # Compare the same measured geometry across all planners.
            key = tuple(rectangle)
            if key in reference_distances:
                if distance_px != reference_distances[key]:
                    raise RuntimeError(
                        "Planner reachability or shortest-distance disagreement."
                    )
            else:
                reference_distances[key] = distance_px

            route = [
                to_metres(point) for point in simplify_path(path)
            ]
            draw_route(route)
            result = execute_route(route) if path else no_route_result()

            row = {
                "run_id": RUN_ID,
                "trial_number": trial_number,
                "layout": layout,
                "repetition": repetition,
                "algorithm": algorithm,
                "order_seed": ORDER_SEED,
                "repetitions_per_layout": REPETITIONS,
                "expected_trials": total,
                "requested_box_position_json": json.dumps(requested_position),
                "box_position_json": json.dumps(actual_position),
                "box_size_json": json.dumps(box_size),
                "box_orientation_json": json.dumps(actual_orientation),
                "grid_rectangle_json": json.dumps(rectangle),
                "waypoints_json": json.dumps(route),
                "planning_time_ms": planning_ms,
                "nodes_expanded": stats["nodes_expanded"],
                "planned_distance_m": (
                    distance_px * 0.001 if distance_px is not None else ""
                ),
                "route_segments": max(0, len(route) - 1),
                **result,
                "grid_size_px": GRID_SIZE,
                "planning_radius_px": PLANNING_RADIUS,
                "waypoint_tolerance_m": WAYPOINT_TOLERANCE,
                "basic_time_step_ms": timestep,
                "forward_speed_rad_s": FORWARD_SPEED,
                "max_wheel_speed_rad_s": MAX_WHEEL_SPEED,
                "turn_gain": TURN_GAIN,
                "turn_threshold_rad": TURN_THRESHOLD,
                "max_trial_seconds": MAX_TRIAL_SECONDS,
                "added_motion_noise": False,
            }

            save_result(row)
            results.append(row)
            print(
                f"RESULT: {result['outcome']} | "
                f"Time: {result['simulation_time_s']:.2f} s | "
                f"Box contact: {result['box_collision_detected']}"
            )

    print("\n--- EXTENDED EXPERIMENT COMPLETE ---")
    print(f"Recorded trials: {len(results)}/{total}")
    for algorithm in ALGORITHMS:
        counts = Counter(
            row["outcome"] for row in results
            if row["algorithm"] == algorithm
        )
        print(f"{algorithm}: {dict(counts)}")

    print(f"Run ID: {RUN_ID}")
    print(f"Results appended to: {OUTPUT}")
    stop()

    while robot.step(timestep) != -1:
        pass


try:
    main()
finally:
    stop()