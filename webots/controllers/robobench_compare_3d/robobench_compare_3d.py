import csv
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from controller import Supervisor

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

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

project_folder = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(project_folder))

from planner import find_path
from bfs import find_path_bfs
from world_geometry import box_to_grid_rectangle

START = (-0.25, 0.0)
GOAL = (0.25, 0.0)
GRID_SIZE = 20
PLANNING_RADIUS = 40
METRES_PER_PIXEL = 0.001

WAYPOINT_TOLERANCE = 0.008
FORWARD_SPEED = 1.5
MAX_WHEEL_SPEED = 2.0
TURN_GAIN = 1.5
MAX_TRIAL_SECONDS = 180.0

ALGORITHMS = ["A-star", "Dijkstra", "BFS"]
RUN_ID = datetime.now(timezone.utc).isoformat()

front_sensors = []

for name in ["ps0", "ps1", "ps6", "ps7"]:
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
        raise SystemExit


def to_pixels(point):
    return (
        round(point[0] / METRES_PER_PIXEL + 300),
        round(point[1] / METRES_PER_PIXEL + 200),
    )


def to_metres(point):
    return (
        (point[0] - 300) * METRES_PER_PIXEL,
        (point[1] - 200) * METRES_PER_PIXEL,
    )


def simplify_path(path):
    if len(path) <= 2:
        return path[:]

    simplified = [path[0]]

    for index in range(1, len(path) - 1):
        previous, current, following = path[index - 1:index + 2]

        incoming = (
            current[0] - previous[0],
            current[1] - previous[1],
        )
        outgoing = (
            following[0] - current[0],
            following[1] - current[1],
        )

        if incoming != outgoing:
            simplified.append(current)

    simplified.append(path[-1])
    return simplified


def read_box():
    position = list(box_node.getPosition())
    size = list(box_node.getField("size").getSFVec3f())
    orientation = list(box_node.getOrientation())

    rectangle = box_to_grid_rectangle(position, size, orientation)

    return position, size, orientation, rectangle


def reset_robot():
    stop()
    robot_node.getField("translation").setSFVec3f(
        [START[0], START[1], 0.0]
    )
    robot_node.getField("rotation").setSFRotation([0, 0, 1, 0])
    robot_node.resetPhysics()

    started = robot.getTime()

    while robot.getTime() - started < 0.5:
        stop()
        step()

    position = robot_node.getPosition()
    orientation = robot_node.getOrientation()
    heading = math.atan2(orientation[3], orientation[0])

    if (
        math.hypot(position[0] - START[0], position[1] - START[1])
        > 0.01
        or abs(heading) > 0.1
    ):
        raise RuntimeError(
            "Robot reset failed. Check that the start is unobstructed."
        )


def touching_box():
    return any(
        math.dist(a.getPoint(), b.getPoint()) <= 0.00001
        for a in robot_node.getContactPoints(True)
        for b in box_node.getContactPoints(False)
    )


def draw_route(route):
    for name in ["ROBOBENCH_ROUTE", "ROBOBENCH_GOAL"]:
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

    if not route:
        return

    coordinates = " ".join(
        f"{x:.6f} {y:.6f} 0.004" for x, y in route
    )
    indices = " ".join(str(index) for index in range(len(route)))

    root_children.importMFNodeFromString(
        -1,
        "DEF ROBOBENCH_ROUTE Shape { "
        "appearance Appearance { material Material { "
        "diffuseColor 1 0.5 0 emissiveColor 1 0.5 0 "
        "} } geometry IndexedLineSet { "
        "coord Coordinate { point [ " + coordinates + " ] } "
        "coordIndex [ " + indices + " -1 ] } }",
    )


def execute_route(route):
    started = robot.getTime()
    previous_time = started
    previous_position = robot_node.getPosition()[:2]

    travelled = 0.0
    waypoint = 1
    obstacle_count = 0
    last_print = -1.0

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
            and math.dist(current, route[waypoint])
            <= WAYPOINT_TOLERANCE
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

        orientation = robot_node.getOrientation()
        heading = math.atan2(orientation[3], orientation[0])

        target_x, target_y = route[waypoint]
        dx = target_x - current[0]
        dy = target_y - current[1]
        distance = math.hypot(dx, dy)

        target_heading = math.atan2(dy, dx)
        error = math.atan2(
            math.sin(target_heading - heading),
            math.cos(target_heading - heading),
        )

        turn = max(
            -MAX_WHEEL_SPEED,
            min(MAX_WHEEL_SPEED, TURN_GAIN * error),
        )

        if abs(error) > 0.15:
            command = "TURNING"
            left_speed, right_speed = -turn, turn

            if previous_command != command:
                turn_episodes += 1
        else:
            command = "MOVING"
            forward = min(FORWARD_SPEED, distance * 10.0)

            left_speed = max(
                -MAX_WHEEL_SPEED,
                min(MAX_WHEEL_SPEED, forward - turn),
            )
            right_speed = max(
                -MAX_WHEEL_SPEED,
                min(MAX_WHEEL_SPEED, forward + turn),
            )

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        previous_command = command

        if now - last_print >= 5.0:
            print(
                f"{command} | Waypoint {waypoint}/{len(route) - 1} | "
                f"x={current[0]:.3f}, y={current[1]:.3f}"
            )
            last_print = now


def append_result(row):
    folder = project_folder / "results"
    folder.mkdir(exist_ok=True)
    filename = folder / "webots_geometry_comparison.csv"

    needs_header = (
        not filename.exists() or filename.stat().st_size == 0
    )

    with filename.open("a", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(row))
        if needs_header:
            writer.writeheader()
        writer.writerow(row)

    return filename


stop()

# This experiment assumes the box remains static.
if box_node.getField("mass").getSFFloat() > 0:
    raise RuntimeError("Set WoodenBox mass to 0 for this static-map test.")

results = []

for algorithm in ALGORITHMS:
    print(f"\n--- Starting {algorithm} trial ---")

    box_position, box_size, box_orientation, rectangle = read_box()
    print(f"Box footprint in planner pixels: {rectangle}")

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

    route = [to_metres(point) for point in simplify_path(path)]
    planned_distance = (
        sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        * METRES_PER_PIXEL
        if path else None
    )

    draw_route(route)

    if path:
        reset_robot()
        result = execute_route(route)
    else:
        stop()
        result = {
            "outcome": "NO ROUTE",
            "box_collision_detected": False,
            "travelled_distance_m": 0.0,
            "simulation_time_s": 0.0,
            "turn_command_time_s": 0.0,
            "move_command_time_s": 0.0,
            "idle_command_time_s": 0.0,
            "turn_episodes": 0,
            "goal_error_m": "",
        }

    row = {
        "run_id": RUN_ID,
        "algorithm": algorithm,
        "box_position_json": json.dumps(box_position),
        "box_size_json": json.dumps(box_size),
        "box_orientation_json": json.dumps(box_orientation),
        "grid_rectangle_json": json.dumps(rectangle),
        "waypoints_json": json.dumps(route),
        "planning_time_ms": planning_ms,
        "nodes_expanded": stats["nodes_expanded"],
        "planned_distance_m": (
            planned_distance if planned_distance is not None else ""
        ),
        "route_segments": max(0, len(route) - 1),
        **result,
        "grid_size_px": GRID_SIZE,
        "planning_radius_px": PLANNING_RADIUS,
        "waypoint_tolerance_m": WAYPOINT_TOLERANCE,
    }

    filename = append_result(row)
    results.append(row)

    print(f"RESULT: {algorithm} | {result['outcome']}")

print("\n--- WORLD-GEOMETRY COMPARISON COMPLETE ---")

for row in results:
    planned_text = (
        f"{row['planned_distance_m']:.3f} m"
        if row["planned_distance_m"] != "" else "N/A"
    )

    print(
        f"{row['algorithm']:8} | {row['outcome']} | "
        f"Planned: {planned_text} | "
        f"Travelled: {row['travelled_distance_m']:.3f} m | "
        f"Total: {row['simulation_time_s']:.2f} s | "
        f"Collision: {row['box_collision_detected']}"
    )

print(f"Results appended to: {filename}")
stop()

while robot.step(timestep) != -1:
    pass