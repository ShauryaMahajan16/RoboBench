import csv
import json
import math
import sys
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

project_folder = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(project_folder))
from planner import find_path

START = (-0.25, 0.0)
GOAL = (0.25, 0.0)
REPEATS = 3

WAYPOINT_TOLERANCE = 0.008
FORWARD_SPEED = 1.5
MAX_WHEEL_SPEED = 2.0
TURN_GAIN = 1.5
MAX_TRIAL_SECONDS = 180.0

RUN_ID = datetime.now(timezone.utc).isoformat()

sensors = []
for name in ["ps0", "ps1", "ps6", "ps7"]:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    sensors.append(sensor)

robot_node.enableContactPointsTracking(timestep, True)
box_node.enableContactPointsTracking(timestep, False)


def stop():
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


def step():
    if robot.step(timestep) == -1:
        stop()
        raise SystemExit


def reset_robot():
    stop()
    robot_node.getField("translation").setSFVec3f(
        [START[0], START[1], 0.0]
    )
    robot_node.getField("rotation").setSFRotation([0, 0, 1, 0])
    robot_node.resetPhysics()

    start = robot.getTime()
    while robot.getTime() - start < 0.5:
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
        raise RuntimeError("Robot reset failed.")


def touching_box():
    return any(
        math.dist(a.getPoint(), b.getPoint()) <= 0.00001
        for a in robot_node.getContactPoints(True)
        for b in box_node.getContactPoints(False)
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


def draw_route(route):
    children = robot.getRoot().getField("children")

    for name in ["ROBOBENCH_ROUTE", "ROBOBENCH_GOAL"]:
        node = robot.getFromDef(name)
        if node is not None:
            node.remove()

    children.importMFNodeFromString(
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

    coordinates = " ".join(
        f"{x:.6f} {y:.6f} 0.004" for x, y in route
    )
    indices = " ".join(str(i) for i in range(len(route)))

    children.importMFNodeFromString(
        -1,
        "DEF ROBOBENCH_ROUTE Shape { "
        "appearance Appearance { material Material { "
        "diffuseColor 1 0.5 0 emissiveColor 1 0.5 0 "
        "} } geometry IndexedLineSet { "
        "coord Coordinate { point [ " + coordinates + " ] } "
        "coordIndex [ " + indices + " -1 ] } }",
    )


def execute(route):
    started = robot.getTime()
    previous_time = started
    previous_position = robot_node.getPosition()[:2]

    travelled = 0.0
    waypoint = 1
    obstacle_count = 0
    previous_command = "IDLE"
    command_times = {"TURNING": 0.0, "MOVING": 0.0, "IDLE": 0.0}
    turn_episodes = 0
    last_print = -1.0

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
            front = max(sensor.getValue() for sensor in sensors)
            obstacle_count = obstacle_count + 1 if front >= 120 else 0

            if obstacle_count >= 2:
                outcome = "OBSTACLE STOP"

        if outcome is not None:
            stop()

            if not math.isclose(
                sum(command_times.values()), elapsed, abs_tol=0.000001
            ):
                raise RuntimeError("Time accounting mismatch.")

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
        dx = route[waypoint][0] - current[0]
        dy = route[waypoint][1] - current[1]
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
            forward = min(FORWARD_SPEED, math.hypot(dx, dy) * 10.0)
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
            print(f"{command} | Waypoint {waypoint}/{len(route) - 1}")
            last_print = now


stop()

box_position = box_node.getField("translation").getSFVec3f()
box_size = box_node.getField("size").getSFVec3f()
box_rotation = box_node.getField("rotation").getSFRotation()

if (
    any(abs(a - b) > 0.00001 for a, b in zip(
        box_position, [0, 0, 0.1]
    ))
    or any(abs(a - b) > 0.00001 for a, b in zip(
        box_size, [0.2, 0.2, 0.2]
    ))
    or abs(box_rotation[3]) > 0.00001
):
    raise RuntimeError("Restore the original unrotated box layout.")

path = find_path(
    (50, 200), (550, 200),
    [(200, 100, 400, 300)],
    radius=40,
    cell_size=20,
)

if not path:
    raise RuntimeError("No reference route found.")

route = [
    ((x - 300) * 0.001, (y - 200) * 0.001)
    for x, y in simplify_path(path)
]

planned_distance = sum(
    math.dist(a, b) for a, b in zip(path, path[1:])
) * 0.001

draw_route(route)
step()

folder = project_folder / "results"
folder.mkdir(exist_ok=True)
filename = folder / "webots_same_route.csv"
results = []

for trial in range(1, REPEATS + 1):
    print(f"\n--- Same-route trial {trial}/{REPEATS} ---")
    reset_robot()
    result = execute(route)

    row = {
        "run_id": RUN_ID,
        "trial": trial,
        "reference_planner": "A-star",
        "planned_distance_m": planned_distance,
        "route_segments": len(route) - 1,
        "waypoints_json": json.dumps(route),
        **result,
        "waypoint_tolerance_m": WAYPOINT_TOLERANCE,
    }

    needs_header = (
        not filename.exists() or filename.stat().st_size == 0
    )

    with filename.open("a", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(row))
        if needs_header:
            writer.writeheader()
        writer.writerow(row)

    results.append(row)
    print(f"Trial {trial}: {result['outcome']}")

print("\n--- SAME-ROUTE CONTROL COMPLETE ---")

for row in results:
    print(
        f"Trial {row['trial']} | {row['outcome']} | "
        f"Travelled: {row['travelled_distance_m']:.3f} m | "
        f"Turning: {row['turn_command_time_s']:.2f} s | "
        f"Moving: {row['move_command_time_s']:.2f} s | "
        f"Total: {row['simulation_time_s']:.2f} s | "
        f"Collision: {row['box_collision_detected']}"
    )

print(f"Results appended to: {filename}")
stop()

while robot.step(timestep) != -1:
    pass