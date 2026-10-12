import csv
import math
import sys
from pathlib import Path
from controller import Supervisor

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
robot_node = robot.getSelf()

if robot_node is None:
    raise RuntimeError("Set E-puck's supervisor field to TRUE.")

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

obstacle_node = robot.getFromDef("ROBOBENCH_OBSTACLE")

if obstacle_node is None:
    raise RuntimeError("Set WoodenBox's DEF to ROBOBENCH_OBSTACLE.")

# The planner below describes this particular box layout.
box_position = obstacle_node.getField("translation").getSFVec3f()
box_size = obstacle_node.getField("size").getSFVec3f()
box_rotation = obstacle_node.getField("rotation").getSFRotation()

if (
    any(abs(a - b) > 0.00001 for a, b in zip(
        box_position, [0.0, 0.0, 0.1]
    ))
    or any(abs(a - b) > 0.00001 for a, b in zip(
        box_size, [0.2, 0.2, 0.2]
    ))
    or abs(box_rotation[3]) > 0.00001
):
    raise RuntimeError(
        "This test requires an unrotated box at 0 0 0.1 "
        "with size 0.2 0.2 0.2."
    )

# Include the robot's wheels and other descendant bodies.
robot_node.enableContactPointsTracking(timestep, True)
obstacle_node.enableContactPointsTracking(timestep, False)

front_sensors = []

for name in ["ps0", "ps1", "ps6", "ps7"]:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    front_sensors.append(sensor)

project_folder = Path(__file__).resolve().parents[3]

if not (project_folder / "planner.py").exists():
    raise RuntimeError(f"planner.py missing from {project_folder}")

sys.path.insert(0, str(project_folder))
from planner import find_path

METRES_PER_PIXEL = 0.001
WAYPOINT_TOLERANCE = 0.008
FORWARD_SPEED = 1.5
MAX_WHEEL_SPEED = 2.0
TURN_GAIN = 1.5
MAX_TEST_SECONDS = 180.0


def to_pixels(x, y):
    return (
        round(x / METRES_PER_PIXEL + 300),
        round(y / METRES_PER_PIXEL + 200),
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


def touching_box():
    robot_contacts = robot_node.getContactPoints(True)
    box_contacts = obstacle_node.getContactPoints(False)

    # Compare shared physical contact positions. The node IDs
    # identify the queried bodies, not necessarily the other body.
    for robot_contact in robot_contacts:
        a = robot_contact.getPoint()

        for box_contact in box_contacts:
            b = box_contact.getPoint()

            if math.dist(a, b) <= 0.00001:
                return True

    return False


stats = {}

path = find_path(
    to_pixels(-0.25, 0.0),
    to_pixels(0.25, 0.0),
    [(200, 100, 400, 300)],
    radius=40,
    cell_size=20,
    stats=stats,
)

if not path:
    raise RuntimeError("A-star could not find a route.")

route = [to_metres(point) for point in simplify_path(path)]

planned_distance = sum(
    math.dist(a, b) for a, b in zip(path, path[1:])
) * METRES_PER_PIXEL

root_children = robot.getRoot().getField("children")

# Replace visual markers. These have no physical collision bodies.
for name in ["ROBOBENCH_GOAL", "ROBOBENCH_ROUTE"]:
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

coordinates = " ".join(
    f"{x:.6f} {y:.6f} 0.004" for x, y in route
)
indices = " ".join(str(index) for index in range(len(route)))

root_children.importMFNodeFromString(
    -1,
    "DEF ROBOBENCH_ROUTE Shape { "
    "appearance Appearance { material Material { "
    "diffuseColor 1 0.5 0 emissiveColor 1 0.5 0 "
    "} } "
    "geometry IndexedLineSet { "
    "coord Coordinate { point [ " + coordinates + " ] } "
    "coordIndex [ " + indices + " -1 ] "
    "} }",
)

start_time = robot.getTime()
last_print = -1.0
waypoint_index = 1
obstacle_count = 0
finished = False
recorded = False
start_checked = False
collision_detected = False
state = "STARTING"

previous_position = None
travelled_distance = 0.0

print("A-star trial started with box-contact monitoring.")

while robot.step(timestep) != -1:
    now = robot.getTime()
    elapsed = now - start_time

    position = robot_node.getPosition()
    current = (position[0], position[1])

    if previous_position is not None:
        travelled_distance += math.dist(current, previous_position)
    previous_position = current

    orientation = robot_node.getOrientation()
    heading = math.atan2(orientation[3], orientation[0])

    left_speed = 0.0
    right_speed = 0.0

    if not finished:
        if touching_box():
            collision_detected = True
            finished = True
            state = "COLLISION"

        elif elapsed >= 0.25:
            if not start_checked:
                start_checked = True

                if math.dist(current, route[0]) > 0.02:
                    finished = True
                    state = "START POSITION ERROR"

            if not finished:
                while (
                    waypoint_index < len(route)
                    and math.dist(current, route[waypoint_index])
                    <= WAYPOINT_TOLERANCE
                ):
                    print(f"Waypoint {waypoint_index} reached.")
                    waypoint_index += 1

                if waypoint_index >= len(route):
                    finished = True
                    state = "GOAL REACHED"

                elif elapsed >= MAX_TEST_SECONDS:
                    finished = True
                    state = "TIMEOUT"

                else:
                    front = max(
                        sensor.getValue() for sensor in front_sensors
                    )

                    if front >= 120.0:
                        obstacle_count += 1
                    else:
                        obstacle_count = 0

                    if obstacle_count >= 2:
                        finished = True
                        state = "OBSTACLE STOP"

                    else:
                        target_x, target_y = route[waypoint_index]
                        dx = target_x - current[0]
                        dy = target_y - current[1]
                        distance = math.hypot(dx, dy)

                        target_heading = math.atan2(dy, dx)
                        heading_error = math.atan2(
                            math.sin(target_heading - heading),
                            math.cos(target_heading - heading),
                        )

                        turn = max(
                            -MAX_WHEEL_SPEED,
                            min(
                                MAX_WHEEL_SPEED,
                                TURN_GAIN * heading_error,
                            ),
                        )

                        if abs(heading_error) > 0.15:
                            state = "TURNING"
                            left_speed = -turn
                            right_speed = turn
                        else:
                            state = "FOLLOWING A-STAR"
                            forward = min(
                                FORWARD_SPEED, distance * 10.0
                            )

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

    if finished and not recorded:
        goal_error = math.dist(current, route[-1])

        row = {
            "algorithm": "A-star",
            "outcome": state,
            "box_collision_detected": collision_detected,
            "planned_distance_m": planned_distance,
            "travelled_distance_m": travelled_distance,
            "simulation_time_s": elapsed,
            "goal_error_m": goal_error,
            "nodes_expanded": stats["nodes_expanded"],
            "grid_size_px": 20,
            "planning_radius_px": 40,
            "waypoint_tolerance_m": WAYPOINT_TOLERANCE,
        }

        results_folder = project_folder / "results"
        results_folder.mkdir(exist_ok=True)
        filename = results_folder / "webots_trials.csv"
        needs_header = (
            not filename.exists() or filename.stat().st_size == 0
        )

        with filename.open(
            "a", newline="", encoding="utf-8"
        ) as file:
            writer = csv.DictWriter(file, fieldnames=list(row))
            if needs_header:
                writer.writeheader()
            writer.writerow(row)

        print(
            f"RESULT: {state} | "
            f"Box collision detected: {collision_detected} | "
            f"Travelled: {travelled_distance:.3f} m | "
            f"Time: {elapsed:.2f} s | "
            f"Goal error: {goal_error:.3f} m"
        )
        print(f"Trial appended to: {filename}")
        recorded = True

    if not finished and now - last_print >= 1.0:
        print(
            f"{state} | Waypoint: {waypoint_index}/{len(route) - 1} | "
            f"x: {current[0]:.3f} m | y: {current[1]:.3f} m"
        )
        last_print = now