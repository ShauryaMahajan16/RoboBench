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
MAP_CENTRE_X = 300
MAP_CENTRE_Y = 200

WAYPOINT_TOLERANCE = 0.008
FORWARD_SPEED = 1.5
MAX_WHEEL_SPEED = 2.0
TURN_GAIN = 1.5
MAX_TEST_SECONDS = 180.0


def to_pixels(x, y):
    return (
        round(x / METRES_PER_PIXEL + MAP_CENTRE_X),
        round(y / METRES_PER_PIXEL + MAP_CENTRE_Y),
    )


def to_metres(point):
    return (
        (point[0] - MAP_CENTRE_X) * METRES_PER_PIXEL,
        (point[1] - MAP_CENTRE_Y) * METRES_PER_PIXEL,
    )


def simplify_path(path):
    if len(path) <= 2:
        return path[:]

    simplified = [path[0]]

    for index in range(1, len(path) - 1):
        previous = path[index - 1]
        current = path[index]
        following = path[index + 1]

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


start = to_pixels(-0.25, 0.0)
goal = to_pixels(0.25, 0.0)
obstacles = [(200, 100, 400, 300)]

stats = {}

path = find_path(
    start, goal, obstacles,
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

print(f"A-star planned distance: {planned_distance:.3f} m")
print(f"Nodes expanded: {stats['nodes_expanded']}")

for index, point in enumerate(route):
    print(f"Waypoint {index}: {point}")

root_children = robot.getRoot().getField("children")

# Visual goal marker: no physical collision body.
marker = robot.getFromDef("ROBOBENCH_GOAL")

if marker is None:
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

    if robot.step(timestep) == -1:
        raise SystemExit

    marker = robot.getFromDef("ROBOBENCH_GOAL")

if marker is None:
    raise RuntimeError("Could not create the goal marker.")

marker.getField("translation").setSFVec3f(
    [route[-1][0], route[-1][1], 0.015]
)

# Replace the old visual route with A-star's calculated route.
old_route = robot.getFromDef("ROBOBENCH_ROUTE")

if old_route is not None:
    old_route.remove()

coordinates = " ".join(
    f"{x:.6f} {y:.6f} 0.004" for x, y in route
)
indices = " ".join(str(index) for index in range(len(route)))

route_node = (
    "DEF ROBOBENCH_ROUTE Shape { "
    "appearance Appearance { "
    "material Material { "
    "diffuseColor 1 0.5 0 emissiveColor 1 0.5 0 "
    "} } "
    "geometry IndexedLineSet { "
    "coord Coordinate { point [ " + coordinates + " ] } "
    "coordIndex [ " + indices + " -1 ] "
    "} }"
)

root_children.importMFNodeFromString(-1, route_node)

start_time = robot.getTime()
last_print = -1.0
waypoint_index = 1
obstacle_count = 0
finished = False
start_checked = False
state = "STARTING"

previous_position = None
travelled_distance = 0.0

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

    if not finished and elapsed >= 0.25:
        if not start_checked:
            start_checked = True

            if math.dist(current, route[0]) > 0.02:
                finished = True
                state = "START POSITION ERROR"
                print("Set starting translation to -0.25 0 0.")

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

                print(
                    "SUCCESS: A-star route completed. "
                    f"Planned: {planned_distance:.3f} m | "
                    f"Travelled: {travelled_distance:.3f} m | "
                    f"Time: {elapsed:.2f} simulation seconds"
                )

            elif elapsed >= MAX_TEST_SECONDS:
                finished = True
                state = "TIMEOUT"
                print("Time limit reached. Wheels stopped.")

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
                    print("Obstacle detected on route. Wheels stopped.")

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
                        min(MAX_WHEEL_SPEED, TURN_GAIN * heading_error),
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

    if now - last_print >= 1.0:
        print(
            f"{state} | "
            f"Waypoint: {min(waypoint_index, len(route) - 1)}"
            f"/{len(route) - 1} | "
            f"x: {current[0]:.3f} m | y: {current[1]:.3f} m"
        )
        last_print = now