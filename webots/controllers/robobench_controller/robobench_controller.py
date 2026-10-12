import math
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

# Positions in metres. The first point is the expected start.
ROUTE = [
    (-0.25, 0.00),
    (-0.25, 0.20),
    (0.25, 0.20),
    (0.25, 0.00),
]

WAYPOINT_TOLERANCE = 0.012
FORWARD_SPEED = 1.5
MAX_WHEEL_SPEED = 2.0
TURN_GAIN = 1.5
MAX_TEST_SECONDS = 120.0

root_children = robot.getRoot().getField("children")

# Create or reposition the visual goal.
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

marker.getField("translation").setSFVec3f([0.25, 0.0, 0.015])

# Draw the intended route as an orange line.
old_route = robot.getFromDef("ROBOBENCH_ROUTE")
if old_route is not None:
    old_route.remove()

root_children.importMFNodeFromString(
    -1,
    """
    DEF ROBOBENCH_ROUTE Shape {
      appearance Appearance {
        material Material {
          diffuseColor 1 0.5 0
          emissiveColor 1 0.5 0
        }
      }
      geometry IndexedLineSet {
        coord Coordinate {
          point [
            -0.25 0    0.004
            -0.25 0.20 0.004
             0.25 0.20 0.004
             0.25 0    0.004
          ]
        }
        coordIndex [0 1 2 3 -1]
      }
    }
    """,
)

start_time = robot.getTime()
last_print = -1.0
waypoint_index = 1
obstacle_count = 0
finished = False
start_checked = False
state = "STARTING"

previous_position = None
travelled_distance = 0.0

print("Waypoint navigation started.")

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

            if math.dist(current, ROUTE[0]) > 0.02:
                finished = True
                state = "START POSITION ERROR"
                print("Set the starting translation to -0.25 0 0.")

        if not finished:
            # Advance only after reaching the current waypoint.
            while (
                waypoint_index < len(ROUTE)
                and math.dist(current, ROUTE[waypoint_index])
                <= WAYPOINT_TOLERANCE
            ):
                print(f"Waypoint {waypoint_index} reached.")
                waypoint_index += 1

            if waypoint_index >= len(ROUTE):
                finished = True
                state = "GOAL REACHED"
                print(
                    "SUCCESS: Route completed. "
                    f"Travelled distance: {travelled_distance:.3f} m"
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
                    target_x, target_y = ROUTE[waypoint_index]
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
                        state = "FOLLOWING ROUTE"
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
            f"Waypoint: {min(waypoint_index, 3)}/3 | "
            f"x: {current[0]:.3f} m | "
            f"y: {current[1]:.3f} m"
        )
        last_print = now