import math
from controller import Supervisor

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
robot_node = robot.getSelf()
box_node = robot.getFromDef("ROBOBENCH_OBSTACLE")

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

if robot_node is None or box_node is None:
    raise RuntimeError(
        "Enable supervisor TRUE and set the box DEF "
        "to ROBOBENCH_OBSTACLE."
    )

robot_node.enableContactPointsTracking(timestep, True)
box_node.enableContactPointsTracking(timestep, False)

SPEED = 0.5
MAX_SECONDS = 10.0

start_time = robot.getTime()
last_print = -1.0
finished = False
start_checked = False

print("Contact-detector validation started.")


def shared_contact_count(robot_contacts, box_contacts):
    count = 0

    for robot_contact in robot_contacts:
        for box_contact in box_contacts:
            if math.dist(
                robot_contact.getPoint(),
                box_contact.getPoint(),
            ) <= 0.00001:
                count += 1
                break

    return count


while robot.step(timestep) != -1:
    now = robot.getTime()
    elapsed = now - start_time

    robot_contacts = robot_node.getContactPoints(True)
    box_contacts = box_node.getContactPoints(False)

    shared = shared_contact_count(robot_contacts, box_contacts)
    speed = 0.0

    if not finished:
        if shared > 0:
            finished = True
            print(
                "PASS: Physical box contact detected. "
                f"Shared contact points: {shared}. Wheels stopped."
            )

        elif elapsed >= MAX_SECONDS:
            finished = True
            print(
                "FAIL: No shared box contact detected before timeout. "
                "Wheels stopped."
            )

        elif elapsed >= 0.5:
            if not start_checked:
                start_checked = True
                position = robot_node.getPosition()
                orientation = robot_node.getOrientation()
                heading = math.atan2(
                    orientation[3], orientation[0]
                )

                if (
                    math.hypot(position[0] + 0.16, position[1]) > 0.01
                    or abs(heading) > 0.1
                ):
                    finished = True
                    print(
                        "SETUP ERROR: Use translation -0.16 0 0 "
                        "and rotation 0 0 1 0."
                    )

            if not finished:
                speed = SPEED

    left_motor.setVelocity(speed)
    right_motor.setVelocity(speed)

    if not finished and now - last_print >= 0.5:
        print(
            f"Robot contacts: {len(robot_contacts)} | "
            f"Box contacts: {len(box_contacts)} | "
            f"Shared: {shared}"
        )
        last_print = now