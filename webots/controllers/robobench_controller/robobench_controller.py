from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

sensors = []

for index in range(8):
    sensor = robot.getDevice(f"ps{index}")
    sensor.enable(timestep)
    sensors.append(sensor)

FORWARD_SPEED = 1.0
TURN_SPEED = 1.0

DETECT_THRESHOLD = 120.0
CLEAR_THRESHOLD = 90.0

MIN_TURN_SECONDS = 2.0
MAX_TURN_SECONDS = 5.0
MAX_TEST_SECONDS = 20.0

state = "FORWARD"
start_time = robot.getTime()
turn_start = 0.0
turn_direction = "LEFT"

detection_count = 0
clear_count = 0
last_print = -1.0

print("Obstacle avoidance test started.")

while robot.step(timestep) != -1:
    now = robot.getTime()
    elapsed = now - start_time
    values = [sensor.getValue() for sensor in sensors]

    # Front and front-diagonal sensors.
    right_front = max(values[0], values[1])
    left_front = max(values[6], values[7])
    front = max(right_front, left_front)

    left_speed = 0.0
    right_speed = 0.0

    if elapsed < 0.25:
        pass

    elif elapsed >= MAX_TEST_SECONDS:
        if state != "STOPPED":
            print("Test time limit reached. Stopping.")
        state = "STOPPED"

    elif state == "FORWARD":
        if front >= DETECT_THRESHOLD:
            detection_count += 1
        else:
            detection_count = 0

        if detection_count >= 2:
            state = "TURNING"
            turn_start = now
            clear_count = 0

            # Turn away from the side with the stronger reading.
            turn_direction = (
                "RIGHT" if left_front > right_front else "LEFT"
            )

            print(f"Obstacle detected. Turning {turn_direction}.")
        else:
            left_speed = FORWARD_SPEED
            right_speed = FORWARD_SPEED

    elif state == "TURNING":
        turn_elapsed = now - turn_start

        if front < CLEAR_THRESHOLD:
            clear_count += 1
        else:
            clear_count = 0

        if (
            turn_elapsed >= MIN_TURN_SECONDS
            and clear_count >= 3
        ):
            state = "FORWARD"
            detection_count = 0
            print("Front clear. Resuming forward movement.")

        elif turn_elapsed >= MAX_TURN_SECONDS:
            state = "STOPPED"
            print("Front did not clear in time. Stopping.")

        elif turn_direction == "LEFT":
            left_speed = -TURN_SPEED
            right_speed = TURN_SPEED

        else:
            left_speed = TURN_SPEED
            right_speed = -TURN_SPEED

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    if now - last_print >= 0.5:
        print(
            f"State: {state} | "
            f"Front right: {right_front:.1f} | "
            f"Front left: {left_front:.1f}"
        )
        last_print = now