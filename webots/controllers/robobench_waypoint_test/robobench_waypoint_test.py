import csv
import json
import math
import random
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean

from controller import Supervisor


PROJECT = Path(__file__).resolve().parents[3]
OUTPUT = PROJECT / "results" / "webots_waypoint_test.csv"
RUN_ID = datetime.now(timezone.utc).isoformat()

START = (-0.25, 0.0)
GOAL = (0.25, 0.0)

BASE_ROUTE = [
    START,
    (-0.15, 0.0),
    (-0.15, -0.16),
    (0.15, -0.16),
    (0.15, 0.0),
    GOAL,
]

TOLERANCE = 0.008
MAX_SECONDS = 300.0
REPETITIONS = 3

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
robot_node = robot.getSelf()
box_node = robot.getFromDef("ROBOBENCH_OBSTACLE")

if robot_node is None or box_node is None:
    raise RuntimeError(
        "Enable supervisor TRUE and use box DEF ROBOBENCH_OBSTACLE."
    )

left = robot.getDevice("left wheel motor")
right = robot.getDevice("right wheel motor")
for motor in (left, right):
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)

sensors = []
for name in ("ps0", "ps1", "ps6", "ps7"):
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    sensors.append(sensor)

robot_node.enableContactPointsTracking(timestep, True)
box_node.enableContactPointsTracking(timestep, False)


def stop():
    left.setVelocity(0.0)
    right.setVelocity(0.0)


def step():
    if robot.step(timestep) == -1:
        raise SystemExit("Simulation ended before completion.")


def heading():
    rotation = robot_node.getOrientation()
    return math.atan2(rotation[3], rotation[0])


def collision():
    robot_contacts = robot_node.getContactPoints(True)
    box_contacts = box_node.getContactPoints(False)
    return any(
        math.dist(a.getPoint(), b.getPoint()) <= 0.00001
        for a in robot_contacts
        for b in box_contacts
    )


def route_length(route):
    return sum(
        math.dist(a, b) for a, b in zip(route, route[1:])
    )


def add_midpoints(route):
    result = [route[0]]
    for a, b in zip(route, route[1:]):
        result.append(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2))
        result.append(b)
    return result


ROUTES = {
    "Original waypoints": BASE_ROUTE,
    "Extra midpoints": add_midpoints(BASE_ROUTE),
}

if not math.isclose(
    route_length(ROUTES["Original waypoints"]),
    route_length(ROUTES["Extra midpoints"]),
    abs_tol=1e-12,
):
    raise RuntimeError("Route lengths differ.")


def reset_trial():
    stop()
    robot_node.getField("translation").setSFVec3f(
        [START[0], START[1], 0.0]
    )
    robot_node.getField("rotation").setSFRotation([0, 0, 1, 0])
    robot_node.resetPhysics()
    box_node.getField("translation").setSFVec3f([0, 0, 0.1])
    box_node.getField("rotation").setSFRotation([0, 0, 1, 0])

    started = robot.getTime()
    while robot.getTime() - started < 0.5:
        stop()
        step()

    if (
        math.dist(robot_node.getPosition()[:2], START) > 0.01
        or abs(heading()) > 0.1
        or collision()
    ):
        raise RuntimeError("Trial reset failed.")


def draw_route(route):
    for name in ("ROBOBENCH_ROUTE", "ROBOBENCH_GOAL"):
        node = robot.getFromDef(name)
        if node is not None:
            node.remove()

    points = " ".join(
        f"{x:.6f} {y:.6f} 0.004" for x, y in route
    )
    indices = " ".join(str(i) for i in range(len(route)))

    robot.getRoot().getField("children").importMFNodeFromString(
        -1,
        "DEF ROBOBENCH_ROUTE Shape { "
        "appearance Appearance { material Material { "
        "diffuseColor 1 0.5 0 emissiveColor 1 0.5 0 } } "
        "geometry IndexedLineSet { "
        "coord Coordinate { point [ " + points + " ] } "
        "coordIndex [ " + indices + " -1 ] } }",
    )


def clamp(value):
    return max(-2.0, min(2.0, value))


def execute(route):
    started = robot.getTime()
    previous_time = started
    previous_position = robot_node.getPosition()[:2]
    previous_command = "IDLE"
    durations = {"IDLE": 0.0, "MOVING": 0.0, "TURNING": 0.0}

    waypoint = 1
    travelled = 0.0
    obstacle_count = 0
    turn_episodes = 0
    last_print = started

    while True:
        step()
        now = robot.getTime()
        durations[previous_command] += now - previous_time
        previous_time = now

        current = robot_node.getPosition()[:2]
        travelled += math.dist(current, previous_position)
        previous_position = current
        contact = collision()

        while (
            waypoint < len(route)
            and math.dist(current, route[waypoint]) <= TOLERANCE
        ):
            waypoint += 1

        outcome = None
        if contact:
            outcome = "COLLISION"
        elif waypoint >= len(route):
            outcome = "GOAL REACHED"
        elif now - started >= MAX_SECONDS:
            outcome = "TIMEOUT"

        if outcome is None:
            front = max(sensor.getValue() for sensor in sensors)
            obstacle_count = obstacle_count + 1 if front >= 120 else 0
            if obstacle_count >= 2:
                outcome = "OBSTACLE STOP"

        if outcome is not None:
            stop()
            elapsed = now - started
            if not math.isclose(
                sum(durations.values()), elapsed, abs_tol=1e-6
            ):
                raise RuntimeError("Time accounting mismatch.")

            return {
                "outcome": outcome,
                "box_collision_detected": contact,
                "simulation_time_s": elapsed,
                "move_command_time_s": durations["MOVING"],
                "turn_command_time_s": durations["TURNING"],
                "idle_command_time_s": durations["IDLE"],
                "travelled_distance_m": travelled,
                "turn_episodes": turn_episodes,
                "goal_error_m": math.dist(current, GOAL),
            }

        dx = route[waypoint][0] - current[0]
        dy = route[waypoint][1] - current[1]
        desired = math.atan2(dy, dx)
        error = math.atan2(
            math.sin(desired - heading()),
            math.cos(desired - heading()),
        )
        turn = clamp(1.5 * error)

        if abs(error) > 0.15:
            command = "TURNING"
            left_speed, right_speed = -turn, turn
            if previous_command != command:
                turn_episodes += 1
        else:
            command = "MOVING"
            forward = min(1.5, math.hypot(dx, dy) * 10.0)
            left_speed = clamp(forward - turn)
            right_speed = clamp(forward + turn)

        left.setVelocity(left_speed)
        right.setVelocity(right_speed)
        previous_command = command

        if now - last_print >= 20:
            print(f"{command} | Waypoint {waypoint}/{len(route) - 1}")
            last_print = now


def save(row):
    OUTPUT.parent.mkdir(exist_ok=True)
    needs_header = not OUTPUT.exists() or OUTPUT.stat().st_size == 0

    if not needs_header:
        with OUTPUT.open(encoding="utf-8", newline="") as file:
            if next(csv.reader(file)) != list(row):
                raise RuntimeError("Existing CSV has a different schema.")

    with OUTPUT.open("a", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(row))
        if needs_header:
            writer.writeheader()
        writer.writerow(row)


def main():
    size = box_node.getField("size").getSFVec3f()
    if any(abs(value - 0.2) > 0.00001 for value in size):
        raise RuntimeError("Set box size to 0.2 0.2 0.2.")
    if box_node.getField("mass").getSFFloat() > 0:
        raise RuntimeError("Set box mass to 0.")

    rng = random.Random(42)
    results = []

    print("Starting six same-geometry waypoint trials.")
    print("Both planned routes are 0.820 m long.")

    for repetition in range(1, REPETITIONS + 1):
        order = list(ROUTES)
        rng.shuffle(order)

        for condition in order:
            route = ROUTES[condition]
            print(
                f"\n--- Trial {len(results) + 1}/6 | "
                f"Repeat {repetition} | {condition} ---"
            )
            reset_trial()
            draw_route(route)
            result = execute(route)

            row = {
                "run_id": RUN_ID,
                "trial_number": len(results) + 1,
                "repetition": repetition,
                "condition": condition,
                "waypoints_json": json.dumps(route),
                "route_segments": len(route) - 1,
                "planned_distance_m": route_length(route),
                **result,
                "waypoint_tolerance_m": TOLERANCE,
                "basic_time_step_ms": timestep,
                "order_seed": 42,
                "max_trial_seconds": MAX_SECONDS,
                "forward_speed_rad_s": 1.5,
                "max_wheel_speed_rad_s": 2.0,
                "turn_gain": 1.5,
                "turn_threshold_rad": 0.15,
                "added_motion_noise": False,
            }
            save(row)
            results.append(row)
            print(
                f"RESULT: {result['outcome']} | "
                f"Total: {result['simulation_time_s']:.3f} s | "
                f"Moving: {result['move_command_time_s']:.3f} s | "
                f"Turning: {result['turn_command_time_s']:.3f} s"
            )

    print("\n--- WAYPOINT CONTROL COMPLETE ---")
    for condition in ROUTES:
        selected = [r for r in results if r["condition"] == condition]
        successes = [
            r for r in selected
            if r["outcome"] == "GOAL REACHED"
            and not r["box_collision_detected"]
        ]
        print(f"{condition} | Collision-free goals: {len(successes)}/3")

        if len(successes) == REPETITIONS:
            print(
                f"  Mean total: "
                f"{mean(r['simulation_time_s'] for r in successes):.3f} s"
                f" | Moving: "
                f"{mean(r['move_command_time_s'] for r in successes):.3f} s"
                f" | Turning: "
                f"{mean(r['turn_command_time_s'] for r in successes):.3f} s"
            )
        else:
            print("  Inspect individual outcomes before comparing times.")

    print(f"Run ID: {RUN_ID}")
    print(f"Results appended to: {OUTPUT}")
    stop()

    while robot.step(timestep) != -1:
        pass


try:
    main()
finally:
    stop()