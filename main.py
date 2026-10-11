import tkinter as tk
import math
from planner import find_path

# Create the simulation window.
window = tk.Tk()
window.title("RoboBench — A-star Navigation")

canvas = tk.Canvas(window, width=600, height=400, bg="white")
canvas.pack()

# Draw the planning grid.
cell_size = 20

for x in range(0, 601, cell_size):
    canvas.create_line(x, 0, x, 400, fill="#eeeeee")

for y in range(0, 401, cell_size):
    canvas.create_line(0, y, 600, y, fill="#eeeeee")

# Define and draw the obstacles.
obstacles = [
    (280, 0, 320, 400),
]

for obstacle in obstacles:
    canvas.create_rectangle(*obstacle, fill="grey")

# Set the robot's starting position and movement speed.
robot_x, robot_y = 60, 200
robot_radius = 12
speed = 2
distance_travelled = 0

# Set and draw the goal.
goal_x, goal_y = 540, 200

canvas.create_oval(
    goal_x - 10, goal_y - 10,
    goal_x + 10, goal_y + 10,
    fill="green",
)
canvas.create_text(goal_x, goal_y - 25, text="Goal")

status = canvas.create_text(300, 25, text="Planning route...")

# Find a route that leaves room for the robot's body.
path = find_path(
    (robot_x, robot_y),
    (goal_x, goal_y),
    obstacles,
    robot_radius,
    cell_size,
)

# Draw the planned route.
if path:
    for start, end in zip(path, path[1:]):
        canvas.create_line(
            *start, *end, fill="orange", width=3
        )

    print(f"A-star found a route with {len(path) - 1} grid steps.")
    canvas.itemconfig(status, text="Robot following A-star route...")
else:
    print("A-star could not find a route.")
    canvas.itemconfig(status, text="FAILED: No route found")

# Draw the robot above the route.
robot = canvas.create_oval(
    robot_x - robot_radius, robot_y - robot_radius,
    robot_x + robot_radius, robot_y + robot_radius,
    fill="blue",
)


def hits_obstacle(x, y):
    """Check whether the robot's circular body touches an obstacle."""
    for left, top, right, bottom in obstacles:
        nearest_x = max(left, min(x, right))
        nearest_y = max(top, min(y, bottom))

        distance = math.hypot(x - nearest_x, y - nearest_y)

        if distance <= robot_radius:
            return True

    return False


# Point zero is the start; move towards point one first.
waypoint_index = 1


def update():
    global robot_x, robot_y, distance_travelled, waypoint_index

    if not path:
        return

    if waypoint_index >= len(path):
        canvas.itemconfig(status, text="SUCCESS: Goal reached")
        print(f"Goal reached. Distance: {distance_travelled:.1f} pixels")
        return

    # Select the next point along the planned route.
    target_x, target_y = path[waypoint_index]

    dx = target_x - robot_x
    dy = target_y - robot_y
    remaining = math.hypot(dx, dy)

    # Once we reach this point, select the next one.
    if remaining <= 0.000001:
        waypoint_index += 1
        window.after(20, update)
        return

    # Move towards the waypoint without overshooting it.
    step = min(speed, remaining)
    next_x = robot_x + step * dx / remaining
    next_y = robot_y + step * dy / remaining

    if hits_obstacle(next_x, next_y):
        canvas.itemconfig(status, text="FAILED: Route blocked")
        print(f"Blocked. Distance travelled: {distance_travelled:.1f} pixels")
        return

    robot_x, robot_y = next_x, next_y
    distance_travelled += step

    canvas.coords(
        robot,
        robot_x - robot_radius, robot_y - robot_radius,
        robot_x + robot_radius, robot_y + robot_radius,
    )

    window.after(20, update)


window.after(500, update)
window.mainloop()