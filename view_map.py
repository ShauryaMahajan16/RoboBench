import tkinter as tk
from tkinter import ttk

from planner import find_path
from test_maps import generate_obstacles, route_distance

START = (60, 200)
GOAL = (540, 200)
RADIUS = 12
CELL_SIZE = 20

window = tk.Tk()
window.title("RoboBench — Generated Map Viewer")

controls = ttk.Frame(window, padding=10)
controls.pack(fill="x")

ttk.Label(controls, text="Map seed:").pack(side="left")

seed_value = tk.StringVar(value="0")
seed_input = ttk.Entry(
    controls, textvariable=seed_value, width=6
)
seed_input.pack(side="left", padx=5)

algorithm_value = tk.StringVar(value="A-star")
algorithm_input = ttk.Combobox(
    controls,
    textvariable=algorithm_value,
    values=["A-star", "Dijkstra"],
    state="readonly",
    width=10,
)
algorithm_input.pack(side="left", padx=5)

canvas = tk.Canvas(
    window, width=600, height=400, bg="white"
)
canvas.pack(padx=10)

status_value = tk.StringVar()
ttk.Label(
    window, textvariable=status_value, padding=10
).pack()


def draw_circle(point, radius, colour):
    x, y = point

    canvas.create_oval(
        x - radius, y - radius,
        x + radius, y + radius,
        fill=colour,
    )


def show_map():
    try:
        seed = int(seed_value.get())

        if not 0 <= seed <= 99:
            raise ValueError
    except ValueError:
        status_value.set("Enter a whole-number seed from 0 to 99.")
        return

    obstacles = generate_obstacles(seed)
    algorithm = algorithm_value.get()

    path = find_path(
        START,
        GOAL,
        obstacles,
        RADIUS,
        CELL_SIZE,
        use_heuristic=(algorithm == "A-star"),
    )

    canvas.delete("all")

    for x in range(0, 601, CELL_SIZE):
        canvas.create_line(x, 0, x, 400, fill="#eeeeee")

    for y in range(0, 401, CELL_SIZE):
        canvas.create_line(0, y, 600, y, fill="#eeeeee")

    for obstacle in obstacles:
        canvas.create_rectangle(*obstacle, fill="grey")

    if path:
        for a, b in zip(path, path[1:]):
            canvas.create_line(
                *a, *b, fill="orange", width=3
            )

    draw_circle(START, RADIUS, "blue")
    draw_circle(GOAL, 10, "green")

    canvas.create_text(START[0], START[1] - 25, text="Start")
    canvas.create_text(GOAL[0], GOAL[1] - 25, text="Goal")

    if path:
        status_value.set(
            f"Seed {seed} | {algorithm} | "
            f"Route found | Distance: {route_distance(path):.0f} px"
        )
    else:
        status_value.set(
            f"Seed {seed} | {algorithm} | No route found"
        )


def next_map():
    try:
        seed = int(seed_value.get())
    except ValueError:
        seed = -1

    seed_value.set(str((seed + 1) % 100))
    show_map()


def find_blocked_map():
    for seed in range(100):
        obstacles = generate_obstacles(seed)

        path = find_path(
            START, GOAL, obstacles, RADIUS, CELL_SIZE
        )

        if not path:
            seed_value.set(str(seed))
            show_map()
            return

    status_value.set("No blocked map found.")


ttk.Button(
    controls, text="Show map", command=show_map
).pack(side="left", padx=5)

ttk.Button(
    controls, text="Next map", command=next_map
).pack(side="left", padx=5)

ttk.Button(
    controls, text="Find blocked map", command=find_blocked_map
).pack(side="left", padx=5)

algorithm_input.bind(
    "<<ComboboxSelected>>", lambda event: show_map()
)
seed_input.bind("<Return>", lambda event: show_map())

show_map()
window.mainloop()