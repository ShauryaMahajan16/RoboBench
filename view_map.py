import math
import tkinter as tk
from tkinter import ttk

from bfs import find_path_bfs
from planner import find_path
from test_maps import generate_obstacles

WIDTH = 600
HEIGHT = 400
START = (60, 200)
GOAL = (540, 200)
RADIUS = 12
CELL_SIZE = 20
SPEED = 3


class RoboBenchApp:
    def __init__(self, window):
        self.window = window
        self.window.title("RoboBench — Interactive Simulation")

        self.pending = None
        self.path = []
        self.obstacles = []
        self.robot = None
        self.position = START
        self.waypoint = 1
        self.distance = 0

        controls = ttk.Frame(window, padding=10)
        controls.pack(fill="x")

        ttk.Label(controls, text="Map:").pack(side="left")

        self.map_name = tk.StringVar(value="Two obstacles")
        map_picker = ttk.Combobox(
            controls,
            textvariable=self.map_name,
            values=[
                "Clear map",
                "Two obstacles",
                "Blocking wall",
                "Generated map",
            ],
            state="readonly",
            width=16,
        )
        map_picker.pack(side="left", padx=5)
        map_picker.bind(
            "<<ComboboxSelected>>",
            lambda event: self.show_map(),
        )

        ttk.Label(controls, text="Seed:").pack(side="left")

        self.seed = tk.StringVar(value="0")
        seed_entry = ttk.Entry(
            controls, textvariable=self.seed, width=5
        )
        seed_entry.pack(side="left", padx=5)
        seed_entry.bind("<Return>", lambda event: self.show_map())

        self.algorithm = tk.StringVar(value="A-star")
        algorithm_picker = ttk.Combobox(
            controls,
            textvariable=self.algorithm,
            values=["A-star", "Dijkstra", "BFS"],
            state="readonly",
            width=10,
        )
        algorithm_picker.pack(side="left", padx=5)
        algorithm_picker.bind(
            "<<ComboboxSelected>>",
            lambda event: self.show_map(),
        )

        ttk.Button(
            controls, text="Run", command=self.run
        ).pack(side="left", padx=5)

        ttk.Button(
            controls, text="Reset", command=self.show_map
        ).pack(side="left", padx=5)

        self.canvas = tk.Canvas(
            window, width=WIDTH, height=HEIGHT, bg="white"
        )
        self.canvas.pack(padx=10)

        self.status = tk.StringVar()
        ttk.Label(
            window, textvariable=self.status, padding=10
        ).pack()

        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.show_map()

    def stop_animation(self):
        if self.pending is not None:
            self.window.after_cancel(self.pending)
            self.pending = None

    def get_obstacles(self):
        name = self.map_name.get()

        if name == "Clear map":
            return []

        if name == "Two obstacles":
            return [
                (200, 100, 260, 280),
                (360, 220, 440, 300),
            ]

        if name == "Blocking wall":
            return [(280, 0, 320, 400)]

        seed = int(self.seed.get())

        if not 0 <= seed <= 99:
            raise ValueError("Seed must be between 0 and 99.")

        return generate_obstacles(seed)

    def circle(self, point, radius, colour):
        x, y = point

        return self.canvas.create_oval(
            x - radius, y - radius,
            x + radius, y + radius,
            fill=colour,
        )

    def show_map(self):
        self.stop_animation()
        self.path = []

        try:
            self.obstacles = self.get_obstacles()
        except ValueError:
            self.status.set("Enter a whole-number seed from 0 to 99.")
            return False

        self.position = START
        self.waypoint = 1
        self.distance = 0
        self.canvas.delete("all")

        for x in range(0, WIDTH + 1, CELL_SIZE):
            self.canvas.create_line(
                x, 0, x, HEIGHT, fill="#eeeeee"
            )

        for y in range(0, HEIGHT + 1, CELL_SIZE):
            self.canvas.create_line(
                0, y, WIDTH, y, fill="#eeeeee"
            )

        for obstacle in self.obstacles:
            self.canvas.create_rectangle(
                *obstacle, fill="grey"
            )

        self.circle(GOAL, 10, "green")
        self.canvas.create_text(
            GOAL[0], GOAL[1] - 25, text="Goal"
        )

        self.robot = self.circle(START, RADIUS, "blue")
        self.status.set("Ready. Press Run to plan and move.")
        return True

    def run(self):
        if not self.show_map():
            return

        stats = {}
        algorithm = self.algorithm.get()

        if algorithm == "BFS":
            self.path = find_path_bfs(
                START, GOAL, self.obstacles, RADIUS, CELL_SIZE,
                stats=stats,
            )
        else:
            self.path = find_path(
                START, GOAL, self.obstacles, RADIUS, CELL_SIZE,
                use_heuristic=(algorithm == "A-star"),
                stats=stats,
            )

        self.nodes = stats["nodes_expanded"]

        if not self.path:
            self.status.set(
                f"{algorithm}: No route found. "
                f"Nodes expanded: {self.nodes}"
            )
            return

        for a, b in zip(self.path, self.path[1:]):
            self.canvas.create_line(
                *a, *b, fill="orange", width=3, tags="route"
            )

        self.canvas.tag_lower("route", self.robot)
        self.status.set(
            f"{algorithm}: Running | Nodes expanded: {self.nodes}"
        )
        self.pending = self.window.after(20, self.update)

    def segment_hits_obstacle(self, a, b):
        # Movement follows horizontal or vertical route segments.
        min_x, max_x = sorted((a[0], b[0]))
        min_y, max_y = sorted((a[1], b[1]))

        for left, top, right, bottom in self.obstacles:
            gap_x = max(left - max_x, min_x - right, 0)
            gap_y = max(top - max_y, min_y - bottom, 0)

            if math.hypot(gap_x, gap_y) <= RADIUS:
                return True

        return False

    def update(self):
        self.pending = None

        if self.waypoint >= len(self.path):
            self.status.set(
                f"SUCCESS: {self.algorithm.get()} reached the goal | "
                f"Distance: {self.distance:.0f} px | "
                f"Nodes expanded: {self.nodes}"
            )
            return

        x, y = self.position
        target_x, target_y = self.path[self.waypoint]

        dx = target_x - x
        dy = target_y - y
        remaining = math.hypot(dx, dy)

        if remaining <= 0.000001:
            self.waypoint += 1
        else:
            step = min(SPEED, remaining)
            next_position = (
                x + step * dx / remaining,
                y + step * dy / remaining,
            )

            if self.segment_hits_obstacle(
                self.position, next_position
            ):
                self.status.set("FAILED: Movement touches an obstacle.")
                return

            self.position = next_position
            self.distance += step

            next_x, next_y = self.position
            self.canvas.coords(
                self.robot,
                next_x - RADIUS, next_y - RADIUS,
                next_x + RADIUS, next_y + RADIUS,
            )

        self.pending = self.window.after(20, self.update)

    def close(self):
        self.stop_animation()
        self.window.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = RoboBenchApp(root)
    root.mainloop()