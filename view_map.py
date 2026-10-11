import math
import tkinter as tk
from tkinter import ttk

from bfs import find_path_bfs
from planner import find_path
from test_maps import generate_obstacles

WIDTH = 600
HEIGHT = 400
RADIUS = 12
CELL_SIZE = 20
SPEED = 3
DEFAULT_START = (60, 200)
DEFAULT_GOAL = (540, 200)


class RoboBenchApp:
    def __init__(self, window):
        self.window = window
        window.title("RoboBench — Custom Map Simulator")

        self.pending = None
        self.path = []
        self.custom_obstacles = []
        self.custom_start = DEFAULT_START
        self.custom_goal = DEFAULT_GOAL
        self.robot = None
        self.nodes = 0

        controls = ttk.Frame(window, padding=10)
        controls.pack(fill="x")

        ttk.Label(controls, text="Map:").pack(side="left")
        self.map_name = tk.StringVar(value="Two obstacles")
        picker = ttk.Combobox(
            controls,
            textvariable=self.map_name,
            values=[
                "Clear map",
                "Two obstacles",
                "Blocking wall",
                "Generated map",
                "Custom map",
            ],
            state="readonly",
            width=16,
        )
        picker.pack(side="left", padx=5)
        picker.bind(
            "<<ComboboxSelected>>", lambda event: self.show_map()
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
            "<<ComboboxSelected>>", lambda event: self.show_map()
        )

        editor = ttk.Frame(window, padding=(10, 0, 10, 10))
        editor.pack(fill="x")

        ttk.Label(editor, text="Click tool:").pack(side="left")
        self.tool = tk.StringVar(value="Obstacle")
        ttk.Combobox(
            editor,
            textvariable=self.tool,
            values=["Obstacle", "Erase", "Start", "Goal"],
            state="readonly",
            width=10,
        ).pack(side="left", padx=5)

        ttk.Button(
            editor, text="Run", command=self.run
        ).pack(side="left", padx=5)

        ttk.Button(
            editor, text="Reset robot", command=self.show_map
        ).pack(side="left", padx=5)

        ttk.Button(
            editor, text="Clear custom map", command=self.clear_custom
        ).pack(side="left", padx=5)

        self.canvas = tk.Canvas(
            window, width=WIDTH, height=HEIGHT, bg="white"
        )
        self.canvas.pack(padx=10)
        self.canvas.bind("<Button-1>", self.edit_map)

        self.status = tk.StringVar()
        ttk.Label(
            window,
            textvariable=self.status,
            padding=10,
            wraplength=600,
        ).pack()

        window.protocol("WM_DELETE_WINDOW", self.close)
        self.show_map()

    def stop_animation(self):
        if self.pending is not None:
            self.window.after_cancel(self.pending)
            self.pending = None

    def load_map(self):
        name = self.map_name.get()
        self.start = DEFAULT_START
        self.goal = DEFAULT_GOAL

        if name == "Custom map":
            self.start = self.custom_start
            self.goal = self.custom_goal
            return self.custom_obstacles.copy()

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
            raise ValueError

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
            self.obstacles = self.load_map()
        except ValueError:
            self.status.set("Enter a whole-number seed from 0 to 99.")
            return False

        self.position = self.start
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
            self.canvas.create_rectangle(*obstacle, fill="grey")

        self.circle(self.goal, 10, "green")
        self.canvas.create_text(
            self.goal[0], self.goal[1] - 25, text="Goal"
        )

        self.robot = self.circle(self.start, RADIUS, "blue")

        if self.map_name.get() == "Custom map":
            self.status.set(
                "Choose a click tool and edit the map. "
                "Start and goal snap to grid points. Press Run."
            )
        else:
            self.status.set("Ready. Press Run to plan and move.")

        return True

    def edit_map(self, event):
        if self.map_name.get() != "Custom map":
            self.status.set("Select Custom map before editing.")
            return

        x, y = event.x, event.y
        if not (0 <= x < WIDTH and 0 <= y < HEIGHT):
            return

        tool = self.tool.get()

        if tool == "Obstacle":
            left = (x // CELL_SIZE) * CELL_SIZE
            top = (y // CELL_SIZE) * CELL_SIZE
            obstacle = (
                left, top, left + CELL_SIZE, top + CELL_SIZE
            )

            if obstacle not in self.custom_obstacles:
                self.custom_obstacles.append(obstacle)

        elif tool == "Erase":
            self.custom_obstacles = [
                obstacle
                for obstacle in self.custom_obstacles
                if not (
                    obstacle[0] <= x < obstacle[2]
                    and obstacle[1] <= y < obstacle[3]
                )
            ]

        else:
            # Snap endpoints to the same grid and keep the
            # robot's body away from the map boundary.
            grid_x = ((x + CELL_SIZE // 2) // CELL_SIZE) * CELL_SIZE
            grid_y = ((y + CELL_SIZE // 2) // CELL_SIZE) * CELL_SIZE

            point = (
                max(CELL_SIZE, min(WIDTH - CELL_SIZE, grid_x)),
                max(CELL_SIZE, min(HEIGHT - CELL_SIZE, grid_y)),
            )

            if tool == "Start":
                self.custom_start = point
            elif tool == "Goal":
                self.custom_goal = point

        self.show_map()

    def clear_custom(self):
        self.custom_obstacles.clear()
        self.custom_start = DEFAULT_START
        self.custom_goal = DEFAULT_GOAL
        self.map_name.set("Custom map")
        self.show_map()

    def run(self):
        if not self.show_map():
            return

        stats = {}
        algorithm = self.algorithm.get()

        if algorithm == "BFS":
            self.path = find_path_bfs(
                self.start, self.goal, self.obstacles,
                RADIUS, CELL_SIZE, stats=stats,
            )
        else:
            self.path = find_path(
                self.start, self.goal, self.obstacles,
                RADIUS, CELL_SIZE,
                use_heuristic=(algorithm == "A-star"),
                stats=stats,
            )

        self.nodes = stats["nodes_expanded"]

        if not self.path:
            self.status.set(
                f"{algorithm}: No route found. "
                "Check endpoint clearance and passage width. "
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
        dx, dy = target_x - x, target_y - y
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