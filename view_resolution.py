import math
import tkinter as tk
from tkinter import ttk

from planner import find_path
from test_maps import START, GOAL, RADIUS, generate_obstacles

WIDTH = 600
HEIGHT = 400
SCALE = 0.6
GRID_SIZES = [10, 20, 40]


class ResolutionViewer:
    def __init__(self, window):
        self.window = window
        window.title("RoboBench — Grid Resolution Comparison")

        controls = ttk.Frame(window, padding=10)
        controls.pack(fill="x")

        ttk.Label(controls, text="Map seed:").pack(side="left")

        self.seed = tk.StringVar(value="37")
        entry = ttk.Entry(
            controls, textvariable=self.seed, width=6
        )
        entry.pack(side="left", padx=5)
        entry.bind("<Return>", lambda event: self.show())

        ttk.Button(
            controls, text="Show", command=self.show
        ).pack(side="left", padx=5)

        ttk.Label(
            controls,
            text="Maps missed at 40px: 37, 42, 58, 61, 65, 81",
        ).pack(side="left", padx=10)

        panels = ttk.Frame(window, padding=10)
        panels.pack()

        self.canvases = {}
        self.labels = {}

        for column, grid_size in enumerate(GRID_SIZES):
            panel = ttk.Frame(panels)
            panel.grid(row=0, column=column, padx=5)

            ttk.Label(
                panel, text=f"{grid_size}-pixel grid"
            ).pack(pady=5)

            canvas = tk.Canvas(
                panel,
                width=int(WIDTH * SCALE),
                height=int(HEIGHT * SCALE),
                bg="white",
            )
            canvas.pack()

            label = ttk.Label(
                panel, text="", wraplength=350
            )
            label.pack(pady=8)

            self.canvases[grid_size] = canvas
            self.labels[grid_size] = label

        self.status = tk.StringVar()
        ttk.Label(
            window, textvariable=self.status, padding=10
        ).pack()

        self.show()

    def circle(self, canvas, point, radius, colour):
        x, y = point
        canvas.create_oval(
            (x - radius) * SCALE,
            (y - radius) * SCALE,
            (x + radius) * SCALE,
            (y + radius) * SCALE,
            fill=colour,
        )

    def show(self):
        try:
            seed = int(self.seed.get())
            if not 0 <= seed <= 99:
                raise ValueError
        except ValueError:
            self.status.set("Enter a whole-number seed from 0 to 99.")
            return

        obstacles = generate_obstacles(seed)

        for grid_size in GRID_SIZES:
            canvas = self.canvases[grid_size]
            canvas.delete("all")
            stats = {}

            path = find_path(
                START, GOAL, obstacles, RADIUS, grid_size,
                stats=stats,
            )

            # The search grid is anchored at START, not necessarily
            # at the map's top-left corner.
            for x in range(START[0] % grid_size, WIDTH + 1, grid_size):
                canvas.create_line(
                    x * SCALE, 0,
                    x * SCALE, HEIGHT * SCALE,
                    fill="#dddddd",
                )

            for y in range(START[1] % grid_size, HEIGHT + 1, grid_size):
                canvas.create_line(
                    0, y * SCALE,
                    WIDTH * SCALE, y * SCALE,
                    fill="#dddddd",
                )

            # Faint red areas show the planner's safety margins.
            for left, top, right, bottom in obstacles:
                canvas.create_rectangle(
                    (left - RADIUS) * SCALE,
                    (top - RADIUS) * SCALE,
                    (right + RADIUS) * SCALE,
                    (bottom + RADIUS) * SCALE,
                    fill="#ffe0e0",
                    outline="#e6aaaa",
                )

            for obstacle in obstacles:
                canvas.create_rectangle(
                    *(value * SCALE for value in obstacle),
                    fill="grey",
                )

            for a, b in zip(path, path[1:]):
                canvas.create_line(
                    a[0] * SCALE, a[1] * SCALE,
                    b[0] * SCALE, b[1] * SCALE,
                    fill="orange", width=3,
                )

            self.circle(canvas, START, RADIUS, "blue")
            self.circle(canvas, GOAL, 10, "green")

            if path:
                distance = sum(
                    math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(path, path[1:])
                )

                text = (
                    f"Route found: {distance:.0f} px\n"
                    f"Nodes expanded: {stats['nodes_expanded']}"
                )
            else:
                text = (
                    "No route found\n"
                    f"Nodes expanded: {stats['nodes_expanded']}"
                )

            self.labels[grid_size].config(text=text)

        self.status.set(
            f"Seed {seed}: identical obstacles and robot size. "
            "Red shading shows conservative obstacle safety margins."
        )


if __name__ == "__main__":
    root = tk.Tk()
    app = ResolutionViewer(root)
    root.mainloop()