import math
import random
import tkinter as tk
from tkinter import ttk, messagebox

from bfs import find_path_bfs
from planner import find_path
from compare_gui import ComparisonApp
from motion import movement_is_safe
from view_map import RADIUS, CELL_SIZE

SPEED = 2.0
MAX_STEPS = 10000
FRAME_DELAY_MS = 10


class NoiseApp(ComparisonApp):
    def __init__(self, window):
        self.active_extra = 0.0
        super().__init__(window)
        window.title("RoboBench — Noise and Planning Clearance")

        controls = ttk.Frame(window, padding=10)
        controls.pack(fill="x")

        ttk.Label(controls, text="Noise σ:").pack(side="left")
        self.noise_value = tk.StringVar(value="2.0")
        ttk.Entry(
            controls, textvariable=self.noise_value, width=6
        ).pack(side="left", padx=5)

        ttk.Label(controls, text="Motion seed:").pack(side="left")
        self.motion_seed = tk.StringVar(value="0")
        ttk.Entry(
            controls, textvariable=self.motion_seed, width=6
        ).pack(side="left", padx=5)

        ttk.Label(
            controls, text="Extra clearance:"
        ).pack(side="left")
        self.clearance_value = tk.StringVar(value="0")
        ttk.Entry(
            controls, textvariable=self.clearance_value, width=6
        ).pack(side="left", padx=5)

        ttk.Label(
            window,
            text=(
                "Physical robot radius: 12px. Orange: planned route. "
                "Red: actual movement. Settings apply when you press Run."
            ),
            padding=10,
            wraplength=600,
        ).pack()

        ttk.Label(
            window,
            text=(
                "Compare All uses the selected clearance for planning. "
                "It does not measure noisy-motion success."
            ),
            padding=(10, 0, 10, 10),
            wraplength=600,
        ).pack()

    def read_clearance(self):
        extra = float(self.clearance_value.get())

        if not math.isfinite(extra) or extra < 0:
            raise ValueError

        return extra

    def plan_with(self, algorithm, stats):
        planning_radius = RADIUS + self.active_extra

        if algorithm == "BFS":
            return find_path_bfs(
                self.start,
                self.goal,
                self.obstacles,
                planning_radius,
                CELL_SIZE,
                stats=stats,
            )

        return find_path(
            self.start,
            self.goal,
            self.obstacles,
            planning_radius,
            CELL_SIZE,
            use_heuristic=(algorithm == "A-star"),
            stats=stats,
        )

    def compare_all(self):
        try:
            self.active_extra = self.read_clearance()
        except ValueError:
            messagebox.showerror(
                "Invalid clearance",
                "Extra clearance must be a finite non-negative number.",
            )
            return

        super().compare_all()

    def run(self):
        try:
            sigma = float(self.noise_value.get())
            seed = int(self.motion_seed.get())
            extra = self.read_clearance()

            if not math.isfinite(sigma) or sigma < 0:
                raise ValueError

        except ValueError:
            messagebox.showerror(
                "Invalid settings",
                "Noise and clearance must be finite non-negative numbers. "
                "The motion seed must be a whole number.",
            )
            return

        if not self.show_map():
            return

        self.active_extra = extra
        self.active_sigma = sigma
        self.active_seed = seed
        self.rng = random.Random(seed)
        self.motion_steps = 0

        stats = {}
        self.path = self.plan_with(self.algorithm.get(), stats)
        self.nodes = stats["nodes_expanded"]

        if not self.path:
            self.status.set(
                f"No route with {extra:g}px extra clearance. "
                "Try less clearance or a wider passage."
            )
            return

        try:
            self.validate_route(self.path)
        except ValueError as error:
            self.status.set("Route validation failed.")
            messagebox.showerror("Invalid route", str(error))
            return

        for a, b in zip(self.path, self.path[1:]):
            self.canvas.create_line(
                *a, *b,
                fill="orange",
                width=3,
                tags="route",
            )

        self.canvas.tag_lower("route", self.robot)
        self.status.set(
            f"{self.algorithm.get()} | Noise σ: {sigma:g}px | "
            f"Extra clearance: {extra:g}px | Seed: {seed} | Running"
        )

        self.pending = self.window.after(
            FRAME_DELAY_MS, self.update
        )

    def draw_robot(self, colour="blue"):
        x, y = self.position

        # Physical size remains unchanged.
        self.canvas.coords(
            self.robot,
            x - RADIUS, y - RADIUS,
            x + RADIUS, y + RADIUS,
        )
        self.canvas.itemconfig(self.robot, fill=colour)
        self.canvas.tag_raise(self.robot)

    def finish(self, outcome):
        self.draw_robot(
            "blue" if outcome == "SUCCESS" else "red"
        )

        self.status.set(
            f"{outcome} | {self.algorithm.get()} | "
            f"Noise σ: {self.active_sigma:g}px | "
            f"Extra clearance: {self.active_extra:g}px | "
            f"Distance: {self.distance:.1f}px | "
            f"Steps: {self.motion_steps}"
        )

    def update(self):
        self.pending = None

        if self.motion_steps >= MAX_STEPS:
            self.finish("TIMEOUT")
            return

        while self.waypoint < len(self.path):
            target = self.path[self.waypoint]

            if math.dist(self.position, target) > 1.0:
                break

            self.waypoint += 1

        if self.waypoint >= len(self.path):
            self.finish("SUCCESS")
            return

        x, y = self.position
        target_x, target_y = self.path[self.waypoint]
        dx, dy = target_x - x, target_y - y
        remaining = math.hypot(dx, dy)
        commanded_step = min(SPEED, remaining)

        next_position = (
            x + commanded_step * dx / remaining
            + self.rng.gauss(0, self.active_sigma),
            y + commanded_step * dy / remaining
            + self.rng.gauss(0, self.active_sigma),
        )

        self.motion_steps += 1
        self.distance += math.dist(self.position, next_position)

        self.canvas.create_line(
            *self.position,
            *next_position,
            fill="#cc3333",
            width=2,
            tags="actual_motion",
        )

        # Collision checks use the actual robot radius.
        safe = movement_is_safe(
            self.position,
            next_position,
            self.obstacles,
            RADIUS,
        )

        self.position = next_position
        self.draw_robot()

        if not safe:
            self.finish("COLLISION")
            return

        self.pending = self.window.after(
            FRAME_DELAY_MS, self.update
        )


if __name__ == "__main__":
    root = tk.Tk()
    app = NoiseApp(root)
    root.mainloop()