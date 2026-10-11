import math
import random
import tkinter as tk
from tkinter import ttk, messagebox

from compare_gui import ComparisonApp
from motion import movement_is_safe
from view_map import RADIUS

SPEED = 2.0
MAX_STEPS = 10000
FRAME_DELAY_MS = 10


class NoiseApp(ComparisonApp):
    def __init__(self, window):
        super().__init__(window)
        window.title("RoboBench — Motion Uncertainty")

        controls = ttk.Frame(window, padding=10)
        controls.pack(fill="x")

        ttk.Label(
            controls, text="Movement noise σ:"
        ).pack(side="left")

        self.noise_value = tk.StringVar(value="1.0")
        ttk.Entry(
            controls,
            textvariable=self.noise_value,
            width=7,
        ).pack(side="left", padx=5)

        ttk.Label(
            controls, text="Motion seed:"
        ).pack(side="left")

        self.motion_seed = tk.StringVar(value="0")
        ttk.Entry(
            controls,
            textvariable=self.motion_seed,
            width=7,
        ).pack(side="left", padx=5)

        ttk.Label(
            controls,
            text="Set values, then press Run.",
        ).pack(side="left", padx=10)

        ttk.Label(
            window,
            text=(
                "Orange: planned route. Red trace: actual movement. "
                "Compare All measures route planning, not noisy motion."
            ),
            padding=10,
            wraplength=600,
        ).pack()

    def run(self):
        try:
            sigma = float(self.noise_value.get())
            seed = int(self.motion_seed.get())

            if not math.isfinite(sigma) or sigma < 0:
                raise ValueError

        except ValueError:
            messagebox.showerror(
                "Invalid settings",
                "Noise must be a finite non-negative number. "
                "The motion seed must be a whole number.",
            )
            return

        # The existing Run method plans the route and schedules
        # this subclass's update method.
        super().run()

        self.active_sigma = sigma
        self.active_seed = seed
        self.rng = random.Random(seed)
        self.motion_steps = 0

        if self.path:
            self.status.set(
                f"{self.algorithm.get()} | Noise σ: {sigma} px | "
                f"Seed: {seed} | Running"
            )

    def draw_robot(self, colour="blue"):
        x, y = self.position

        self.canvas.coords(
            self.robot,
            x - RADIUS, y - RADIUS,
            x + RADIUS, y + RADIUS,
        )
        self.canvas.itemconfig(self.robot, fill=colour)
        self.canvas.tag_raise(self.robot)

    def finish(self, outcome):
        colour = "blue" if outcome == "SUCCESS" else "red"
        self.draw_robot(colour)

        self.status.set(
            f"{outcome} | {self.algorithm.get()} | "
            f"Noise σ: {self.active_sigma} px | "
            f"Distance: {self.distance:.1f} px | "
            f"Movement steps: {self.motion_steps}"
        )

    def update(self):
        self.pending = None

        if self.motion_steps >= MAX_STEPS:
            self.finish("TIMEOUT")
            return

        # Use the same one-pixel waypoint tolerance as motion.py.
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

        safe = movement_is_safe(
            self.position,
            next_position,
            self.obstacles,
            RADIUS,
        )

        self.position = next_position
        self.draw_robot()

        if not safe:
            # Show the attempted end position, as recorded
            # by the headless motion simulation.
            self.finish("COLLISION")
            return

        self.pending = self.window.after(
            FRAME_DELAY_MS, self.update
        )


if __name__ == "__main__":
    root = tk.Tk()
    app = NoiseApp(root)
    root.mainloop()