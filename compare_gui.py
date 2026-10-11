import math
import random
import statistics
import time
import tkinter as tk
from tkinter import ttk, messagebox

from bfs import find_path_bfs
from planner import find_path
from view_map import RoboBenchApp, RADIUS, CELL_SIZE

REPEATS = 20
WARMUP_RUNS = 3
ALGORITHMS = ["A-star", "Dijkstra", "BFS"]


class ComparisonApp(RoboBenchApp):
    def __init__(self, window):
        self.table = None
        super().__init__(window)

        window.title("RoboBench — Simulator and Comparison")

        comparison = ttk.Frame(window, padding=10)
        comparison.pack(fill="x")

        ttk.Button(
            comparison,
            text="Compare All",
            command=self.compare_all,
        ).pack(anchor="w", pady=(0, 8))

        columns = (
            "algorithm", "route", "distance", "time", "nodes"
        )

        self.table = ttk.Treeview(
            comparison,
            columns=columns,
            show="headings",
            height=3,
        )

        headings = {
            "algorithm": "Algorithm",
            "route": "Route found",
            "distance": "Distance (px)",
            "time": "Median time (ms)",
            "nodes": "Nodes expanded",
        }

        for column in columns:
            self.table.heading(column, text=headings[column])
            self.table.column(column, width=115, anchor="center")

        self.table.pack(fill="x")

        ttk.Label(
            comparison,
            text=(
                "Timing: 20 runs per algorithm after warm-up. "
                "Small differences may reflect measurement noise."
            ),
            wraplength=600,
        ).pack(anchor="w", pady=(8, 0))

    def show_map(self):
        result = super().show_map()

        if self.table is not None:
            for item in self.table.get_children():
                self.table.delete(item)

        return result

    def plan_with(self, algorithm, stats):
        if algorithm == "BFS":
            return find_path_bfs(
                self.start,
                self.goal,
                self.obstacles,
                RADIUS,
                CELL_SIZE,
                stats=stats,
            )

        return find_path(
            self.start,
            self.goal,
            self.obstacles,
            RADIUS,
            CELL_SIZE,
            use_heuristic=(algorithm == "A-star"),
            stats=stats,
        )

    def validate_route(self, path):
        if not path:
            return

        if path[0] != self.start or path[-1] != self.goal:
            raise ValueError("Incorrect route endpoints.")

        for x, y in path:
            if not (
                RADIUS <= x <= 600 - RADIUS
                and RADIUS <= y <= 400 - RADIUS
            ):
                raise ValueError("Route leaves the map.")

        for a, b in zip(path, path[1:]):
            dx = abs(b[0] - a[0])
            dy = abs(b[1] - a[1])

            if (dx, dy) not in [
                (CELL_SIZE, 0), (0, CELL_SIZE)
            ]:
                raise ValueError("Invalid route step.")

            if self.segment_hits_obstacle(a, b):
                raise ValueError("Route touches an obstacle.")

    def path_distance(self, path):
        return sum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(path, path[1:])
        )

    def compare_all(self):
        if not self.show_map():
            return

        timings = {name: [] for name in ALGORITHMS}
        results = {}
        rng = random.Random(42)

        try:
            for algorithm in ALGORITHMS:
                for _ in range(WARMUP_RUNS):
                    self.plan_with(algorithm, {})

            for _ in range(REPEATS):
                order = ALGORITHMS.copy()
                rng.shuffle(order)

                for algorithm in order:
                    stats = {}

                    started = time.perf_counter_ns()
                    path = self.plan_with(algorithm, stats)
                    elapsed_ms = (
                        time.perf_counter_ns() - started
                    ) / 1_000_000

                    # Validation is excluded from the timing.
                    self.validate_route(path)
                    timings[algorithm].append(elapsed_ms)

                    results[algorithm] = {
                        "found": bool(path),
                        "distance": (
                            self.path_distance(path) if path else None
                        ),
                        "nodes": stats["nodes_expanded"],
                    }

            reference = results["Dijkstra"]

            for algorithm in ALGORITHMS:
                result = results[algorithm]

                if result["found"] != reference["found"]:
                    raise ValueError(
                        f"{algorithm} disagrees on reachability."
                    )

                if result["found"] and not math.isclose(
                    result["distance"], reference["distance"]
                ):
                    raise ValueError(
                        f"{algorithm} disagrees on shortest distance."
                    )

        except ValueError as error:
            self.status.set("Comparison failed.")
            messagebox.showerror("Comparison error", str(error))
            return

        for algorithm in ALGORITHMS:
            result = results[algorithm]

            self.table.insert(
                "",
                "end",
                values=(
                    algorithm,
                    "Yes" if result["found"] else "No",
                    (
                        f"{result['distance']:.0f}"
                        if result["found"] else "N/A"
                    ),
                    f"{statistics.median(timings[algorithm]):.3f}",
                    result["nodes"],
                ),
            )

        self.status.set(
            "Comparison complete. Select an algorithm and press Run "
            "to watch its route. Map changes clear the comparison."
        )


if __name__ == "__main__":
    root = tk.Tk()
    app = ComparisonApp(root)
    root.mainloop()