# RoboBench: Effects of Grid Resolution, Planning Clearance, and Route Selection on Simulated Robot Navigation

Author: Shaurya Mahajan

Status: Initial draft. Related work, references, figures, and additional
experimental validation remain to be completed.

## Abstract

This study evaluates how grid resolution, planning clearance, and route
selection affect simulated robot navigation. RoboBench compares A-star,
Dijkstra, and breadth-first search on static rectangular obstacle maps.
Across 100 generated maps, 10- and 20-pixel grids found routes on 95 maps,
while a 40-pixel grid found routes on 89. A coarse-to-fine retry strategy
preserved fine-grid reachability on 244 passage cases but did not reduce
average planning time. In a separate motion experiment, additional
planning clearance reduced collisions under Gaussian displacement noise
while making some maps unplannable. Finally, nine Webots trials showed
different execution times despite equal planned distances within each
layout. These results illustrate tradeoffs between planning computation,
route availability, and execution performance in the tested environments.

## 1. Introduction

A planned route is only one component of robot navigation. The grid
representation determines which routes a planner can search. Safety
margins change the available free space. The route-following controller
then determines how the selected route is executed.

RoboBench examines three questions:

1. How does grid spacing affect route availability, distance, and
   planning computation?
2. How does extra planning clearance affect execution under movement
   uncertainty?
3. Can equally short planned routes produce different execution times?

The project combines controlled 2D experiments with a small Webots
execution study. It evaluates existing planning methods rather than
introducing a new path-planning algorithm.

## 2. Methods

### 2.1 Planning model

The 2D environment measures 600 by 400 pixels. The physical robot is
represented by a circle with radius 12 pixels. Movement is restricted
to horizontal and vertical grid edges.

A-star uses Manhattan distance. Dijkstra uses the same priority-queue
search with a zero heuristic. Breadth-first search uses a separate
queue-based implementation. All methods use the same grid costs and
planning assumptions.

Planning uses conservative rectangular obstacle inflation. Returned
routes are checked for endpoint correctness, valid steps, map boundaries,
and circular robot clearance. Agreement between planners provides an
additional consistency check, rather than proof of correctness.

### 2.2 Grid resolution and resolution retry

One hundred maps, generated using seeds 0 through 99, were evaluated
at grid spacings of 10, 20, and 40 pixels.

Each planner received two warm-up runs and ten recorded runs per map
and spacing. Execution order was shuffled using a fixed seed. Timing
excluded external route validation and file writing.

A separate experiment evaluated 244 passage layouts. Passage widths
were 20, 30, 40, and 60 pixels, with 61 vertical positions per width.
Fixed 10-pixel A-star was compared with a strategy that tried 40,
20, and 10 pixels, stopping at the first successful search.

### 2.3 Movement uncertainty and planning clearance

A simplified holonomic controller moved towards successive waypoints.
Each step commanded up to two pixels of movement and received independent
Gaussian displacement error on each axis. The controller had perfect
position feedback.

The robot used a one-pixel waypoint tolerance and a 10,000-step limit.
Collision checking covered the entire movement segment; obstacle
touching counted as collision.

The noise experiment used 19 reachable maps and 20 motion seeds per map.
The clearance experiment included all 20 generated maps, including
unavailable routes. Extra planning clearances were 0, 4, 8, and 12 pixels,
with movement noise fixed at sigma 2 pixels per axis per step.

### 2.4 Webots execution

An E-puck robot followed routes around a static box in three placements.
Each placement was tested once with each planner, producing nine trials.

Obstacle footprints were obtained from the Webots world geometry.
Trials used the same route-following controller, perfect Supervisor
position and orientation feedback, and no deliberately added motion
noise. Physical contact monitoring covered the named obstacle box.

Execution duration was measured in simulation seconds, separately from
planning computation time.

## 3. Results

### 3.1 Grid resolution

| Grid spacing | Maps with routes | A-star time | Dijkstra time | BFS time |
|---|---:|---:|---:|---:|
| 10 px | 95/100 | 4.308 ms | 12.332 ms | 3.461 ms |
| 20 px | 95/100 | 1.309 ms | 3.333 ms | 1.047 ms |
| 40 px | 89/100 | 0.416 ms | 0.748 ms | 0.291 ms |

Times are means of per-map median planning times, including no-route
cases, from one local experiment run.

The 10- and 20-pixel grids returned identical shortest distances on all
95 mutually reachable maps. Compared with 40 pixels, the 20-pixel grid
recovered six routes and shortened routes on 39 of the 89 maps reachable
at both spacings.

### 3.2 Coarse-to-fine retry

Both fixed 10-pixel planning and resolution retry found routes in
153 of 244 passage cases.

For reachable cases, mean case-median times were 0.761 ms for fixed
planning and 0.792 ms for retry. For no-route cases, they were 5.104 ms
and 6.880 ms.

Retry returned longer routes in 41 successful cases. Mean extra distance
across all 153 successful cases was 8.0 pixels, with a maximum of 60 pixels.

### 3.3 Planning clearance under movement noise

| Extra clearance | Successes / 400 | Collisions | No-route cases |
|---|---:|---:|---:|
| 0 px | 193 | 187 | 20 |
| 4 px | 193 | 187 | 20 |
| 8 px | 256 | 24 | 120 |
| 12 px | 240 | 0 | 160 |

No timeouts occurred. Eight pixels produced the highest overall success
among tested clearances. Twelve pixels produced no observed collisions,
but more cases had no planned route.

On the 12 maps reachable at every clearance, successes increased from
124/240 at zero extra clearance to 240/240 at eight pixels. Mean planned
distance increased from 650.0 to 713.3 pixels.

### 3.4 Webots execution

| Planner | Goals reached | Detected box collisions | Mean execution |
|---|---:|---:|---:|
| A-star | 3/3 | 0 | 81.08 s |
| Dijkstra | 3/3 | 0 | 72.14 s |
| BFS | 3/3 | 0 | 69.90 s |

Within each layout, all three planners returned equal planned distances.
A-star routes contained five straight segments; Dijkstra and BFS routes
contained four.

These observations show that equal planned distance did not produce
equal execution time in this controller and layout set. They do not
establish a general ranking of the planners.

## 4. Discussion

Coarser grids reduced planning computation but sometimes removed routes
or increased their distance. Trying coarse grids first did not provide
an average speed benefit in the tested passage cases.

Extra clearance improved observed execution success under displacement
noise, but also excluded routes. Therefore, minimizing collisions and
maximizing completed tasks across all maps can favour different settings.

The Webots results suggest that route structure matters during execution.
The controller slows near waypoints, and selected routes differed in
segment lengths and turning requirements. Further controlled experiments
are needed to separate these effects.

## 5. Limitations

Generated obstacles align to a 20-pixel grid, which can favour particular
resolutions. Designed passage cases supplement this map set.

The 2D movement model has perfect position feedback and no wheel dynamics.
It is distinct from the Webots model; their execution measurements should
not be pooled.

The Webots study uses one box, three placements, and one trial per
planner per placement. It cannot establish reliability or general
performance differences.

Trials sharing a map are not independent environment samples.
Timing depends on hardware, system load, and implementation details.
No statistical significance analysis has been performed.

Conservative obstacle inflation can exclude physically valid routes.
Automated checks and agreement between planners do not eliminate every
possible implementation error.

## 6. Conclusion

In the tested environments, navigation performance depended on grid
resolution, planning clearance, and the route selected for execution.
Shortest grid distance alone did not describe task success or execution
duration. RoboBench provides an experimental framework for examining
these tradeoffs.

## References

To be completed after reviewing primary sources on graph search,
robot configuration spaces, benchmarking, and Webots.