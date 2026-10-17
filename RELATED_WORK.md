# Related Work and Research Positioning

## Graph-search foundations

Dijkstra presented a method for finding a minimum-length path between
vertices in a graph [1]. Hart, Nilsson, and Raphael developed the
heuristic search framework associated with A-star [2].

LaValle presents breadth-first search, Dijkstra's algorithm, and A-star
within a common discrete-planning framework [3, Chapter 2].
Breadth-first search is a suitable shortest-path baseline when all
edges have equal cost.

RoboBench uses these established methods on four-connected grids.
Its purpose is to compare their implementations under shared planning
assumptions, rather than propose a new graph-search algorithm.

## Robot geometry and free space

Configuration-space planning represents robot placements as states
and excludes states that collide with obstacles [3, Chapter 4].

RoboBench accounts for robot size through conservative rectangular
obstacle inflation. For a circular robot, this approximation excludes
some corner configurations that an exact geometric model could allow.

The clearance experiment adds a further planning margin while keeping
the physical robot radius fixed. It measures how this chosen margin
changes route availability and execution outcomes under the specified
displacement-noise model.

## Planning and simulated execution

LaValle distinguishes geometric motion planning from planning under
differential constraints [3]. RoboBench's 2D controller uses a simplified
holonomic motion model with perfect position feedback. Its Webots
controller drives a simulated wheeled robot.

Michel describes Webots as an environment for modelling, programming,
and simulating mobile robots with sensors, actuators, and physical
properties [4]. RoboBench uses Webots for a small execution study with
perfect Supervisor pose feedback.

The two RoboBench motion models answer different experimental questions.
The 2D noise results are not evidence of equivalent performance in
Webots or on a physical robot.

## Scope of the present study

RoboBench examines three interacting choices:

1. Grid spacing and its effects on route availability and planning cost.
2. Planning clearance and its effects on execution under movement noise.
3. Selection among equal-distance routes and its relationship to
   controller execution time.

The contribution is an experimental implementation and a documented
set of observations under controlled settings.

This initial background review does not establish a previously
unexplored research gap. A broader review of navigation benchmarks,
uncertainty-aware planning, and controller-aware path selection remains
necessary before making novelty claims.

## References

[1] E. W. Dijkstra, “A note on two problems in connexion with graphs,”
Numerische Mathematik, vol. 1, pp. 269–271, 1959.
https://doi.org/10.1007/BF01386390

[2] P. E. Hart, N. J. Nilsson, and B. Raphael, “A Formal Basis for the
Heuristic Determination of Minimum Cost Paths,” IEEE Transactions on
Systems Science and Cybernetics, vol. 4, no. 2, pp. 100–107, 1968.
https://doi.org/10.1109/TSSC.1968.300136

[3] S. M. LaValle, Planning Algorithms. Cambridge University Press, 2006.
Author-hosted text:
https://msl.cs.uiuc.edu/~lavalle/planning/

[4] O. Michel, “Cyberbotics Ltd. Webots: Professional Mobile Robot
Simulation,” International Journal of Advanced Robotic Systems,
vol. 1, no. 1, 2004.
https://doi.org/10.5772/5618