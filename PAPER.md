# RoboBench: A Controlled Simulation Study of Grid Resolution, Planning Clearance, and Waypoint Representation

Author: Shaurya Mahajan

Status: Research-enhanced working draft, 18 October 2026. Results remain
descriptive. Broader validation and independent technical review are needed
before submission. Literature comparisons do not constitute new experiments.

## Abstract

Robot navigation results depend on the search representation, geometric
clearance, and execution controller. This study examines these dependencies
through a reproducible Python framework and controlled Webots experiments.
A-star, Dijkstra, and breadth-first search agreed on route availability
and grid-path distance in the reported comparisons. On 100 generated maps,
10- and 20-pixel grids found routes on 95 maps, compared with 89 at
40 pixels. Resolution retry preserved tested fine-grid reachability on
244 passage cases but did not reduce average planning time. Under
simplified Gaussian displacement noise, increasing planning clearance
reduced collisions while making some cases unplannable. An 81-trial
Webots study found different execution durations despite equal planned
distances within each of nine box placements. A separate six-trial control
showed that adding collinear midpoints to one planned polyline increased
execution time from 83.616 to 123.232 simulation seconds. Follow-up
12- and 36-trial controls recovered original-route timings by removing
redundant points. These observations identify a waypoint-following
implementation effect, not a new planning algorithm or general planner
ranking. They motivate reporting representation, controller settings,
and outcome denominators alongside path length and planning time.

**Keywords:** grid search; robot navigation; benchmarking; obstacle
clearance; waypoint representation; Webots.

## 1. Introduction

Shortest-path search solves a specified graph problem. Executing the
returned path introduces further choices: robot geometry, waypoint
selection, speed scheduling, turning, and stopping criteria. A comparison
that attributes every execution-time difference to the search algorithm
can therefore obscure effects elsewhere in the navigation pipeline.

Established benchmarking research emphasizes shared problem definitions,
multiple performance measures, and inspectable results [5, 6]. RoboBench
applies these principles to a small, transparent simulation study. Its
scope is controlled experimentation rather than a replacement for a
community benchmark suite or a new navigation stack.

The study asks:

1. **Representation:** How does grid spacing affect route availability,
   shortest grid-path distance, and planning computation?
2. **Clearance:** How does an added planning margin affect execution under
   a specified displacement-noise model, including unavailable routes?
3. **Execution:** Can equal-distance routes take different execution times,
   and can redundant waypoint targets introduce delay on the same polyline?

The contributions are an inspectable experimental implementation, archived
results and figure-generation scripts, and controls separating planned
geometry from waypoint sampling. Collinear-point removal is treated as
routine preprocessing, not an algorithmic invention.

All findings are conditional on the implemented models. The 2D uncertain
motion experiment and the Webots wheeled-robot experiments are separate
studies; the latter do not validate the former's noise model.

## 2. Related Work and Study Positioning

### 2.1 Graph search and configuration space

Dijkstra introduced a minimum-length graph-path procedure [1]. Hart,
Nilsson, and Raphael established the heuristic-search framework associated
with A-star [2]. LaValle discusses these methods and breadth-first search
in discrete planning, and develops configuration-space representations
for robot geometry [3, Chapters 2 and 4].

RoboBench uses equal-cost four-connected grids. Its shared A-star/Dijkstra
implementation and separately implemented BFS are baselines under common
assumptions. Their agreement is not an independent proof of correctness.
Rectangular inflation conservatively approximates circular-robot clearance.

### 2.2 Benchmark design and map diversity

Sturtevant introduced shared grid-pathfinding maps and problem sets to
improve comparability and examined how map properties affect evaluation
[5]. This is directly relevant to RoboBench's 20-pixel-aligned generator:
the generated cases cannot establish performance across diverse map
families. The present study has not evaluated the Moving AI datasets.
Their adoption would also require explicit choices about connectivity,
robot radius, map scale, and obstacle interpretation.

Moll, Şucan, and Kavraki developed motion-planning benchmarking infrastructure
with extensible result storage and analysis [6]. Their discussion of
multiple metrics and missing observations motivates reporting failure
counts alongside successful-route distances. RoboBench uses its own CSV
workflow; it neither uses OMPL in the experiments nor claims an equivalent
benchmarking capability.

### 2.3 Tracking and velocity selection

Coulter describes Pure Pursuit tracking through lookahead geometry and
examines parameter-dependent behaviour [7]. Macenski et al. develop
Regulated Pure Pursuit, including velocity regulation and collision-related
mechanisms for service-robot tracking [8]. These provide relevant controller
baselines for future work.

RoboBench instead uses sequential waypoint targets, heading correction,
and a target-distance-dependent forward wheel command. Its observed delay
should not be attributed to Pure Pursuit or navigation controllers in
general. Removing collinear points changes this controller's target list;
it is not a Pure Pursuit implementation or a comparative evaluation of
published tracking methods.

### 2.4 Uncertainty and geometric margins

Blackmore, Ono, and Williams formulate path planning with explicit
probabilistic failure constraints under their system assumptions [9].
RoboBench's clearance sweep is substantially simpler: it selects fixed
geometric margins and records outcomes under a chosen noise process.
It propagates no planned state covariance, solves no chance constraint,
and offers no certified probability-of-collision bound.

### 2.5 Scope relative to prior work

Webots supplies a robot-simulation environment [4]. The role of this study
is to expose interactions among discretization, margins, and a deliberately
simple execution policy using controlled comparisons. It does not establish
a previously unexplored research gap or outperform the cited systems.
The related-work comparison informs interpretation and future validation,
not a claim that external benchmark results are directly comparable.

## 3. Methods

### 3.1 Planning model

The 2D environment measures 600 by 400 pixels. The physical robot is
represented by a circle with radius 12 pixels. Movement is restricted
to horizontal and vertical grid edges.

A-star uses Manhattan distance. Dijkstra uses the same priority-queue
search with a zero heuristic. Breadth-first search uses a separately
implemented queue-based search.

Planning uses conservative rectangular obstacle inflation. Returned
routes are checked for endpoint correctness, valid steps, map boundaries,
and circular robot clearance. Agreement between planners provides an
additional consistency check rather than proof of correctness.

The common 2D start and goal are (60, 200) and (540, 200) pixels. Grid
locations are anchored to the start; comparisons use aligned endpoints.
“No route” refers to this discrete, conservatively inflated problem, not
necessarily to disconnected continuous free space.

### 3.2 Grid resolution and resolution retry

One hundred maps, generated using seeds 0 through 99, were evaluated
at grid spacings of 10, 20, and 40 pixels.

Each planner received two warm-up runs and ten recorded runs per map
and spacing. Execution order was shuffled using a fixed seed. Timing
excluded external route validation and file writing.

A separate experiment evaluated 244 passage layouts. Passage widths
were 20, 30, 40, and 60 pixels, with passage centres at every integer
y-coordinate from 160 through 220.

Fixed 10-pixel A-star was compared with a strategy that tried 40,
20, and 10 pixels, stopping at the first successful search. This is
resolution retry rather than a spatially adaptive grid.

Each method received two warm-up runs and ten recorded runs per case.
Retry timing included the complete planning call, input checks, and
all attempted resolutions. External validation and file writing
were excluded.

### 3.3 Movement uncertainty and planning clearance

A simplified holonomic controller moved towards successive waypoints.
Each step commanded up to two pixels of movement and received independent
Gaussian displacement error on each axis. The controller had perfect
position feedback.

The robot used a one-pixel waypoint tolerance and a 10,000-step limit.
Collision checking covered the entire movement segment. Touching an
obstacle counted as collision.

A-star planned on a 20-pixel grid. The noise experiment used 19 reachable
maps from generated seeds 0 through 19. Seed 10 had no planned route
and was excluded. Each noise setting used the same maps and 20 motion
seeds per map.

Noise standard deviations were 0, 0.25, 0.5, 1, and 2 pixels per axis
per movement step.

The clearance experiment included all 20 maps, including unavailable
routes. Extra planning clearances were 0, 4, 8, and 12 pixels, with
movement noise fixed at sigma 2 pixels per axis per step.

The physical radius remained 12 pixels. Planning used this radius plus
the selected clearance, including at map boundaries. Each setting used
20 motion trials per map, with matched trial seeds across settings.

### 3.4 Webots planner comparisons

An E-puck followed routes around a static box measuring 0.2 metres
on each side. Obstacle footprints were calculated from world geometry.

An initial study tested three box placements once per planner,
producing nine trials. A separate extended study tested nine placements,
three planners, and three repetitions per combination, producing
81 trials.

The extended placements used every combination of box-centre
x-coordinates -0.04, 0, and 0.04 metres and y-coordinates -0.03, 0,
and 0.03 metres. The box-centre z-coordinate was 0.1 metres.

Layout/repetition blocks and planner order within each block were
shuffled using seed 42. Before every trial, the robot was returned
to the nominal starting pose, its physics state was reset, and it
settled with stopped wheels for at least 0.5 simulation seconds.

The planning grid spacing was 20 pixels, with a planning radius of
40 pixels and a mapping of 0.001 metres per pixel. These settings
differ from the 2D motion experiments.

The start and goal were approximately (-0.25, 0) and (0.25, 0) metres.
Collinear grid points were removed before execution, retaining
endpoints and turns. Waypoint tolerance was 0.008 metres.

The controller used perfect Supervisor position and orientation
feedback. No motion noise was deliberately added.

When heading error exceeded 0.15 radians, the robot turned in place.
Otherwise, its nominal forward wheel speed was the smaller of
1.5 radians per second and ten times the remaining waypoint distance
in metres. The distance-to-wheel-speed coefficient is therefore
10 radians per second per metre. Heading correction used gain 1.5, with
wheel speeds limited to 2 radians per second. The initial heading was
zero radians, facing positive x. The simulation step was 32 milliseconds.
Execution timing excluded planning and reset/settling.

Physical box contact was detected by matching contact-point locations
reported for the robot and named box within 0.00001 metres. Two consecutive front-sensor
readings at or above 120 triggered a separate obstacle-stop outcome.
Planner-comparison trials had a 180-second execution limit.

Execution duration was measured in simulation seconds. Moving, turning,
and idle durations were assigned according to the command active during
each simulation step. Moving commands can include steering corrections.

### 3.5 Controlled waypoint experiment

A separate experiment held the planned geometric polyline fixed:

(-0.25, 0) → (-0.15, 0) → (-0.15, -0.16) →
(0.15, -0.16) → (0.15, 0) → (0.25, 0), in metres.

The original representation had five segments. The second inserted one
collinear midpoint into every segment, producing ten segments while
preserving the same 0.820-metre planned polyline.

Each condition was executed three times. Condition order was shuffled
within each repetition using seed 42. Both conditions used the same
controller, nominal starting pose, reset procedure, and waypoint
tolerance. The execution limit was 300 seconds.

### 3.6 Collinear-waypoint cleanup control

A follow-up experiment tested four conditions: the original route and
the midpoint route, each with and without collinear-point cleanup.
Each condition was executed three times, giving 12 trials.

Cleanup retained corners and endpoints. It converted the ten-segment
midpoint route back into the original five-segment route without
changing the planned geometric polyline or its 0.820-metre length.

The controller, nominal starting pose, reset procedure, and waypoint
tolerance were held fixed. Condition order was shuffled within each
repetition using seed 42. No motion noise was deliberately added.

### 3.7 Multi-route waypoint cleanup

The four waypoint conditions were evaluated on three planned polylines:
Bottom, its mirrored Top route, and an Early bottom turn route.
Each planned polyline was 0.820 metres long.

Bottom and Top each had five original segments. Early bottom turn had
four. Each route/condition combination was repeated three times,
producing 36 trials. Route/repetition blocks and condition order within
each block were shuffled using seed 42.

The environment, controller, nominal starting pose, reset procedure,
and waypoint tolerance were held fixed. Early bottom turn followed
start, (-0.25, -0.16), (0.15, -0.16), (0.15, 0), and goal, in metres.
Cleanup controls retained the 300-second execution limit. These were
hand-specified-route tests, not additional planner comparisons.

### 3.8 Outcomes, aggregation, and experimental units

Planning feasibility, physical execution, and computation are separate
outcomes. The clearance study reports success, collision, timeout, and
unavailable-route counts over all scheduled cases. Successful-trial
distance averages are conditional outcomes, not averages over failures.
The common-map clearance comparison restricts analysis to the same
12 maps with routes at every setting.

Grid and retry timings first summarize repeated calls within a map/case
by their median, then average those medians. The extended Webots summary
gives each layout equal weight by averaging its three-repetition median.
Waypoint-condition durations are means of three repetitions. Moving,
turning, and idle categories are defined by issued commands, not by an
independent physical motion classifier.

A repeated timing call is not another map. Likewise, repeated execution
of a route is not an independent environment. These samples are used
descriptively; no pooled significance test treats them as independent
environment draws. Outcome denominators accompany performance summaries,
consistent with the missing-data concerns in benchmarking literature [6].

### 3.9 Mechanistic interpretation of waypoint slowdown

The following is an idealized derivation from the implemented controller,
not a fitted model or an additional experiment. On a straight segment,
assume aligned heading, negligible slip, continuous motion, and equal
wheel commands. If wheel radius is r_w and the forward command is
min(u_max, k_d d), then translational speed is r_w times that command.
Inside the proportional region, remaining distance follows approximately
dd/dt = -alpha d, where alpha = r_w k_d.

The approximate time to reduce distance from d_0 to acceptance tolerance
epsilon is ln(d_0 / epsilon) / alpha. Each added target can restart this
approach phase. This explains why holding planned length fixed need not
hold execution time fixed. The approximation excludes discrete stepping,
heading corrections, target-switching geometry, and wheel dynamics; it
does not predict the reported durations exactly.

## 4. Results

### 4.1 Grid resolution

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

![Route availability and planning time at three grid spacings](figures/01_grid_resolution.png)

**Figure 1.** Grid resolution results on 100 generated maps. Left:
number of maps with a route; all three planners agreed. Right: mean
of per-map median planning times, including no-route cases. Connecting
lines guide the eye between tested spacings.

### 4.2 Coarse-to-fine retry

Both fixed 10-pixel planning and resolution retry found routes in
153 of 244 passage cases.

For reachable cases, mean case-median times were 0.761 ms for fixed
planning and 0.792 ms for retry. For no-route cases, they were 5.104 ms
and 6.880 ms.

Retry returned longer routes in 41 successful cases. Mean extra distance
across all 153 successful cases was 8.0 pixels, with a maximum of 60 pixels.

![Planning times for fixed fine-grid search and resolution retry](figures/02_resolution_retry.png)

**Figure 2.** Mean of per-case median planning times for fixed 10-pixel
A-star and resolution retry. Groups contain 153 cases with routes and
91 without routes. Retry timing includes all attempted resolutions.

### 4.3 Movement uncertainty

At noise sigma values of 0, 0.25, 0.5, and 1 pixel per axis per step,
all 380 trials per setting reached the goal. At sigma 2, 193 of 380
trials succeeded and 187 collided. No timeouts occurred.

Mean travelled distance among successful trials increased from
650.5 pixels at zero noise to 853.4 pixels at sigma 1, approximately
a 31% increase.

At sigma 2, mean successful-trial distance was 1952.3 pixels. This
average excludes failed trials and describes a selected subset.

![Success rate and successful-trial distance under movement noise](figures/03_motion_noise.png)

**Figure 3.** Noise results using 19 planned maps and 20 motion seeds
per map per setting. Left: success rate over 380 trials. Right: mean
travelled distance among successful trials only. The successful subset
changes at sigma 2. Connecting lines guide the eye.

### 4.4 Planning clearance under movement noise

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

![Outcome proportions and matched-map success across planning clearances](figures/04_planning_clearance.png)

**Figure 4.** Clearance results at noise sigma 2. Left: outcome proportions
across 400 cases per setting, including unavailable routes. Right:
success rates on the same 12 maps reachable at every clearance,
with 240 trials per setting. Observed 100% success is not a guarantee
of future reliability.

### 4.5 Initial Webots study

All nine initial trials reached the goal without detected box contact.
Mean execution durations across the three placements were 81.08 seconds
for A-star, 72.14 for Dijkstra, and 69.90 for BFS.

![Initial Webots execution duration by planner and box placement](figures/05_webots_execution.png)

**Figure 5.** The initial nine-trial Webots study. Each bar represents
one trial. Planned distances matched within each placement: 0.820 metres
for Centre and 0.740 metres for Right-up and Left-down. This figure
shows the initial study, not the extended 81-trial experiment.

### 4.6 Extended Webots study

All 81 trials reached the goal without detected box contact or other
recorded failure outcomes. Planned distances matched across all planners
and repetitions within each of the nine layouts.

Each value below is the mean of nine layout medians, where each layout
median summarizes three repetitions.

| Planner | Goals | Moving time | Turning time | Total execution |
|---|---:|---:|---:|---:|
| A-star | 27/27 | 72.341 s | 8.693 s | 81.067 s |
| Dijkstra | 27/27 | 63.324 s | 8.779 s | 72.135 s |
| BFS | 27/27 | 63.314 s | 6.549 s | 69.895 s |

Trials included approximately 0.032 seconds of idle time. Independently
summarized components and rounded values may not sum exactly to the
displayed total.

A-star routes contained five segments and four turning episodes.
Dijkstra routes contained four segments and four turning episodes.
BFS routes contained four segments and three turning episodes.

A-star's largest within-layout execution-time range across repetitions
was 0.032 seconds, equal to one simulation step. Dijkstra and BFS had
no measured within-layout execution-time variation.

Relative to Dijkstra, A-star spent approximately 9.017 seconds longer
in moving-command mode, while turning times were nearly equal.
Dijkstra's approximately 2.240-second total difference from BFS was
mostly associated with approximately 2.229 seconds of additional turning.

### 4.7 Controlled waypoint experiment

All six trials reached the goal without detected box contact.

| Condition | Segments | Mean moving time | Mean turning time | Mean total |
|---|---:|---:|---:|---:|
| Original waypoints | 5 | 74.848 s | 8.736 s | 83.616 s |
| Extra midpoints | 10 | 114.624 s | 8.576 s | 123.232 s |

Both representations described the same 0.820-metre planned polyline.
Each trial included approximately 0.032 seconds of idle time.

Adding midpoints increased total execution time by 39.616 seconds,
approximately 47.4%. Moving-command time increased by 39.776 seconds,
while turning-command time decreased by 0.160 seconds.

All three repetitions within each condition had identical durations
at the displayed precision.

![Execution breakdown for the same route with different waypoint counts](figures/06_waypoint_control.png)

**Figure 6.** Mean execution durations across three repetitions per
condition. Both waypoint representations describe the same 0.820-metre
planned polyline. Inserting collinear midpoints increased total duration
from 83.616 to 123.232 simulation seconds. Most of the increase occurred
during moving commands. All six trials reached the goal without detected
box contact. The approximately 0.032-second idle component is too small
to distinguish at this scale.

### 4.8 Collinear-waypoint cleanup

All 12 trials reached the goal without detected box contact.

| Condition | Collision-free goals | Mean execution | Maximum goal error |
|---|---:|---:|---:|
| Original / unchanged | 3/3 | 83.616 s | 0.007963 m |
| Midpoints / unchanged | 3/3 | 123.232 s | 0.007970 m |
| Original / cleanup | 3/3 | 83.616 s | 0.007962 m |
| Midpoints / cleanup | 3/3 | 83.616 s | 0.007963 m |

Cleanup reduced midpoint-route execution time by 39.616 seconds,
approximately 32.1% relative to the unchanged midpoint condition.
The original route and its execution duration were unchanged at the
displayed precision.

This demonstrates the benefit of removing redundant waypoint slowdown
targets for this route and controller. It does not establish a new
planning algorithm or general performance improvement.

The planner-comparison experiments already removed collinear grid
points. This cleanup therefore does not eliminate their differences
in route geometry or explain all between-planner execution differences.

Archived data: data/webots_waypoint_cleanup.csv.
Full report: WAYPOINT_CLEANUP_RESULTS.md.

### 4.9 Multi-route cleanup results

All 36 trials reached the goal without detected box contact. Maximum
reported goal errors were below 0.008 metres.

| Route | Original unchanged | Midpoints unchanged | Original cleanup | Midpoints cleanup |
|---|---:|---:|---:|---:|
| Bottom | 83.616 s | 123.232 s | 83.616 s | 83.616 s |
| Top | 83.616 s | 123.232 s | 83.616 s | 83.616 s |
| Early bottom turn | 74.368 s | 107.584 s | 74.368 s | 74.368 s |

Values are mean execution durations across three repetitions.

Cleanup reduced midpoint-route execution time by approximately 32.1%
for Bottom and Top, and 30.9% for Early bottom turn. Cleaned midpoint
routes recovered the original waypoint lists and matched their
execution durations at the displayed precision.

The Early bottom turn route remained faster despite equal planned
length. Removing redundant collinear points does not remove differences
in corners, segment lengths, or controller responses.

These results extend the observation to three polylines in one
environment. Top and Bottom are mirror images, so geometric diversity
remains limited. Repetitions are consistency checks, not independent
environments or robustness trials.

Archived data: data/webots_waypoint_routes.csv.
Full report: WAYPOINT_ROUTES_RESULTS.md.

## 5. Discussion

### 5.1 The graph problem changes with resolution

The coarser grid was cheaper to search in this implementation but sometimes
lost routes or lengthened them. Agreement at 10 and 20 pixels on these
maps does not identify a universally optimal spacing. The map generator's
20-pixel alignment and the separate passage cases delimit this finding.
Map diversity is a substantive validation requirement, not something that
can be replaced by more timing repetitions [5].

### 5.2 Margin selection trades route availability against observed safety

Eight pixels of extra clearance produced the highest overall completion
among tested settings. Twelve pixels removed observed collisions but
excluded more routes. Choosing a margin solely from collision counts would
therefore answer a different question from maximizing completed tasks.

The common-map analysis helps separate route availability from execution,
but conditions on maps that remain plannable at every setting. Both the
all-case and common-map denominators are needed. The tested Gaussian
process and fixed margin do not yield the probabilistic guarantees of a
chance-constrained planner [9].

### 5.3 Separate planner cost from controller cost

Equal-distance Webots routes differed in segment structure and execution
duration. These are planner-selected routes under one tracking policy,
not evidence that A-star computation takes longer than BFS computation.
Moving-command time, turn episodes, and target tolerance make the observed
differences more interpretable than total duration alone.

The single-route insertion control held planned geometry fixed. Cleanup
then recovered the original target list and timing; the multi-route control
repeated this observation on three limited polylines. This supports the
specific explanation that extra target-approach slowdowns cause a delay
under the tested policy. The 47.4% insertion increase and 32.1% cleanup
reduction use different reference durations, so they are not inconsistent.

Collinear cleanup is routine preprocessing, not a new tracking algorithm.
The planner-comparison controller already used it. Consequently, cleanup
cannot be presented as a remedy for all between-planner differences.
Existing lookahead and velocity-regulating trackers [7, 8] are relevant
comparisons that this study has not yet implemented.

### 5.4 What the measurements establish

Recorded durations are consistent under the tested reset procedure.
Near-identical repetitions do not estimate behaviour under new maps,
localization error, disturbances, or physical deployment. Bottom and Top
are symmetric, limiting the information gained by treating them as separate
geometries.

Executed distance can be shorter than the planned polyline because of
waypoint tolerance and steering before exact target coordinates. This
does not demonstrate a better shortest path. Successful-trial distance
averages under noise also require their outcome counts, as failed trials
are excluded. Multiple metrics and transparent missing outcomes follow
the evaluation concerns discussed by Moll et al. [6].

## 6. Limitations and Threats to Validity

**Construct validity.** The 2D model is a holonomic point-position controller
with circular collision geometry and independent displacement noise.
It is not a sensor, wheel-slip, or localization model. Webots uses perfect
Supervisor pose feedback with no deliberately added motion noise. Results
from these two models must not be pooled. Contact monitoring covers the
named box, not every possible scene collision.

**Internal validity.** A-star and Dijkstra share code and all planners
share assumptions. Agreement and separate geometric checks cannot rule out
all common errors. Initial pose checks allow small deviations, and pose
reset is not a claim that every simulator random state has been reset.
No additional simulation random seed is recorded. The fixed order seed
controls scheduling, not simulator randomness. Repeated execution may be
nearly deterministic in the tested conditions.

**External validity.** Generated maps align to a 20-pixel grid. Webots
tests use one box, nine closely related placements, one start/goal pair,
and three waypoint polylines, two of which are mirrors. The 81-trial study
contains nine layouts, not 81 independent environments. No standardized
external map suite, additional tracking controller, dynamic obstacles,
or physical robot experiment was evaluated.

**Measurement and inference.** Planning times vary with hardware, system
load, and code revision; simulation durations depend on model and policy.
No confidence intervals or significance tests are reported. A trial-level
binomial interpretation would ignore dependence among trials sharing a
map or route. Zero observed contacts is not a certified safety bound.
Small goal-error differences do not establish greater accuracy.

**Reproducibility completeness.** Individual Webots trials are archived,
but the archived 2D summaries do not retain all raw calls or motion trials.
Historical experiment commits and simulator settings are not yet recorded
in a complete run-by-run manifest. Regenerating figures is weaker evidence
than independently reproducing the simulations.

**Scope of novelty.** The work is a controlled implementation study of
established methods. Broader related work and independent review are
needed before making novelty or publication-readiness claims.

## 7. Reproducibility and Evidence Availability

Repository: https://github.com/ShauryaMahajan16/RoboBench

Archived 2D summaries and individual Webots records are in data/. Figures
are in figures/. The archived summaries do not permit every raw 2D trial
to be independently reanalyzed.

On Windows, after creating the project virtual environment and installing
Matplotlib, regenerate Figures 1–5 and Figure 6 respectively:

```powershell
.\.venv\Scripts\python.exe plot_archived_results.py
.\.venv\Scripts\python.exe plot_waypoint_results.py
```

Regenerate the extended Webots report:

```powershell
.\.venv\Scripts\python.exe analyze_webots_extended.py
```

These commands were successfully exercised from a fresh clone in a separate
environment on the same Windows laptop. That check establishes the plotting
and analysis workflow on that machine; it does not rerun the simulations
or establish cross-platform reproducibility.

| Study | Archived trial data | Detailed report |
|---|---|---|
| Initial nine-trial planner comparison | data/webots_layout_trials.csv | WEBOTS_LAYOUT_RESULTS.md |
| Extended 81-trial planner comparison | data/webots_extended_trials.csv | WEBOTS_EXTENDED_RESULTS.md |
| Six-trial waypoint insertion | data/webots_waypoint_test.csv | WAYPOINT_RESULTS.md |
| Twelve-trial cleanup control | data/webots_waypoint_cleanup.csv | WAYPOINT_CLEANUP_RESULTS.md |
| Thirty-six-trial multi-route control | data/webots_waypoint_routes.csv | WAYPOINT_ROUTES_RESULTS.md |

Current controller names and functions are:

- robobench_layout_trials: 81-trial planner comparison.
- robobench_waypoint_test: 12-trial cleanup control.
- robobench_waypoint_routes: 36-trial multi-route control.

Historical nine- and six-trial controllers are earlier Git revisions.
Reproducing them requires the matching revision, not only the current
filename. Webots controllers append runs with run IDs; completeness and
duplicate condition keys must be checked before aggregation.

REPRODUCING.md records the plotting snapshot and environment. The reported
plotting environment is not assumed to be the historical timing environment
for every experiment. Exact experiment commits, Webots version, world
settings, hardware, and all raw 2D trials should be captured before release
as a fully specified research artifact.

AI assistance was used in code development and manuscript drafting.
Automated checks and recorded results do not replace author understanding,
source verification, or independent technical review.

## 8. Conclusion and Next Validation Steps

Under the tested models, discretization and clearance changed planning
cost, route availability, and observed execution outcomes. Equal planned
distance did not ensure equal execution duration. Holding one polyline
fixed while adding waypoint targets exposed a delay under the implemented
distance-based speed policy; removing redundant points recovered the
original-route timing on three tested polylines.

The practical lesson is to specify the discrete problem and execution
policy when interpreting planner comparisons. These observations do not
establish a new search algorithm, general safety guarantee, or superiority
over published navigation systems.

The highest-priority extensions are: (1) external map families with explicit
connectivity and footprint conventions [5]; (2) comparison with a documented
lookahead tracker on identical polylines, measuring cross-track error and
clearance as well as duration [7, 8]; and (3) paired uncertain-execution
experiments on held-out maps, preserving raw records and summarizing
uncertainty at the map level. These are proposed experiments, not completed
results.

## References

[1] E. W. Dijkstra, “A note on two problems in connexion with graphs,”
Numerische Mathematik, vol. 1, pp. 269–271, 1959.
https://doi.org/10.1007/BF01386390

[2] P. E. Hart, N. J. Nilsson, and B. Raphael, “A Formal Basis for the
Heuristic Determination of Minimum Cost Paths,” IEEE Transactions on
Systems Science and Cybernetics, vol. 4, no. 2, pp. 100–107, 1968.
https://doi.org/10.1109/TSSC.1968.300136

[3] S. M. LaValle, Planning Algorithms. Cambridge University Press, 2006.
Author-hosted edition: https://msl.cs.uiuc.edu/~lavalle/planning/

[4] O. Michel, “Cyberbotics Ltd. Webots: Professional Mobile Robot
Simulation,” International Journal of Advanced Robotic Systems,
vol. 1, no. 1, 2004. https://doi.org/10.5772/5618

[5] N. R. Sturtevant, “Benchmarks for Grid-Based Pathfinding,” IEEE
Transactions on Computational Intelligence and AI in Games, vol. 4,
no. 2, pp. 144–148, 2012.
Author-hosted manuscript: https://www.cs.du.edu/~sturtevant/papers/benchmarks.pdf
Benchmark repository: https://www.movingai.com/benchmarks/grids.html

[6] M. Moll, I. A. Şucan, and L. E. Kavraki, “Benchmarking Motion Planning
Algorithms: An Extensible Infrastructure for Analysis and Visualization,”
IEEE Robotics & Automation Magazine, vol. 22, no. 3, pp. 96–102, 2015.
https://doi.org/10.1109/MRA.2015.2448276

[7] R. C. Coulter, “Implementation of the Pure Pursuit Path Tracking
Algorithm,” Carnegie Mellon University, CMU-RI-TR-92-01, 1992.
https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm

[8] S. Macenski, S. Singh, F. Martin, and J. Gines, “Regulated Pure Pursuit
for Robot Path Tracking,” 2023. Author manuscript, arXiv:2305.20026.
https://arxiv.org/abs/2305.20026

[9] L. Blackmore, M. Ono, and B. C. Williams, “Chance-Constrained Optimal
Path Planning With Obstacles,” IEEE Transactions on Robotics, vol. 27,
no. 6, pp. 1080–1094, 2011. https://doi.org/10.1109/TRO.2011.2161160
Author manuscript: https://hdl.handle.net/1721.1/80914
