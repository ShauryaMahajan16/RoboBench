# Reproducing the RoboBench Figures

## Recorded plotting environment

- Operating system: Windows
- Python: 3.14.7
- Matplotlib: 3.11.2
- Recorded repository commit:
  `60fea12cf25cce05f394aa8f78e3a4ba119b0834`

These versions were reported by the environment used to regenerate
the archived-data figures. Historical benchmark timings may have been
collected with different software versions or system conditions.

## 1. Download the project

Open PowerShell in the directory where you want to store the project:

```powershell
git clone https://github.com/ShauryaMahajan16/RoboBench.git
cd RoboBench
```

To use the recorded plotting snapshot:

```powershell
git checkout 60fea12cf25cce05f394aa8f78e3a4ba119b0834
```

This selects a historical commit and puts Git in detached HEAD mode.
It does not delete the repository's branches.

## 2. Create the Python environment

With Python installed, run:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install matplotlib==3.11.2
```

The recorded environment used Python 3.14.7. Compatibility of the plotting
workflow with other Python versions has not been separately verified.

## 3. Regenerate the paper figures

Run:

```powershell
.\.venv\Scripts\python.exe plot_archived_results.py
```

The script reads these archived files:

- data/resolution_summary.csv
- data/adaptive_summary.csv
- data/motion_summary.csv
- data/clearance_summary.csv
- data/webots_layout_trials.csv

It reuses the validation and plotting functions in plot_results.py
and the layout adjustments in finalize_figures.py.

The Webots figure uses the final run ID appearing in the archived CSV.
That run must contain exactly one trial for each of the nine
layout/planner combinations.

## 4. Expected outputs

The figures directory receives PNG and PDF versions of:

1. 01_grid_resolution
2. 02_resolution_retry
3. 03_motion_noise
4. 04_planning_clearance
5. 05_webots_execution

Existing files with these names are overwritten.

The expected archived Webots run is:

`2026-10-12T11:20:51.460891+00:00`

Small rendering differences may occur across environments. Byte-for-byte
identical image or PDF files are not guaranteed.

## 5. Regenerating figures versus rerunning experiments

Generating figures from archived data does not rerun the experiments.

The archived 2D files contain summaries. They support regeneration of
the displayed figures but do not contain every individual timing or
motion trial. The Webots archive contains individual layout trials.

To run the corresponding 2D experiments again:

```powershell
.\.venv\Scripts\python.exe resolution_experiment.py
.\.venv\Scripts\python.exe benchmark_adaptive.py
.\.venv\Scripts\python.exe benchmark_motion.py
.\.venv\Scripts\python.exe benchmark_clearance.py
```

These commands write fresh results to results/. They do not automatically
replace the archived files in data/.

To generate figures from fresh local results:

```powershell
.\.venv\Scripts\python.exe finalize_figures.py
```

This command also requires results/webots_layout_trials.csv. A fresh
clone will not contain that ignored local results file.

Rerunning the 3D experiment requires Webots and the
robobench_layout_trials controller. Webots is not required to plot
its archived trial data.

New timing measurements can differ because of hardware, system load,
software versions, and implementation changes. They should be reported
as a new experiment run rather than substituted silently into the
historical report.