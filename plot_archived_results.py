from pathlib import Path

import finalize_figures
import plot_results


ROOT = Path(__file__).resolve().parent
DATA = ROOT / "data"

REQUIRED_FILES = (
    "resolution_summary.csv",
    "adaptive_summary.csv",
    "motion_summary.csv",
    "clearance_summary.csv",
    "webots_layout_trials.csv",
)


def main():
    missing = [
        filename
        for filename in REQUIRED_FILES
        if not (DATA / filename).is_file()
    ]

    if missing:
        raise ValueError(
            "Missing archived data files: " + ", ".join(missing)
        )

    # Reuse the existing data checks and corrected figure layouts.
    plot_results.RESULTS = DATA

    print(f"Reading archived data from: {DATA}")
    finalize_figures.main()


if __name__ == "__main__":
    try:
        main()
    except (ValueError, KeyError) as error:
        raise SystemExit(f"Cannot finish plotting: {error}")