import matplotlib.pyplot as plt

import plot_results


def save_final_figure(fig, name):
    move_legend = name in {
        "04_planning_clearance",
        "05_webots_execution",
    }

    if move_legend:
        ax = fig.axes[0]
        handles, labels = ax.get_legend_handles_labels()

        legend = ax.get_legend()
        if legend is not None:
            legend.remove()

        fig.legend(
            handles,
            labels,
            loc="upper center",
            bbox_to_anchor=(0.5, 1.0),
            ncol=len(labels),
            frameon=False,
        )

        fig.tight_layout(rect=(0, 0, 1, 0.88))
    else:
        fig.tight_layout()

    for extension in ("png", "pdf"):
        path = plot_results.FIGURES / f"{name}.{extension}"
        fig.savefig(path, dpi=300, bbox_inches="tight")

    plt.close(fig)
    print(f"Saved: {name}.png and {name}.pdf")


def main():
    # Reuse the original data checks and plotting functions.
    # Replace only the final layout and saving step.
    plot_results.save = save_final_figure
    plot_results.main()


if __name__ == "__main__":
    try:
        main()
    except (ValueError, KeyError) as error:
        raise SystemExit(f"Cannot finish plotting: {error}")