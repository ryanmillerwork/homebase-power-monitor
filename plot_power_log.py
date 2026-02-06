#!/usr/bin/env python3
import argparse
import csv
import datetime as dt
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import os


DEFAULT_LOG_PATH = Path(
    "/mnt/analysis/data/ngage/calibration_and_logging/HB5power.log"
)


def parse_rows(path: Path):
    times = []
    data = {
        "v": [],
        "a": [],
        "w": [],
        "pct": [],
        "min_v": [],
        "max_v": [],
        "hrs_remaining": [],
    }
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                ts = dt.datetime.fromisoformat(row["timestamp"])
                values = {key: float(row[key]) for key in data}
            except (KeyError, ValueError):
                continue

            times.append(ts)
            for key, value in values.items():
                data[key].append(value)
    return times, data


def plot_log(times, data, output_path: Optional[Path]):
    if not times:
        raise SystemExit("No valid rows found in log file.")

    t0 = times[0]
    hours = [(t - t0).total_seconds() / 3600.0 for t in times]
    min_v = data["min_v"][0]
    max_v = data["max_v"][0]
    knee_v = 24.0
    tail_fraction = 0.1

    def alt_hours_remaining(voltages, capacity_hours):
        if max_v <= min_v or knee_v <= min_v or max_v <= knee_v:
            pct = [
                max(0.0, min(1.0, (v - min_v) / (max_v - min_v)))
                for v in voltages
            ]
        else:
            pct = []
            for v in voltages:
                if v >= knee_v:
                    head = (v - knee_v) / (max_v - knee_v)
                    pct.append(tail_fraction + (1.0 - tail_fraction) * head)
                else:
                    tail = (v - min_v) / (knee_v - min_v)
                    pct.append(tail_fraction * max(0.0, tail))
        return [p * capacity_hours for p in pct]

    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    fig.suptitle("HB5 Power Log")

    axes = axes.ravel()
    axes[0].plot(hours, data["v"], label="V")
    axes[0].set_ylabel("Voltage (V)")

    axes[1].plot(hours, data["a"], label="A", color="tab:orange")
    axes[1].set_ylabel("Current (A)")

    axes[2].plot(hours, data["w"], label="W", color="tab:green")
    axes[2].set_ylabel("Power (W)")

    axes[3].plot(hours, data["pct"], label="Pct", color="tab:red")
    axes[3].set_ylabel("Charge (%)")

    end_time = times[-1]
    actual_remaining = [
        max(0.0, (end_time - t).total_seconds() / 3600.0) for t in times
    ]
    alt_remaining = alt_hours_remaining(data["v"], 12.0)
    axes[4].plot(
        hours,
        data["hrs_remaining"],
        label="Predicted Hours",
        color="tab:brown",
    )
    axes[4].plot(
        hours,
        alt_remaining,
        label="Alt Predicted Hours",
        color="tab:olive",
    )
    axes[4].plot(
        hours,
        actual_remaining,
        label="Actual Hours",
        color="tab:gray",
    )
    axes[4].set_ylabel("Hours Remaining")
    axes[4].legend(loc="best")

    axes[5].plot(hours, data["pct"], label="Pct", color="tab:red")
    axes[5].set_ylabel("Charge (%)")

    for ax in axes:
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Elapsed Hours")

    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    if output_path:
        fig.savefig(output_path, dpi=150)
        return

    # If no output provided and no GUI display is available, save a default.
    if not os.environ.get("DISPLAY"):
        default_path = Path.cwd() / "hb5_power_plot.png"
        fig.savefig(default_path, dpi=150)
        print(f"Saved plot to {default_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Plot HB5 power log CSV with matplotlib."
    )
    parser.add_argument(
        "log_path",
        nargs="?",
        default=str(DEFAULT_LOG_PATH),
        help="Path to HB5power.log CSV",
    )
    parser.add_argument(
        "--output",
        "-o",
        help="Optional path to save the plot image instead of showing it.",
    )
    args = parser.parse_args()

    log_path = Path(args.log_path)
    if not log_path.exists():
        raise SystemExit(f"Log file not found: {log_path}")

    times, data = parse_rows(log_path)
    output_path = Path(args.output) if args.output else None
    plot_log(times, data, output_path)


if __name__ == "__main__":
    main()
