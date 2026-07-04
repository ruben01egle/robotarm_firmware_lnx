#!/usr/bin/env python3
"""
Analyze and plot transport timing logs produced by dump_timing_log().

Usage:
    python3 analyze_timing.py transport_timing_20260704_143022.csv [more.csv ...]

If multiple files are given, each is analyzed separately and then overlaid
in the comparison plots (useful for e.g. 1000Hz vs 1100Hz runs).
"""

import sys
import argparse
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt

STAGE_COLS = ["encode_us", "write_syscall_us", "roundtrip_us", "drain_us"]

# Anything above this is not a real measurement for a sub-millisecond RT
# transport cycle - it's corrupted data (e.g. a timespec-diff underflow
# wrapping a uint64_t). Rows like this get pulled out before any stats or
# plots are computed, rather than being allowed to silently dominate the
# mean/max. 1 second is generous on purpose: real cycles here are expected
# in the hundreds-of-microseconds to low-milliseconds range.
GARBAGE_THRESHOLD_US = 1_000_000


def load(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df["timed_out"] = df["timed_out"].astype(bool)
    df["reply_mismatch"] = df["expected_replies"] != df["received_replies"]

    is_garbage = (df[STAGE_COLS + ["total_us"]] > GARBAGE_THRESHOLD_US).any(axis=1)
    n_garbage = int(is_garbage.sum())
    if n_garbage:
        print(f"WARNING: {n_garbage} row(s) exceed {GARBAGE_THRESHOLD_US}us in at least one "
              f"stage - treating as corrupted measurements, not real latency. "
              f"This points to a bug in the timing capture itself (e.g. a timespec "
              f"diff underflow), not an actual multi-second stall. Excluding from "
              f"stats/plots below; see garbage rows printed separately.")
        print(df[is_garbage].to_string())

    return df[~is_garbage].reset_index(drop=True), n_garbage


def print_summary(name: str, df: pd.DataFrame, n_garbage: int = 0) -> None:
    n = len(df)
    if n == 0:
        print(f"\n=== {name} ===")
        if n_garbage:
            print(f"all {n_garbage} row(s) were corrupted/garbage - nothing left to analyze.")
        else:
            print("no samples in this file (0 rows) - skipping analysis. "
                  "Check that set_timing_enabled(true) ran before this dump, "
                  "and that reset_timing_log() wasn't called right before it.")
        return

    n_timeout = int(df["timed_out"].sum())
    n_mismatch = int(df["reply_mismatch"].sum())

    print(f"\n=== {name} ===")
    if n_garbage:
        print(f"({n_garbage} garbage row(s) excluded - see warning above)")
    print(f"samples: {n}   timeouts: {n_timeout} ({100*n_timeout/n:.2f}%)   "
          f"reply mismatches: {n_mismatch} ({100*n_mismatch/n:.2f}%)")

    cols = STAGE_COLS + ["total_us"]
    stats = df[cols].agg(["min", "mean", "median",
                           lambda s: s.quantile(0.99), "max"])
    stats.index = ["min", "mean", "median", "p99", "max"]
    print(stats.round(1).to_string())

    if n_timeout > 0:
        print(f"\ntimed-out rows (first 5):")
        print(df[df["timed_out"]].head())


def plot_file(name: str, df: pd.DataFrame, out_dir: Path) -> None:
    if len(df) == 0:
        return

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(f"Transport timing — {name}", fontsize=13)

    # 1. Per-cycle timeline, stacked stages
    ax = axes[0, 0]
    ax.stackplot(df.index, [df[c] for c in STAGE_COLS], labels=STAGE_COLS, alpha=0.8)
    timeouts = df.index[df["timed_out"]]
    if len(timeouts):
        ax.scatter(timeouts, df.loc[timeouts, "total_us"],
                   color="red", marker="x", zorder=5, label="timed out")
    ax.set_title("Per-cycle breakdown (stacked)")
    ax.set_xlabel("sample #")
    ax.set_ylabel("microseconds")
    ax.legend(loc="upper right", fontsize=8)

    # 2. Distribution of total cycle time
    ax = axes[0, 1]
    ax.hist(df["total_us"], bins=40, color="steelblue", edgecolor="black", alpha=0.8)
    for q, style in [(0.99, "r--"), (0.5, "g--")]:
        val = df["total_us"].quantile(q)
        ax.axvline(val, linestyle=style[1:], color=style[0],
                   label=f"p{int(q*100)}={val:.0f}us")
    ax.set_title("Total cycle time distribution")
    ax.set_xlabel("microseconds")
    ax.set_ylabel("count")
    ax.legend(fontsize=8)

    # 3. Boxplot comparing stage contributions
    ax = axes[1, 0]
    ax.boxplot([df[c] for c in STAGE_COLS], tick_labels=STAGE_COLS, showfliers=True)
    ax.set_title("Stage time spread (boxplot, outliers shown)")
    ax.set_ylabel("microseconds")
    ax.tick_params(axis="x", rotation=20)

    # 4. Roundtrip time over the run (the USB/firmware/bus black box)
    ax = axes[1, 1]
    ax.plot(df.index, df["roundtrip_us"], color="darkorange", linewidth=0.8)
    ax.axhline(df["roundtrip_us"].quantile(0.99), color="red", linestyle="--",
               linewidth=1, label=f"p99={df['roundtrip_us'].quantile(0.99):.0f}us")
    ax.set_title("Roundtrip latency over time\n(syscall return -> first reply byte)")
    ax.set_xlabel("sample #")
    ax.set_ylabel("microseconds")
    ax.legend(fontsize=8)

    fig.tight_layout()
    out_path = out_dir / f"{name}_timing.png"
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"saved plot: {out_path}")


def plot_comparison(dfs: dict[str, pd.DataFrame], out_dir: Path) -> None:
    dfs = {name: df for name, df in dfs.items() if len(df) > 0}
    if len(dfs) < 2:
        return

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("Run comparison", fontsize=13)

    # Overlaid total_us histograms
    ax = axes[0]
    for name, df in dfs.items():
        ax.hist(df["total_us"], bins=40, alpha=0.5, label=name, density=True)
    ax.set_title("Total cycle time distribution")
    ax.set_xlabel("microseconds")
    ax.set_ylabel("density")
    ax.legend(fontsize=8)

    # Stage-by-stage mean/p99 grouped bars
    ax = axes[1]
    means = pd.DataFrame({name: df[STAGE_COLS].mean() for name, df in dfs.items()})
    p99s = pd.DataFrame({name: df[STAGE_COLS].quantile(0.99) for name, df in dfs.items()})
    x = range(len(STAGE_COLS))
    width = 0.8 / len(dfs)
    for i, name in enumerate(dfs):
        offset = (i - (len(dfs) - 1) / 2) * width
        ax.bar([xi + offset for xi in x], means[name], width, alpha=0.6, label=f"{name} mean")
        ax.scatter([xi + offset for xi in x], p99s[name], color="black", marker="_", s=200, zorder=5)
    ax.set_xticks(list(x))
    ax.set_xticklabels(STAGE_COLS, rotation=20)
    ax.set_ylabel("microseconds")
    ax.set_title("Mean per stage (bars) vs p99 (black ticks)")
    ax.legend(fontsize=7)

    fig.tight_layout()
    out_path = out_dir / "comparison_timing.png"
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"saved plot: {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_files", nargs="+", type=Path)
    parser.add_argument("-o", "--out-dir", type=Path, default=Path("."),
                         help="directory to write plots into (default: cwd)")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)

    dfs = {}
    for path in args.csv_files:
        if not path.exists():
            print(f"skipping missing file: {path}", file=sys.stderr)
            continue
        name = path.stem
        df, n_garbage = load(path)
        dfs[name] = df
        print_summary(name, df, n_garbage)
        plot_file(name, df, args.out_dir)

    plot_comparison(dfs, args.out_dir)


if __name__ == "__main__":
    main()