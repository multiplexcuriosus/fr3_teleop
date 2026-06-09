#!/usr/bin/env python3

import argparse

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def find_zero_crossings(t, y):
    """
    Return interpolated zero-crossing times.
    Ignores exact repeated zeros except sign transitions around them.
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)

    crossings = []

    for i in range(len(y) - 1):
        y0, y1 = y[i], y[i + 1]
        t0, t1 = t[i], t[i + 1]

        if y0 == 0.0:
            crossings.append(t0)
        elif y0 * y1 < 0.0:
            # Linear interpolation.
            alpha = -y0 / (y1 - y0)
            crossings.append(t0 + alpha * (t1 - t0))

    # Deduplicate very close crossings.
    if not crossings:
        return np.array([])

    crossings = np.array(crossings)
    keep = [crossings[0]]
    for c in crossings[1:]:
        if abs(c - keep[-1]) > 1e-6:
            keep.append(c)

    return np.array(keep)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument(
        "--fields",
        nargs="*",
        default=None,
        help="Optional field names to plot, e.g. twist.linear.x twist.linear.y",
    )
    parser.add_argument(
        "--topic",
        default=None,
        help="Optional topic filter, e.g. /cartesian_cmd/twist",
    )
    parser.add_argument(
        "--separate",
        action="store_true",
        help="Plot each field in a separate figure.",
    )
    parser.add_argument(
        "--zero-crossings",
        action="store_true",
        help="Print zero-crossing times and periods.",
    )
    parser.add_argument(
        "--relative-time",
        action="store_true",
        default=True,
        help="Use time relative to first sample.",
    )

    args = parser.parse_args()

    df = pd.read_csv(args.csv_path)

    if args.topic is not None:
        df = df[df["topic"] == args.topic]

    if args.fields is not None:
        df = df[df["field"].isin(args.fields)]

    if df.empty:
        raise RuntimeError("No data left after filtering.")

    # Use first timestamp as t=0.
    t0 = df["ros_time_s"].min()
    df["t"] = df["ros_time_s"] - t0

    series_keys = list(df[["topic", "field"]].drop_duplicates().itertuples(index=False, name=None))

    print("\nAvailable series:")
    for topic, field in series_keys:
        print(f"  {topic}:{field}")

    if args.zero_crossings:
        print("\nZero-crossing analysis:")

    if args.separate:
        for topic, field in series_keys:
            sub = df[(df["topic"] == topic) & (df["field"] == field)].sort_values("t")
            t = sub["t"].to_numpy()
            y = sub["value"].to_numpy()

            plt.figure()
            plt.plot(t, y, marker=".", linewidth=1)
            plt.axhline(0.0, linewidth=1)
            plt.grid(True, which="both")
            plt.minorticks_on()
            plt.xlabel("time [s]")
            plt.ylabel(field)
            plt.title(f"{topic}:{field}")

            if args.zero_crossings:
                crossings = find_zero_crossings(t, y)
                periods = np.diff(crossings)

                for c in crossings:
                    plt.axvline(c, linestyle="--", linewidth=0.8)

                print(f"\n{topic}:{field}")
                print(f"  crossings [s]: {np.round(crossings, 6)}")
                if len(periods) > 0:
                    print(f"  periods [s]:   {np.round(periods, 6)}")
                    print(f"  mean period:   {periods.mean():.6f} s")
                    print(f"  median period: {np.median(periods):.6f} s")

    else:
        plt.figure()

        for topic, field in series_keys:
            sub = df[(df["topic"] == topic) & (df["field"] == field)].sort_values("t")
            t = sub["t"].to_numpy()
            y = sub["value"].to_numpy()

            label = f"{topic}:{field}"
            plt.plot(t, y, marker=".", linewidth=1, label=label)

            if args.zero_crossings:
                crossings = find_zero_crossings(t, y)
                periods = np.diff(crossings)

                print(f"\n{topic}:{field}")
                print(f"  crossings [s]: {np.round(crossings, 6)}")
                if len(periods) > 0:
                    print(f"  periods [s]:   {np.round(periods, 6)}")
                    print(f"  mean period:   {periods.mean():.6f} s")
                    print(f"  median period: {np.median(periods):.6f} s")

        plt.axhline(0.0, linewidth=1)
        plt.grid(True, which="both")
        plt.minorticks_on()
        plt.xlabel("time [s]")
        plt.ylabel("value")
        plt.legend()
        plt.title("Logged topic fields")

    plt.show()


if __name__ == "__main__":
    main()