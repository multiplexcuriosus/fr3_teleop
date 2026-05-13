#!/usr/bin/env python3
import argparse
from pathlib import Path

import h5py
import numpy as np


def stat_array(name, arr, max_print=5):
    n = len(arr)
    print(f"\n{name}")
    print(f"  shape: {arr.shape}")
    print(f"  dtype: {arr.dtype}")

    if n == 0:
        print("  EMPTY")
        return

    head = arr[:max_print]
    tail = arr[-max_print:] if n > max_print else arr[:]

    print(f"  min/max: {np.min(arr)} / {np.max(arr)}")
    print(f"  first:   {head}")
    print(f"  last:    {tail}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("h5_path", help="Path to raw_events.h5")
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--height", type=int, default=320)
    args = ap.parse_args()

    path = Path(args.h5_path)
    if not path.exists():
        raise FileNotFoundError(path)

    with h5py.File(path, "r") as f:
        print(f"[INFO] file: {path}")
        print("\n[ATTRS]")
        for k, v in f.attrs.items():
            print(f"  {k}: {v}")

        required = [
            "/events/type",
            "/events/x",
            "/events/y",
            "/events/t_us",
            "/events/packet_id",
            "/packets/ros_t_ns",
            "/packets/monotonic_t_ns",
            "/packets/start_event_idx",
            "/packets/end_event_idx",
            "/packets/event_count",
        ]

        print("\n[DATASETS]")
        missing = []
        for name in required:
            if name in f:
                ds = f[name]
                print(f"  {name}: shape={ds.shape}, dtype={ds.dtype}")
            else:
                print(f"  MISSING: {name}")
                missing.append(name)

        if missing:
            print("\n[FAIL] Missing required datasets.")
            return

        event_type = f["/events/type"][:]
        x = f["/events/x"][:]
        y = f["/events/y"][:]
        t_us = f["/events/t_us"][:]
        packet_id = f["/events/packet_id"][:]

        ros_t_ns = f["/packets/ros_t_ns"][:]
        start_idx = f["/packets/start_event_idx"][:]
        end_idx = f["/packets/end_event_idx"][:]
        event_count = f["/packets/event_count"][:]

        n_events = len(x)
        n_packets = len(ros_t_ns)

        print("\n[SUMMARY]")
        print(f"  events:  {n_events}")
        print(f"  packets: {n_packets}")

        if n_events == 0:
            print("  [WARN] no events stored")
            return

        duration_us = int(t_us[-1]) - int(t_us[0])
        duration_s = duration_us / 1e6
        rate = n_events / duration_s if duration_s > 0 else float("nan")

        print(f"  event time first/last: {int(t_us[0])} / {int(t_us[-1])} us")
        print(f"  event duration:        {duration_s:.3f} s")
        print(f"  avg event rate:        {rate:.1f} events/s")

        ros_duration_s = (int(ros_t_ns[-1]) - int(ros_t_ns[0])) / 1e9 if n_packets > 1 else 0.0
        print(f"  packet ROS duration:   {ros_duration_s:.3f} s")

        print("\n[BASIC STATS]")
        stat_array("/events/type", event_type)
        stat_array("/events/x", x)
        stat_array("/events/y", y)
        stat_array("/events/t_us", t_us)
        stat_array("/events/packet_id", packet_id)
        stat_array("/packets/event_count", event_count)

        print("\n[VALIDITY CHECKS]")
        ok = True

        lengths = {len(event_type), len(x), len(y), len(t_us), len(packet_id)}
        if len(lengths) != 1:
            print(f"  [FAIL] event dataset lengths differ: {lengths}")
            ok = False
        else:
            print("  [OK] event dataset lengths match")

        packet_lengths = {len(ros_t_ns), len(start_idx), len(end_idx), len(event_count)}
        if len(packet_lengths) != 1:
            print(f"  [FAIL] packet dataset lengths differ: {packet_lengths}")
            ok = False
        else:
            print("  [OK] packet dataset lengths match")

        bad_x = np.sum((x < 0) | (x >= args.width))
        bad_y = np.sum((y < 0) | (y >= args.height))
        if bad_x or bad_y:
            print(f"  [FAIL] invalid coordinates: bad_x={bad_x}, bad_y={bad_y}")
            ok = False
        else:
            print("  [OK] coordinates within bounds")

        unique_types = np.unique(event_type)
        print(f"  event types: {unique_types}")
        if not np.all(np.isin(unique_types, [0, 1])):
            print("  [WARN] event types contain values other than 0/1")

        dt = np.diff(t_us.astype(np.int64))
        neg_dt = np.sum(dt < 0)
        zero_dt = np.sum(dt == 0)
        if neg_dt:
            print(f"  [WARN] event timestamps are not monotonic: negative dt count={neg_dt}")
        else:
            print("  [OK] event timestamps monotonic nondecreasing")

        print(f"  zero timestamp gaps: {zero_dt}")

        if start_idx[0] != 0:
            print(f"  [FAIL] first packet start_idx is {start_idx[0]}, expected 0")
            ok = False
        else:
            print("  [OK] first packet starts at event 0")

        if end_idx[-1] != n_events:
            print(f"  [FAIL] last packet end_idx is {end_idx[-1]}, expected {n_events}")
            ok = False
        else:
            print("  [OK] last packet ends at total event count")

        if not np.all(end_idx >= start_idx):
            print("  [FAIL] some packets have end_idx < start_idx")
            ok = False
        else:
            print("  [OK] packet start/end ordering valid")

        if not np.all((end_idx - start_idx) == event_count):
            mismatch = np.where((end_idx - start_idx) != event_count)[0][:10]
            print(f"  [FAIL] packet event_count mismatch at packet indices: {mismatch}")
            ok = False
        else:
            print("  [OK] packet event_count matches index ranges")

        if "/packets/first_event_t_us" in f and "/packets/last_event_t_us" in f:
            first_t = f["/packets/first_event_t_us"][:]
            last_t = f["/packets/last_event_t_us"][:]

            nonempty = event_count > 0
            if np.any(nonempty):
                reconstructed_first = t_us[start_idx[nonempty]]
                reconstructed_last = t_us[end_idx[nonempty] - 1]

                if not np.all(first_t[nonempty] == reconstructed_first):
                    print("  [FAIL] first_event_t_us does not match event ranges")
                    ok = False
                else:
                    print("  [OK] first_event_t_us matches event ranges")

                if not np.all(last_t[nonempty] == reconstructed_last):
                    print("  [FAIL] last_event_t_us does not match event ranges")
                    ok = False
                else:
                    print("  [OK] last_event_t_us matches event ranges")

        print("\n[RESULT]")
        print("  OK" if ok else "  HAS ERRORS/WARNINGS")


if __name__ == "__main__":
    main()