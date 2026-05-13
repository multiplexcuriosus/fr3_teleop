#!/usr/bin/env python3
import argparse
import os
import shutil
import h5py
import numpy as np


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("input_hdf5")
    p.add_argument("--output", "-o", default=None)
    p.add_argument("--event-path", default="/observations/images/event")

    p.add_argument(
        "--mode",
        choices=[
            "mean",
            "weighted_mean",
            "max",
            "decay_max",
            "leaky_sum",
            "motion_enhanced",
        ],
        default="decay_max",
    )

    p.add_argument("--window-size", type=int, default=5)
    p.add_argument("--decay", type=float, default=0.90)
    p.add_argument("--motion-gain", type=float, default=1.0)
    p.add_argument("--in-place-dataset", action="store_true")
    p.add_argument("--out-event-path", default=None)

    return p.parse_args()


def default_output_path(input_path, mode, window_size, decay):
    root, ext = os.path.splitext(input_path)
    if ext == "":
        ext = ".hdf5"
    return f"{root}_{mode}_M{window_size}_decay_{decay:g}{ext}"


def copy_attrs(src, dst):
    for k, v in src.attrs.items():
        dst.attrs[k] = v


def convert_dtype(out_float, dtype):
    if np.issubdtype(dtype, np.integer):
        info = np.iinfo(dtype)
        return np.clip(out_float, info.min, info.max).astype(dtype)
    return out_float.astype(dtype)


def get_causal_window(src_ds, t, window_size):
    start = max(0, t - window_size + 1)
    return src_ds[start:t + 1].astype(np.float32)


def weighted_mean(frames, decay):
    n = frames.shape[0]
    weights = np.array(
        [decay ** (n - 1 - i) for i in range(n)],
        dtype=np.float32,
    )
    weights = weights.reshape((n,) + (1,) * (frames.ndim - 1))
    return np.sum(frames * weights, axis=0) / np.sum(weights)


def decay_max(frames, decay):
    """
    Pixelwise max over previous frames, but older frames are faded.

    Newest frame weight: 1
    Previous frame: decay
    Previous previous: decay^2
    etc.
    """
    n = frames.shape[0]
    weights = np.array(
        [decay ** (n - 1 - i) for i in range(n)],
        dtype=np.float32,
    )
    weights = weights.reshape((n,) + (1,) * (frames.ndim - 1))
    return np.max(frames * weights, axis=0)


def leaky_sum_update(acc, frame, decay):
    return decay * acc + frame


def motion_enhanced(frames, decay, motion_gain):
    """
    Preserves faint structure via decay_max, then adds a temporal-gradient term.

    This can make motion traces stronger, but it can also amplify noise.
    Try only after max/decay_max.
    """
    base = decay_max(frames, decay)

    if frames.shape[0] < 2:
        return base

    prev = frames[:-1]
    curr = frames[1:]
    diffs = np.abs(curr - prev)

    motion = decay_max(diffs, decay)
    return base + motion_gain * motion


def compute_event_frames(
    src_ds,
    dst_ds,
    mode,
    decay,
    window_size,
    motion_gain,
):
    """
    Assumes time is first axis:
      [T, H, W], [T, C, H, W], or [T, H, W, C]

    Modes:
      mean            : unweighted average over last M frames
      weighted_mean   : weighted average, newer frames stronger
      max             : pixelwise max over last M frames
      decay_max       : pixelwise max over last M frames, older frames faded
      leaky_sum       : persistent accumulator: acc = decay*acc + frame
      motion_enhanced : decay_max + temporal-gradient boost
    """
    T = src_ds.shape[0]
    if T == 0:
        return

    if window_size < 1:
        raise ValueError("--window-size must be >= 1")

    acc = np.zeros(src_ds.shape[1:], dtype=np.float32)

    for t in range(T):
        frame = src_ds[t].astype(np.float32)

        if mode == "leaky_sum":
            acc = leaky_sum_update(acc, frame, decay)
            out_float = acc

        else:
            frames = get_causal_window(src_ds, t, window_size)

            if mode == "mean":
                out_float = np.mean(frames, axis=0)

            elif mode == "weighted_mean":
                out_float = weighted_mean(frames, decay)

            elif mode == "max":
                out_float = np.max(frames, axis=0)

            elif mode == "decay_max":
                out_float = decay_max(frames, decay)

            elif mode == "motion_enhanced":
                out_float = motion_enhanced(frames, decay, motion_gain)

            else:
                raise ValueError(f"Unknown mode: {mode}")

        dst_ds[t] = convert_dtype(out_float, dst_ds.dtype)

        if t % 100 == 0 or t == T - 1:
            print(f"[INFO] processed {t + 1}/{T}")


def main():
    args = parse_args()

    if not (0.0 <= args.decay <= 1.0):
        raise ValueError("--decay must be between 0 and 1")

    input_path = args.input_hdf5
    output_path = args.output or default_output_path(
        input_path,
        args.mode,
        args.window_size,
        args.decay,
    )

    if os.path.abspath(input_path) == os.path.abspath(output_path):
        raise ValueError("Output path must be different from input path")

    out_event_path = (
        args.out_event_path
        or (args.event_path if args.in_place_dataset else args.event_path + "_" + args.mode)
    )

    print(f"[INFO] copying:\n  {input_path}\n-> {output_path}")
    shutil.copy2(input_path, output_path)

    with h5py.File(output_path, "r+") as f:
        if args.event_path not in f:
            raise KeyError(f"Event dataset not found: {args.event_path}")

        src_ds = f[args.event_path]
        print(f"[INFO] source: {args.event_path}")
        print(f"[INFO] shape={src_ds.shape}, dtype={src_ds.dtype}")
        print(
            f"[INFO] mode={args.mode}, "
            f"M={args.window_size}, "
            f"decay={args.decay}, "
            f"motion_gain={args.motion_gain}"
        )

        if args.in_place_dataset:
            dst_ds = src_ds
        else:
            if out_event_path in f:
                print(f"[WARN] deleting existing dataset: {out_event_path}")
                del f[out_event_path]

            dst_ds = f.create_dataset(
                out_event_path,
                shape=src_ds.shape,
                dtype=src_ds.dtype,
                chunks=src_ds.chunks,
                compression=src_ds.compression,
                compression_opts=src_ds.compression_opts,
            )
            copy_attrs(src_ds, dst_ds)

        dst_ds.attrs["temporal_mode"] = args.mode
        dst_ds.attrs["temporal_window_size"] = args.window_size
        dst_ds.attrs["decay"] = args.decay
        dst_ds.attrs["motion_gain"] = args.motion_gain
        dst_ds.attrs["source_dataset"] = args.event_path

        compute_event_frames(
            src_ds=src_ds,
            dst_ds=dst_ds,
            mode=args.mode,
            decay=args.decay,
            window_size=args.window_size,
            motion_gain=args.motion_gain,
        )

    print(f"[DONE] output file: {output_path}")
    print(f"[DONE] output event dataset: {out_event_path}")


if __name__ == "__main__":
    main()