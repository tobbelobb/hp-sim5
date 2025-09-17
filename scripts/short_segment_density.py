#!/usr/bin/env python3
import argparse
import math
import warnings
from pathlib import Path

warnings.filterwarnings(
    "ignore",
    message="Unable to import Axes3D",
    module="matplotlib.projections",
)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Visualize density of very short segments in a G-code file."
    )
    parser.add_argument(
        "gcode",
        type=Path,
        help="Path to the G-code file to inspect.",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.1,
        help="Segment length threshold in mm for counting as 'super short' (default: 0.1).",
    )
    parser.add_argument(
        "--time-limit",
        type=float,
        default=15.0,
        help="Maximum amount of motion time to visualize in seconds (default: 15).",
    )
    parser.add_argument(
        "--start-time",
        type=float,
        default=0.0,
        help="Motion time offset in seconds where the visualization window should start (default: 0).",
    )
    parser.add_argument(
        "--bin-size",
        type=float,
        default=0.1,
        help="Width of the time bins in seconds used when computing densities (default: 0.1).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional path to save the plot instead of showing it interactively.",
    )
    return parser.parse_args()


def iter_segments(gcode_path: Path, window_start: float, time_limit: float):
    pos = {"X": 0.0, "Y": 0.0, "Z": 0.0}
    feed = None
    elapsed = 0.0
    window_end = window_start + time_limit

    with gcode_path.open() as handle:
        for raw_line in handle:
            line = raw_line.split(";", 1)[0].strip()
            if not line or not line.startswith(("G0", "G1", "G00", "G01")):
                continue

            tokens = line.split()
            move_axes = {}
            for token in tokens[1:]:
                if not token:
                    continue
                axis = token[0]
                value = token[1:]
                if axis in pos:
                    try:
                        move_axes[axis] = float(value)
                    except ValueError:
                        continue
                elif axis == "F":
                    try:
                        feed = float(value)
                    except ValueError:
                        continue

            previous = pos.copy()
            for axis, value in move_axes.items():
                pos[axis] = value

            dx = pos["X"] - previous["X"]
            dy = pos["Y"] - previous["Y"]
            dz = pos["Z"] - previous["Z"]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance <= 0:
                continue
            if not feed or feed <= 0:
                continue

            duration = 60.0 * distance / feed
            segment_start = elapsed
            segment_end = elapsed + duration
            elapsed = segment_end

            if segment_end <= window_start:
                continue
            if segment_start >= window_end:
                break

            clipped_start = max(segment_start, window_start)
            clipped_end = min(segment_end, window_end)

            yield {
                "start": clipped_start - window_start,
                "end": clipped_end - window_start,
                "distance": distance,
                "absolute_start": segment_start,
                "absolute_end": segment_end,
            }

            if elapsed >= window_end:
                break


def accumulate_short_segments(segments, threshold, time_limit, bin_size):
    num_bins = max(1, math.ceil(time_limit / bin_size))
    counts = [0] * num_bins
    total_segments = 0
    short_segments = 0

    for seg in segments:
        start = seg["start"]
        if start >= time_limit:
            break
        end = min(seg["end"], time_limit)
        midpoint = 0.5 * (start + end)
        total_segments += 1
        if seg["distance"] <= threshold:
            short_segments += 1
            if midpoint <= time_limit:
                idx = min(int(midpoint / bin_size), num_bins - 1)
                counts[idx] += 1

    return counts, total_segments, short_segments


def plot_density(counts, bin_size, output_path, window_start):
    import matplotlib.pyplot as plt

    times = [((i + 0.5) * bin_size) for i in range(len(counts))]
    densities = [c / bin_size for c in counts]

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(times, densities, marker="o", linewidth=1.2)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(f"Short segments per {bin_size:.2f} s")
    ax.set_title(
        "Density of very short segments over time\n"
        f"Window start: {window_start:.2f} s"
    )
    ax.grid(True, linestyle="--", alpha=0.4)
    fig.tight_layout()

    if output_path:
        fig.savefig(output_path)
    else:
        plt.show()


def main():
    args = parse_arguments()
    segments = list(iter_segments(args.gcode, args.start_time, args.time_limit))
    counts, total_segments, short_segments = accumulate_short_segments(
        segments, args.threshold, args.time_limit, args.bin_size
    )

    window_end = args.start_time + args.time_limit
    print(
        f"Processed {total_segments} segments between {args.start_time:.2f} s and "
        f"{window_end:.2f} s; {short_segments} of them are shorter than {args.threshold} mm."
    )

    plot_density(counts, args.bin_size, args.output, args.start_time)


if __name__ == "__main__":
    main()
