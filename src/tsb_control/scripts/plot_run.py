#!/usr/bin/env python3
"""
Turn a recorded run into figures and metrics.

Usage: ros2 run tsb_control plot_run.py <bag> <output dir> [--title TEXT]

Produces <output dir>/<bag name>_map.png (the trajectory against the path),
_response.png (surge and heading against their references) and .json (metrics).
"""

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use('Agg')

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
import rosbag2_py  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

REFERENCE = '#898989'   # neutral, so that the measured signal carries the colour
MEASURED = '#2a78d6'
COMMAND = '#eb6834'
INK = '#333333'
MUTED = '#6b6b6b'
GRID = '#e6e6e6'


def read_bag(path):
    """Return {topic: [(time s, message), ...]} for every topic in the bag."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    topics = {name: [] for name in types}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        topics[topic].append((stamp * 1e-9, deserialize_message(data, types[topic])))
    return topics


def style(axis):
    """Apply a recessive grid and axes."""
    axis.grid(True, color=GRID, linewidth=0.8)
    axis.set_axisbelow(True)
    for side in ('top', 'right'):
        axis.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        axis.spines[side].set_color(GRID)
    axis.tick_params(colors=MUTED, labelsize=9, length=0)
    axis.xaxis.label.set_color(MUTED)
    axis.yaxis.label.set_color(MUTED)


def plot_map(topics, output, title):
    """Draw the trajectory over the commanded path."""
    state = [m for _, m in topics['/state'] if m.valid]
    x = np.array([m.x for m in state])
    y = np.array([m.y for m in state])

    figure, axis = plt.subplots(figsize=(6.4, 5.4), dpi=150)
    style(axis)
    paths = topics.get('/path_following/path', [])
    if paths:
        path = paths[-1][1].poses
        px = [p.pose.position.x for p in path]
        py = [p.pose.position.y for p in path]
        axis.plot(px, py, color=REFERENCE, linewidth=2, linestyle='--', label='Path')
        axis.scatter(
            px, py, s=64, facecolor='white', edgecolor=REFERENCE, linewidth=2, zorder=3,
            label='Waypoints')
    axis.plot(x, y, color=MEASURED, linewidth=2, label='Robot')
    axis.set_aspect('equal', adjustable='datalim')
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
    axis.set_title(title, color=INK, fontsize=12, loc='left')
    axis.legend(frameon=False, labelcolor=INK, fontsize=9)
    figure.tight_layout()
    figure.savefig(output)
    plt.close(figure)


def plot_response(topics, output, title):
    """Draw surge and heading against their references, in two panels."""
    debug = topics['/pid/debug']
    t0 = debug[0][0]
    t = np.array([s - t0 for s, _ in debug])

    # The error is wrapped, so the reference is rebuilt next to the measurement:
    # that way a step across +-180 degrees is drawn as the short turn it is.
    measured = np.degrees(np.unwrap([m.heading.measurement for _, m in debug]))
    reference = measured + np.degrees([m.heading.error for _, m in debug])
    heading_changes = np.ptp(reference) > 1.0

    panels = 2 if heading_changes else 1
    figure, axes = plt.subplots(
        panels, 1, figsize=(8, 2.7 * panels + 0.6), dpi=150, sharex=True, squeeze=False)
    axes = axes[:, 0]
    for axis in axes:
        style(axis)

    axes[0].plot(
        t, [m.surge.reference for _, m in debug], color=REFERENCE, linewidth=2,
        linestyle='--', label='Reference')
    axes[0].plot(
        t, [m.surge.output for _, m in debug], color=COMMAND, linewidth=1.5, label='Command')
    axes[0].plot(
        t, [m.surge.measurement for _, m in debug], color=MEASURED, linewidth=2,
        label='Measured')
    axes[0].set_ylabel('Surge [m/s]')
    axes[0].legend(frameon=False, labelcolor=INK, fontsize=9, ncol=3, loc='upper right')
    axes[0].set_title(title, color=INK, fontsize=12, loc='left')

    if heading_changes:
        axes[1].plot(
            t, reference, color=REFERENCE, linewidth=2, linestyle='--', label='Reference')
        axes[1].plot(t, measured, color=MEASURED, linewidth=2, label='Measured')
        axes[1].set_ylabel('Heading, unwrapped [deg]')
        axes[1].legend(frameon=False, labelcolor=INK, fontsize=9, ncol=2, loc='upper right')
    axes[-1].set_xlabel('Time [s]')

    figure.tight_layout()
    figure.savefig(output)
    plt.close(figure)


def jumps_in(reference, step):
    """Return the indices where a reference changes by more than `step` between samples."""
    return [i for i in range(1, len(reference)) if abs(reference[i] - reference[i - 1]) > step]


def settling_times(t, reference, measured, jumps, boundaries, band):
    """
    Seconds each step in `jumps` takes to enter and stay within a band, or None if it never does.

    A window ends at the next entry of `boundaries`, the steps of both loops: a surge
    dip caused by a later heading step says nothing about how the surge step settled.
    """
    times = []
    for start in jumps:
        end = next((b for b in boundaries if b > start), len(t))
        outside = np.nonzero(np.abs(measured[start:end] - reference[start:end]) > band)[0]
        if outside.size == 0:
            times.append(0.0)
        elif outside[-1] == end - start - 1:
            times.append(None)
        else:
            times.append(round(float(t[start + outside[-1] + 1] - t[start]), 2))
    return times


def metrics(topics):
    """Summarise a run as numbers that can be compared between runs."""
    stamped = topics['/pid/debug']
    debug = [m for _, m in stamped]
    t = np.array([s for s, _ in stamped])
    heading = np.degrees(np.unwrap([m.heading.measurement for m in debug]))
    heading_reference = heading + np.degrees([m.heading.error for m in debug])
    surge_reference = np.array([m.surge.reference for m in debug])
    surge = np.array([m.surge.measurement for m in debug])
    surge_jumps = jumps_in(surge_reference, 0.02)
    heading_jumps = jumps_in(heading_reference, 5.0)
    boundaries = sorted(surge_jumps + heading_jumps)
    result = {
        'samples': len(debug),
        'surge_settling_s': settling_times(
            t, surge_reference, surge, surge_jumps, boundaries, 0.015),
        'heading_settling_s': settling_times(
            t, heading_reference, heading, heading_jumps, boundaries, 2.0),
        'surge_saturated_fraction': float(np.mean([m.surge.saturated for m in debug])),
        'heading_saturated_fraction': float(np.mean([m.heading.saturated for m in debug])),
        'peak_abs_integral_surge': float(max(abs(m.surge.i) for m in debug)),
    }
    status = [m for _, m in topics.get('/path_following/status', [])]
    if status:
        # Corners are transients by design, so report the straight parts on their own.
        straight = [
            s.cross_track_error for s in status
            if not s.finished and s.distance_to_waypoint > 0.5]
        state = [m for _, m in topics['/state'] if m.valid][-1]
        path = topics['/path_following/path'][-1][1].poses[-1].pose.position
        result.update({
            'finished': status[-1].finished,
            'cross_track_rms_m': float(np.sqrt(np.mean(np.square(straight)))),
            'cross_track_max_m': float(np.max(np.abs(straight))),
            'final_distance_to_goal_m': math.hypot(state.x - path.x, state.y - path.y),
        })
    return result


def main():
    """Parse arguments and write the outputs."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--title', default='')
    arguments = parser.parse_args()

    arguments.output.mkdir(parents=True, exist_ok=True)
    name = arguments.bag.name
    title = arguments.title or name
    topics = read_bag(arguments.bag)

    if '/path_following/path' in topics and topics['/path_following/path']:
        plot_map(topics, arguments.output / f'{name}_map.png', title)
    plot_response(topics, arguments.output / f'{name}_response.png', title)
    summary = metrics(topics)
    (arguments.output / f'{name}.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
