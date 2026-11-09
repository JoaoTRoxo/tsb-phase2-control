#!/usr/bin/env python3
"""Regenerate figures and metrics for every bag, using the titles in run.sh."""

from pathlib import Path
import re
import subprocess

RUNS = re.findall(
    r'^\s*"([a-z_]+)\|\d+\|[^|]*\|([^"]+)"$',
    (Path(__file__).parent / 'run.sh').read_text(),
    re.MULTILINE)

for name, title in RUNS:
    bag = Path('experiments/bags') / name
    if bag.exists():
        subprocess.run(
            ['ros2', 'run', 'tsb_control', 'plot_run.py', str(bag), 'docs/results', '--title', title],
            check=True, stdout=subprocess.DEVNULL)
