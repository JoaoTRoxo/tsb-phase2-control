#!/usr/bin/env python3
"""
Regression checks on docs/results: the claims in README.md, as assertions.

Standard library only, so it runs on the host or in CI. A run with no results is
skipped, which lets CI record only a subset of the experiments.
"""

import json
from pathlib import Path
import sys

RESULTS = Path(__file__).resolve().parent.parent / 'docs' / 'results'
failures = []


def load(name):
    """Return the metrics of a run, or None if it was not recorded."""
    path = RESULTS / f'{name}.json'
    return json.loads(path.read_text()) if path.exists() else None


def check(condition, message):
    """Print and remember the outcome of one check."""
    print(('ok    ' if condition else 'FAIL  ') + message)
    if not condition:
        failures.append(message)


for mission in ('square', 'zigzag'):
    run = load(mission)
    if run:
        check(run['finished'], f'{mission}: the last waypoint is reached')
        check(run['cross_track_rms_m'] < 0.03, f'{mission}: RMS cross-track below 3 cm')
        check(run['final_distance_to_goal_m'] < 0.10, f'{mission}: stops within 10 cm of the goal')

steps = load('steps')
if steps:
    check(None not in steps['surge_settling_s'], 'steps: every surge step settles')
    check(None not in steps['heading_settling_s'], 'steps: every heading step settles')
    check(max(steps['heading_settling_s']) < 4.0, 'steps: heading settles within 4 s')

none = load('windup_none')
back_calculation = load('windup_back_calculation')
if none and back_calculation:
    recovery_none = none['surge_settling_s'][1]
    recovery_back = back_calculation['surge_settling_s'][1]
    # None means it had not recovered when the next step arrived, 10 s later.
    check(
        recovery_none is None or recovery_none > 5.0,
        'windup: without anti-windup, recovery takes over 5 s')
    check(
        recovery_back is not None and recovery_back < 2.0,
        'windup: with back-calculation, recovery takes under 2 s')
    check(
        back_calculation['peak_abs_integral_surge'] < 0.05,
        'windup: back-calculation keeps the integrator small')

sys.exit(1 if failures else 0)
