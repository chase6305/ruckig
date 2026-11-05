"""Run with the built ruckig Python module; times include Python call overhead."""

import math
import statistics
import time

from ruckig import InputParameter, Result, Ruckig, Trajectory


def main():
    coefficients = [1.0, 0.8, 0.6, -0.4, -0.2, 0.0]
    cases = [
        ('line-1d', [[1.0], [2.0]], [3.0]),
        ('dense-1d', [[3.0 * k / 20] for k in range(1, 20)], [3.0]),
        ('line-2d', [[1.0, 0.5], [2.0, 1.0]], [3.0, 1.5]),
        ('dense-6d', [[3.0 * k / 20 * c for c in coefficients] for k in range(1, 20)], [3.0 * c for c in coefficients]),
        ('curve-2d', [[0.4, 0.2], [0.8, 0.6], [1.4, 0.4], [2.0, 0.8]], [3.0, 1.5]),
        ('zigzag-2d', [[0.5, 1.0], [1.0, 0.0], [1.5, 1.0]], [2.0, 0.0]),
        ('reverse-1d', [[1.0]], [0.0]),
        ('reverse-1d-many', [[1.0], [-1.0], [1.0], [-1.0]], [0.0]),
        ('uneven-1d', [[0.25], [1.0]], [3.0]),
        ('repeat-1d', [[1.0], [1.0], [2.0]], [3.0]),
        ('arc-2d-19', [[math.sin(math.pi * k / 40), 1 - math.cos(math.pi * k / 40)] for k in range(1, 20)], [1.0, 1.0]),
        ('arc-2d-39', [[math.sin(math.pi * k / 80), 1 - math.cos(math.pi * k / 80)] for k in range(1, 40)], [1.0, 1.0]),
        ('arc-6d-19', [[math.sin(math.pi * k / 40), 1 - math.cos(math.pi * k / 40)] * 3 for k in range(1, 20)], [1.0] * 6),
        ('arc-6d-39', [[math.sin(math.pi * k / 80), 1 - math.cos(math.pi * k / 80)] * 3 for k in range(1, 40)], [1.0] * 6),
        ('sine-2d-19', [[3.0 * k / 20, 0.1 * math.sin(2 * math.pi * k / 20)] for k in range(1, 20)], [3.0, 0.0]),
        ('sine-2d-39', [[3.0 * k / 40, 0.1 * math.sin(2 * math.pi * k / 40)] for k in range(1, 40)], [3.0, 0.0]),
    ]
    print(f'{"Case":<18} {"Stops (s)":>10} {"Auto (s)":>10} {"Median (us)":>12}')
    for name, points, target in cases:
        dofs = len(target)
        inp = InputParameter(dofs)
        inp.current_position = [0.0] * dofs
        inp.target_position = target
        inp.intermediate_positions = points
        inp.max_velocity = inp.max_acceleration = [2.0] * dofs
        inp.max_jerk = [4.0] * dofs
        inp.min_position, inp.max_position = [-10.0] * dofs, [10.0] * dofs
        otg, trajectory = Ruckig(dofs), Trajectory(dofs)
        if otg.calculate_waypoints(inp, trajectory) != Result.Working:
            raise RuntimeError(f'{name}: stopping calculation failed')
        stopping_duration = trajectory.duration
        measurements = []
        for repetition in range(12):
            start = time.perf_counter_ns()
            result = otg.optimize_waypoints(inp, trajectory)
            elapsed_us = (time.perf_counter_ns() - start) / 1000
            if result != Result.Working:
                raise RuntimeError(f'{name}: optimization failed: {result}')
            if repetition > 0:  # Exclude the warm-up call.
                measurements.append(elapsed_us)
        print(f'{name:<18} {stopping_duration:>10.6f} {trajectory.duration:>10.6f} {statistics.median(measurements):>12.1f}')


if __name__ == '__main__':
    main()
