"""Run with the built ruckig Python module; times include Python call overhead."""

import math
import statistics
import time

from ruckig import InputParameter, Result, Ruckig, Trajectory


def benchmark_position_queries():
    batch_size = 128
    print(f'\n{"Sections":>10} {"Query":>18} {"Median/query (us)":>19}')
    for sections in (100, 1000, 10000):
        inp = InputParameter(1)
        inp.current_position = [0.0]
        inp.current_velocity = inp.target_velocity = [1.0]
        inp.target_position = [sections / 4.0]
        inp.intermediate_positions = [[k / 4.0] for k in range(1, sections)]
        inp.max_velocity = [1.0]
        inp.max_acceleration = [2.0]
        inp.max_jerk = [4.0]
        otg, trajectory = Ruckig(1), Trajectory(1)
        velocities, accelerations = [[1.0]] * (sections - 1), [[0.0]] * (sections - 1)
        if otg.calculate_waypoints(inp, trajectory, velocities, accelerations) != Result.Working:
            raise RuntimeError(f'{sections} sections: through-motion calculation failed')
        if len(trajectory.intermediate_durations) != sections or abs(trajectory.duration - sections / 4.0) > 1e-8:
            raise RuntimeError(f'{sections} sections: unexpected chain duration or size')
        for name in ('early', 'near-end', 'near-end-window'):
            positions = [(256 + k) / 4096.0 for k in range(batch_size)]
            if name != 'early':
                positions = [inp.target_position[0] - offset for offset in positions]
            time_after = trajectory.duration - 0.25 if name == 'near-end-window' else 0.0
            # Check the constant-speed samples and public query window outside timing.
            for position in positions:
                p, v, a = trajectory.at_time(position)
                crossing = trajectory.get_first_time_at_position(0, position, time_after)
                if abs(p[0] - position) > 1e-9 or abs(v[0] - 1.0) > 1e-9 or abs(a[0]) > 1e-9:
                    raise RuntimeError(f'{sections} sections: unexpected through-motion sample')
                if crossing is None or not max(0.0, time_after) <= crossing <= trajectory.duration:
                    raise RuntimeError(f'{sections} sections: query returned outside its window')
                if abs(trajectory.at_time(crossing)[0][0] - position) > 1e-9:
                    raise RuntimeError(f'{sections} sections: query returned the wrong position')
            measurements = []
            for repetition in range(12):
                start = time.perf_counter_ns()
                for position in positions:
                    trajectory.get_first_time_at_position(0, position, time_after)
                elapsed_us = (time.perf_counter_ns() - start) / 1000 / batch_size
                if repetition > 0:  # Exclude one complete warm-up batch.
                    measurements.append(elapsed_us)
            print(f'{sections:>10} {name:>18} {statistics.median(measurements):>19.3f}')


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
        ('endpoint-2d', [[0.0, 0.0], [1.0, 1.0], [2.0, 0.5], [3.0, 0.0]], [3.0, 0.0]),
        ('reference-2d', [[-23.0 / 96.0, 0.1]], [0.5, 0.2]),
        ('reference-both-2d', [[-23.0 / 96.0, -0.0276021073126]], [0.5, 0.2]),
        ('reference-state-2d', [[0.14285896102111803, -89.0 / 192.0]], [1.0, 0.5]),
        ('arc-2d-19', [[math.sin(math.pi * k / 40), 1 - math.cos(math.pi * k / 40)] for k in range(1, 20)], [1.0, 1.0]),
        ('arc-2d-39', [[math.sin(math.pi * k / 80), 1 - math.cos(math.pi * k / 80)] for k in range(1, 40)], [1.0, 1.0]),
        ('arc-6d-19', [[math.sin(math.pi * k / 40), 1 - math.cos(math.pi * k / 40)] * 3 for k in range(1, 20)], [1.0] * 6),
        ('arc-6d-39', [[math.sin(math.pi * k / 80), 1 - math.cos(math.pi * k / 80)] * 3 for k in range(1, 40)], [1.0] * 6),
        ('sine-2d-19', [[3.0 * k / 20, 0.1 * math.sin(2 * math.pi * k / 20)] for k in range(1, 20)], [3.0, 0.0]),
        ('sine-2d-39', [[3.0 * k / 40, 0.1 * math.sin(2 * math.pi * k / 40)] for k in range(1, 40)], [3.0, 0.0]),
    ]
    reference_velocities = {
        'reference-2d': [-1.0, 0.0],
        'reference-both-2d': [-1.0, -0.5],
        'reference-state-2d': [0.0, -1.0],
    }
    print(f'{"Case":<18} {"Stops (s)":>10} {"Auto (s)":>10} {"Median (us)":>12}')
    for name, points, target in cases:
        dofs = len(target)
        inp = InputParameter(dofs)
        inp.current_position = [0.0] * dofs
        inp.target_position = target
        if name == 'endpoint-2d':
            inp.current_velocity = inp.target_velocity = [1.0, 0.0]
        elif name in reference_velocities:
            inp.current_velocity = reference_velocities[name]
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

    print(f'\n{"DoFs":>6} {"Input points":>12} {"Filtering median (us)":>23}')
    for dofs, count in [(1, 1000), (1, 5000), (1, 10000), (6, 1000), (6, 5000)]:
        inp = InputParameter(dofs)
        inp.current_position = [0.0] * dofs
        inp.target_position = [0.0] * (dofs - 1) + [float(count + 1)]
        inp.intermediate_positions = [[0.0] * (dofs - 1) + [float(k)] for k in range(1, count + 1)]
        otg = Ruckig(dofs)
        measurements = []
        for repetition in range(12):
            start = time.perf_counter_ns()
            filtered = otg.filter_intermediate_positions(inp, [0.0] * dofs)
            elapsed_us = (time.perf_counter_ns() - start) / 1000
            if filtered:
                raise RuntimeError(f'{count} points: monotone route was not fully filtered')
            if repetition > 0:
                measurements.append(elapsed_us)
        print(f'{dofs:>6} {count:>12} {statistics.median(measurements):>23.1f}')

    benchmark_position_queries()


if __name__ == '__main__':
    main()
