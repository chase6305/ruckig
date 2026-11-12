import math
import unittest

from ruckig import InputParameter, OutputParameter, Profile, Result, Ruckig, RuckigError, Trajectory


class WrapperRegressionTest(unittest.TestCase):
    def setUp(self):
        self.input = InputParameter(1)
        self.input.current_position = [0.0]
        self.input.target_position = [1.0]
        self.input.max_velocity = [1.0]
        self.input.max_acceleration = [1.0]
        self.input.max_jerk = [1.0]

    def test_fresh_output_is_initialized(self):
        for dofs in (1, 3):
            output = OutputParameter(dofs)
            self.assertEqual(output.calculation_duration, 0.0)
            self.assertEqual(output.new_position, [0.0] * dofs)
            self.assertEqual(output.new_velocity, [0.0] * dofs)
            self.assertEqual(output.new_acceleration, [0.0] * dofs)
            self.assertEqual(output.new_jerk, [0.0] * dofs)
            self.assertIsInstance(repr(output), str)

    def test_disabled_axis_clears_previous_minimum_duration(self):
        otg, trajectory = Ruckig(1), Trajectory(1)
        self.assertEqual(otg.calculate(self.input, trajectory), Result.Working)
        self.assertGreater(trajectory.independent_min_durations[0], 0.0)
        self.input.enabled = [False]
        self.assertEqual(otg.calculate(self.input, trajectory), Result.Working)
        self.assertEqual(trajectory.independent_min_durations, [0.0])
        self.assertEqual(trajectory.duration, 0.0)

    def test_pass_to_input_removes_all_crossed_local_waypoints(self):
        self.input.target_position = [4.0]
        self.input.intermediate_positions = [[1.0], [2.0], [3.0]]
        self.input.max_velocity = self.input.max_acceleration = [2.0]
        self.input.max_jerk = [4.0]
        otg, output = Ruckig(1, 5.0), OutputParameter(1)
        self.assertEqual(otg.calculate_waypoints(self.input, output.trajectory), Result.Working)
        self.assertEqual(otg.update(output.trajectory, output), Result.Working)
        self.assertEqual(output.new_section, 2)
        output.pass_to_input(self.input)
        self.assertEqual(self.input.intermediate_positions, [[3.0]])
        output.pass_to_input(self.input)
        self.assertEqual(self.input.intermediate_positions, [[3.0]])
        self.assertEqual(otg.update(output.trajectory, output), Result.Finished)
        output.pass_to_input(self.input)
        self.assertEqual(self.input.intermediate_positions, [])

    def test_invalid_dimensions(self):
        self.input.current_position = []
        with self.assertRaises(RuckigError):
            self.input.validate()

    def test_batch_sampling(self):
        trajectory = Trajectory(1)
        self.assertEqual(Ruckig(1).calculate(self.input, trajectory), Result.Working)
        times = [0.0, 0.5, trajectory.duration]
        positions, velocities, accelerations = trajectory.at_times(times)
        for i, time in enumerate(times):
            self.assertEqual((positions[i], velocities[i], accelerations[i]), trajectory.at_time(time))
        self.assertEqual(trajectory.at_times([]), ([], [], []))

    def test_profile_metadata_enums(self):
        otg, trajectory = Ruckig(1), Trajectory(1)
        for target, direction in ((1.0, Profile.Direction.UP), (-1.0, Profile.Direction.DOWN)):
            self.input.target_position = [target]
            self.assertEqual(otg.calculate(self.input, trajectory), Result.Working)
            profile = trajectory.profiles[0][0]
            self.assertEqual(profile.direction, direction)
            self.assertIsInstance(profile.limits, Profile.ReachedLimits)
            self.assertIsInstance(profile.control_signs, Profile.ControlSigns)

    def test_position_bounds(self):
        trajectory = Trajectory(1)
        otg = Ruckig(1, 0.1)
        self.input.min_position = [0.0]
        self.input.max_position = [1.0]
        self.assertEqual(otg.calculate(self.input, trajectory), Result.Working)
        self.input.max_position = [0.5]
        self.assertEqual(otg.calculate(self.input, trajectory), Result.ErrorPositionalLimits)
        with self.assertRaises(RuckigError):
            otg.update(trajectory, OutputParameter(1))

    def test_offline_execution(self):
        otg = Ruckig(1, 0.3)
        trajectory = Trajectory(1)
        self.input.target_velocity = [0.5]
        self.assertEqual(otg.calculate(self.input, trajectory), Result.Working)
        output = OutputParameter(1)
        output.time = 0.4
        for _ in range(100):
            result = otg.update(trajectory, output)
            self.assertEqual((output.new_position, output.new_velocity, output.new_acceleration), trajectory.at_time(output.time))
            if result == Result.Finished:
                break
        self.assertEqual(result, Result.Finished)
        self.assertEqual(output.time, trajectory.duration)
        self.assertAlmostEqual(output.new_position[0], 1.0)
        self.assertAlmostEqual(output.new_velocity[0], 0.5)
        self.assertEqual(otg.update(output.trajectory, output), Result.Finished)
        self.assertEqual(output.time, trajectory.duration)
        for bad_cycle in (0.0, -1.0, float('nan'), float('inf')):
            otg.delta_time = bad_cycle
            with self.assertRaises(RuckigError):
                otg.update(trajectory, output)
            with self.assertRaises(RuckigError):
                otg.update(self.input, output)

    def test_local_waypoints_stop(self):
        otg = Ruckig(1, 0.3)
        self.input.target_position = [3.0]
        self.input.intermediate_positions = [[1.0], [2.0]]
        output = OutputParameter(1)
        self.assertEqual(otg.calculate_waypoints(self.input, output.trajectory), Result.Working)
        times = output.trajectory.intermediate_durations
        self.assertEqual(len(times), 3)
        self.assertEqual(len(output.trajectory.profiles), 3)
        for i, time in enumerate(times[:-1]):
            p, v, a = output.trajectory.at_time(time)
            self.assertAlmostEqual(p[0], i + 1.0)
            self.assertAlmostEqual(v[0], 0.0)
            self.assertAlmostEqual(a[0], 0.0)
        for _ in range(100):
            result = otg.update(output.trajectory, output)
            if result == Result.Finished:
                break
        self.assertEqual(result, Result.Finished)
        self.assertEqual(output.time, times[-1])
        self.assertAlmostEqual(output.new_position[0], 3.0)
        self.assertEqual(output.new_section, 3)

    def test_local_waypoints_given_states(self):
        otg = Ruckig(1)
        self.input.target_position = [3.0]
        self.input.intermediate_positions = [[1.0], [2.0]]
        trajectory = Trajectory(1)
        velocities, accelerations = [[0.5], [0.5]], [[0.1], [-0.1]]
        self.assertEqual(otg.calculate_waypoints(self.input, trajectory, velocities, accelerations), Result.Working)
        for i, time in enumerate(trajectory.intermediate_durations[:-1]):
            p, v, a = trajectory.at_time(time)
            self.assertAlmostEqual(p[0], i + 1.0)
            self.assertAlmostEqual(v[0], velocities[i][0])
            self.assertAlmostEqual(a[0], accelerations[i][0])
        self.input.intermediate_positions = []
        self.assertEqual(otg.calculate(self.input, trajectory), Result.Working)
        self.assertEqual(len(trajectory.profiles), 1)

    def test_local_waypoints_reject_states(self):
        otg = Ruckig(1)
        trajectory = Trajectory(1)
        self.input.intermediate_positions = [[0.5]]
        for velocities in ([[0.0], [0.0]], [[]], [[float('inf')]], [[2.0]]):
            with self.assertRaises(RuckigError):
                otg.calculate_waypoints(self.input, trajectory, velocities)
        self.input.intermediate_positions = [[float('nan')]]
        with self.assertRaises(RuckigError):
            otg.calculate_waypoints(self.input, trajectory)

    def test_local_waypoints_reject_bounds(self):
        otg = Ruckig(1, 0.1)
        trajectory = Trajectory(1)
        self.input.intermediate_positions = [[2.0]]
        self.input.max_position = [1.5]
        self.assertEqual(otg.calculate_waypoints(self.input, trajectory), Result.ErrorPositionalLimits)
        with self.assertRaises(RuckigError):
            otg.update(trajectory, OutputParameter(1))

    def test_local_waypoints_total_duration(self):
        self.input.target_position = [9000.0]
        self.input.intermediate_positions = [[3000.0], [6000.0]]
        self.assertEqual(Ruckig(1).calculate_waypoints(self.input, Trajectory(1)), Result.ErrorTrajectoryDuration)


class LocalPlanningRegressionTest(unittest.TestCase):
    def setUp(self):
        self.input = InputParameter(1)
        self.input.current_position = [0.0]
        self.input.target_position = [1.0]
        self.input.max_velocity = [1.0]
        self.input.max_acceleration = [1.0]
        self.input.max_jerk = [1.0]

    def test_automatic_waypoint_states(self):
        otg = Ruckig(1, 0.1)
        self.input.max_velocity = [2.0]
        self.input.max_acceleration = [2.0]
        self.input.max_jerk = [4.0]
        self.input.target_position = [3.0]
        self.input.intermediate_positions = [[1.0], [2.0]]
        baseline, optimized = Trajectory(1), Trajectory(1)
        self.assertEqual(otg.calculate_waypoints(self.input, baseline), Result.Working)
        self.assertEqual(otg.optimize_waypoints(self.input, optimized), Result.Working)
        self.assertLess(optimized.duration, baseline.duration * 0.8)
        for i, time in enumerate(optimized.intermediate_durations[:-1]):
            p, v, a = optimized.at_time(time)
            self.assertAlmostEqual(p[0], i + 1.0)
            self.assertGreater(v[0], 0.1)
            self.assertGreater(abs(a[0]), 0.01)
        self.assertEqual(otg.optimize_waypoints(self.input, baseline, max_iterations=0), Result.Working)
        self.assertLess(optimized.duration, baseline.duration)
        with self.assertRaises(RuckigError):
            otg.optimize_waypoints(self.input, optimized, max_iterations=65)
        with self.assertRaises(RuckigError):
            otg.update(optimized, OutputParameter(1))

    def test_zero_jerk_reference_recovers_after_failed_stopping_baseline(self):
        velocity, acceleration = [-0.525, 0.275], [0.5, -0.25]
        inp, trajectory, otg = InputParameter(2), Trajectory(2), Ruckig(2)
        inp.current_position = [0.0, 0.0]
        inp.current_velocity, inp.current_acceleration = velocity, acceleration
        inp.target_position = [2.0 * v + 2.0 * a for v, a in zip(velocity, acceleration)]
        inp.target_velocity = [v + 2.0 * a for v, a in zip(velocity, acceleration)]
        inp.target_acceleration = acceleration
        inp.max_velocity = inp.max_acceleration = [1.0, 1.0]
        inp.max_jerk = [0.0, 0.0]
        self.assertEqual(otg.calculate(inp, trajectory), Result.Working)
        inp.intermediate_positions = [
            [v * t + a * t * t / 2.0 for v, a in zip(velocity, acceleration)]
            for t in (0.5, 1.0)
        ]
        self.assertEqual(otg.optimize_waypoints(inp, trajectory, max_iterations=0), Result.ErrorZeroLimits)
        with self.assertRaises(RuckigError):
            trajectory.at_time(0.0)
        self.assertEqual(otg.optimize_waypoints(inp, trajectory), Result.Working)
        self.assertAlmostEqual(trajectory.duration, 2.0, places=12)
        for time in (0.0, 0.125, 0.5, 1.0, 1.05, 1.1, 1.5, 2.0):
            positions, velocities, accelerations = trajectory.at_time(time)
            for axis, (v, a) in enumerate(zip(velocity, acceleration)):
                self.assertAlmostEqual(positions[axis], v * time + a * time * time / 2.0, places=12)
                self.assertAlmostEqual(velocities[axis], v + a * time, places=12)
                self.assertAlmostEqual(accelerations[axis], a, places=12)

    def test_automatic_search_can_reject_trials_without_throwing(self):
        otg = Ruckig(1)
        self.input.current_velocity = [1.0]
        self.input.target_velocity = [1.0]
        self.input.target_position = [2.0]
        self.input.intermediate_positions = [[0.5], [1.5]]
        self.input.max_jerk = [0.0]
        trajectory = Trajectory(1)
        self.assertEqual(otg.optimize_waypoints(self.input, trajectory), Result.Working)
        self.assertAlmostEqual(trajectory.duration, 2.0)

    def test_dense_waypoints_keep_cruise_speed(self):
        inp = InputParameter(2)
        inp.current_position = [0.0, 0.0]
        inp.target_position = [3.0, -1.5]
        inp.max_velocity = inp.max_acceleration = [2.0, 2.0]
        inp.max_jerk = [4.0, 4.0]
        inp.intermediate_positions = [[3.0 * i / 20, -1.5 * i / 20] for i in range(1, 20)]
        trajectory = Trajectory(2)
        self.assertEqual(Ruckig(2).optimize_waypoints(inp, trajectory), Result.Working)
        self.assertAlmostEqual(trajectory.duration, 3.0)
        self.assertEqual(len(trajectory.intermediate_durations), 20)
        for point, time in zip(inp.intermediate_positions, trajectory.intermediate_durations):
            p, v, _ = trajectory.at_time(time)
            self.assertAlmostEqual(p[0], point[0])
            self.assertAlmostEqual(p[1], point[1])
            self.assertGreater(v[0], 0.1)

    def test_repeated_waypoints_keep_motion(self):
        self.input.target_position = [3.0]
        self.input.max_velocity = self.input.max_acceleration = [2.0]
        self.input.max_jerk = [4.0]
        self.input.intermediate_positions = [[1.0], [1.0], [2.0]]
        trajectory = Trajectory(1)
        self.assertEqual(Ruckig(1).optimize_waypoints(self.input, trajectory), Result.Working)
        self.assertAlmostEqual(trajectory.duration, 3.0)
        times = trajectory.intermediate_durations
        self.assertEqual(times[0], times[1])
        p, v, a = trajectory.at_time(times[0])
        self.assertAlmostEqual(p[0], 1.0)
        self.assertGreater(v[0], 1.0)
        self.assertGreater(a[0], 0.1)

    def test_local_per_section_timing_scalar_oracle(self):
        self.input.target_position = [0.0]
        self.input.intermediate_positions = [[0.0], [0.0]]
        minima = [0.01, 0.02, 0.0]
        self.input.per_section_minimum_duration = minima
        trajectory = Trajectory(1)
        self.assertEqual(Ruckig(1).calculate_waypoints(self.input, trajectory), Result.Working)
        prefix = 0.0
        for index, minimum in enumerate(minima):
            scalar = InputParameter(1)
            scalar.target_position = [0.0]
            scalar.max_velocity = scalar.max_acceleration = scalar.max_jerk = [1.0]
            scalar.minimum_duration = minimum
            oracle = Trajectory(1)
            self.assertEqual(Ruckig(1).calculate(scalar, oracle), Result.Working)
            self.assertGreaterEqual(oracle.duration, minimum)
            prefix += oracle.duration
            self.assertEqual(trajectory.intermediate_durations[index], prefix)
        times = trajectory.intermediate_durations
        self.assertEqual(times[1] - times[0], math.nextafter(0.02, 0.0))

    def test_automatic_per_section_timing_and_validation(self):
        self.input.target_position = [3.0]
        self.input.intermediate_positions = [[1.0], [1.0], [2.0]]
        self.input.per_section_minimum_duration = [0.0, 3.0, 0.0, 0.5]
        trajectory = Trajectory(1)
        otg = Ruckig(1)
        self.assertEqual(otg.optimize_waypoints(self.input, trajectory), Result.Working)
        times = trajectory.intermediate_durations
        self.assertEqual(len(times), 4)
        self.assertGreater(times[1], times[0])
        for point, time in zip(self.input.intermediate_positions, times):
            self.assertAlmostEqual(trajectory.at_time(time)[0][0], point[0])
        self.input.per_section_minimum_duration = [0.0, float('nan'), 0.0, 0.0]
        with self.assertRaises(RuckigError):
            otg.optimize_waypoints(self.input, trajectory)
        with self.assertRaises(RuckigError):
            trajectory.at_time(0.0)
        self.input.intermediate_positions = []
        self.input.per_section_minimum_duration = [4.0]
        self.input.minimum_duration = 5.0
        self.assertEqual(otg.calculate_waypoints(self.input, trajectory), Result.Working)
        self.assertEqual(trajectory.duration, 5.0)

    def test_sampling_rejects_invalid_state_and_time(self):
        trajectory = Trajectory(1)
        self.assertEqual(trajectory.duration, 0.0)
        with self.assertRaises(RuckigError):
            trajectory.at_time(0.0)

        otg = Ruckig(1)
        self.assertEqual(otg.calculate(self.input, trajectory), Result.Working)
        for time in (float('nan'), float('inf'), float('-inf')):
            with self.assertRaises(RuckigError):
                trajectory.at_time(time)
            with self.assertRaises(RuckigError):
                trajectory.at_times([0.0, time])
        p, v, a = trajectory.at_time(-0.1)
        self.assertTrue(all(abs(state[0]) < 1.0 for state in (p, v, a)))
        self.assertIsNone(trajectory.get_first_time_at_position(0, float('nan')))
        self.input.max_position = [0.5]
        self.assertEqual(otg.calculate(self.input, trajectory), Result.ErrorPositionalLimits)
        with self.assertRaises(RuckigError):
            trajectory.at_time(0.0)

    def test_local_position_query_windows(self):
        otg, trajectory = Ruckig(1), Trajectory(1)
        prefix = 2.0 ** -53
        after = math.nextafter(1.0, math.inf)
        for speed, should_match in ((1.0, True), (2.0 ** 23, False)):
            inp = InputParameter(1)
            inp.current_position = [-speed * prefix]
            inp.target_position = [2.0 * speed]
            inp.current_velocity = inp.target_velocity = [speed]
            inp.max_velocity = [2.0 * speed]
            inp.max_acceleration = [2.0]
            inp.max_jerk = [0.0]
            inp.intermediate_positions = [[0.0], [speed]]
            self.assertEqual(otg.calculate_waypoints(inp, trajectory, [[speed], [speed]], [[0.0], [0.0]]), Result.Working)
            self.assertEqual(trajectory.intermediate_durations, [prefix, 1.0, 2.0])
            crossing = trajectory.get_first_time_at_position(0, speed, after)
            if should_match:
                self.assertEqual(crossing, after)
            else:
                self.assertIsNone(crossing)

        self.input.target_position = [3.0]
        self.input.intermediate_positions = [[1.0], [2.0]]
        self.assertEqual(otg.calculate_waypoints(self.input, trajectory), Result.Working)
        duration = trajectory.duration
        position = trajectory.at_time(duration)[0][0]
        self.assertEqual(trajectory.get_first_time_at_position(0, position, duration), duration)
        self.assertIsNone(trajectory.get_first_time_at_position(0, position, math.nextafter(duration, math.inf)))

    def test_dense_curved_waypoints(self):
        inp = InputParameter(2)
        inp.target_position = [1.0, 1.0]
        inp.max_velocity = inp.max_acceleration = [2.0, 2.0]
        inp.max_jerk = [4.0, 4.0]
        inp.intermediate_positions = [[math.sin(math.pi * i / 80), 1 - math.cos(math.pi * i / 80)] for i in range(1, 40)]
        trajectory = Trajectory(2)
        self.assertEqual(Ruckig(2).optimize_waypoints(inp, trajectory), Result.Working)
        self.assertLess(trajectory.duration, 3.5)
        for point, time in zip(inp.intermediate_positions, trajectory.intermediate_durations):
            p, _, _ = trajectory.at_time(time)
            self.assertAlmostEqual(p[0], point[0])
            self.assertAlmostEqual(p[1], point[1])

    def test_refined_multi_axis_trajectory_owns_its_samples(self):
        def plan(dofs):
            inp = InputParameter(dofs)
            inp.target_position = [3.0] + [0.0] * (dofs - 2) + [1.5]
            inp.max_velocity = inp.max_acceleration = [2.0] * dofs
            inp.max_jerk = [4.0] * dofs
            inp.intermediate_positions = [
                [x] + [0.0] * (dofs - 2) + [y]
                for x, y in [(0.4, 0.2), (0.8, 0.6), (1.4, 0.4), (2.0, 0.8)]
            ]
            trajectory = Trajectory(dofs)
            self.assertEqual(Ruckig(dofs).optimize_waypoints(inp, trajectory), Result.Working)
            return trajectory

        oracle, embedded = plan(2), plan(6)
        self.assertEqual(embedded.duration, oracle.duration)
        self.assertEqual(embedded.intermediate_durations, oracle.intermediate_durations)
        for i in range(17):
            time = oracle.duration * i / 16
            for actual, expected in zip(embedded.at_time(time), oracle.at_time(time)):
                self.assertEqual([actual[0], actual[-1]], expected)
                self.assertEqual(actual[1:-1], [0.0] * 4)

    def test_filter_waypoints_locally_and_validate_thresholds(self):
        inp = InputParameter(2)
        inp.current_position = [0.0, 0.0]
        inp.target_position = [1.0, 0.0]
        inp.intermediate_positions = [[0.5, 10.0]]
        otg = Ruckig(2)
        self.assertEqual(otg.filter_intermediate_positions(inp, [0.1, 0.1]), inp.intermediate_positions)
        inp.intermediate_positions = [[0.5, 0.05]]
        self.assertEqual(otg.filter_intermediate_positions(inp, [0.1, 0.1]), [])
        inp.target_position = [0.0, 0.0]
        inp.intermediate_positions = [[10.0, 0.0]]
        self.assertEqual(otg.filter_intermediate_positions(inp, [0.1, 0.1]), inp.intermediate_positions)
        for threshold in ([], [0.1], [-0.1, 0.1], [float('nan'), 0.1]):
            with self.assertRaises(RuckigError):
                otg.filter_intermediate_positions(inp, threshold)
        inp.intermediate_positions = [[1.0]]
        with self.assertRaises(RuckigError):
            otg.filter_intermediate_positions(inp, [0.1, 0.1])


if __name__ == '__main__':
    unittest.main()
