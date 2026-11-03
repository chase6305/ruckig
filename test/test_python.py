import unittest

from ruckig import InputParameter, OutputParameter, Result, Ruckig, RuckigError, Trajectory


class WrapperRegressionTest(unittest.TestCase):
    def setUp(self):
        self.input = InputParameter(1)
        self.input.current_position = [0.0]
        self.input.target_position = [1.0]
        self.input.max_velocity = [1.0]
        self.input.max_acceleration = [1.0]
        self.input.max_jerk = [1.0]

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


if __name__ == '__main__':
    unittest.main()
