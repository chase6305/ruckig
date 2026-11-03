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


if __name__ == '__main__':
    unittest.main()
