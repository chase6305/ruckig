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


if __name__ == '__main__':
    unittest.main()
