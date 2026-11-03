import unittest

from ruckig import InputParameter, Result, Ruckig, RuckigError, Trajectory


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


if __name__ == '__main__':
    unittest.main()
