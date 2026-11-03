use ruckig::{InputParameter, OutputParameter, Result, Ruckig, Trajectory};

fn main() {
    let mut otg = Ruckig::new_direct(2, 0.01);
    let mut input = InputParameter::new(2);
    input.target_position = vec![3.0, 1.5];
    input.intermediate_positions = vec![vec![1.0, 0.5], vec![2.0, 1.0]];
    input.max_velocity = vec![2.0, 2.0];
    input.max_acceleration = vec![2.0, 2.0];
    input.max_jerk = vec![4.0, 4.0];
    input.min_position = vec![0.0, 0.0];
    input.max_position = vec![3.0, 1.5];
    let mut trajectory = Trajectory::new_direct(2);

    assert_eq!(otg.calculate_waypoints(&mut input, &mut trajectory), Result::Working);
    println!("Stop at each waypoint: {:.6} s", trajectory.get_duration());
    let velocities = vec![vec![0.5, 0.25], vec![0.5, 0.25]];
    let accelerations = vec![vec![0.0, 0.0], vec![0.0, 0.0]];
    assert_eq!(otg.calculate_waypoints_with_states(&mut input, &mut trajectory, &velocities, &accelerations), Result::Working);
    println!("Prescribed pass-through states: {:.6} s", trajectory.get_duration());
    assert_eq!(otg.optimize_waypoints(&mut input, &mut trajectory, 8), Result::Working);
    println!("Automatic pass-through states: {:.6} s", trajectory.get_duration());

    let mut output = OutputParameter::new_direct(2);
    let mut status = otg.update_trajectory(&trajectory, &mut output);
    while status == Result::Working {
        status = otg.update_current_trajectory(&mut output);
    }
    assert_eq!(status, Result::Finished);
    println!("Finished at {:?}", output.new_position);
}
