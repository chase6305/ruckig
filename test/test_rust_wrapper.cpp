#include "../src/wrapper/rust.hpp"

int main() {
    auto otg = ruckig_new_offline(2, 8);
    InputParameter input(2);
    input.current_position = {0.0, 0.0};
    input.target_position = {1.0, 1.0};
    input.max_velocity = input.max_acceleration = input.max_jerk = {1.0, 1.0};
    input.intermediate_positions = {{0.5, 0.5}};
    if (!(otg->max_number_of_waypoints == 8 && otg->delta_time == -1.0 && otg->validate_input<false>(input))) {
        return 1;
    }
    input.intermediate_positions.clear();
    input.min_position = {0.0, 0.0};
    input.max_position = {1.0, 1.0};
    Trajectory trajectory(2);
    if (ruckig_calculate(*otg, input, trajectory) != ruckig::Result::Working) {
        return 2;
    }
    auto executor = ruckig_new_direct(2, 0.1);
    OutputParameter output(2);
    int result = ruckig::Result::Working;
    for (size_t i = 0; result == ruckig::Result::Working && i < 100; ++i) {
        result = ruckig_update_trajectory(*executor, trajectory, output);
    }
    return !(result == ruckig::Result::Finished && output.time == trajectory.get_duration());
}
