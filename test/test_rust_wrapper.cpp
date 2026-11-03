#include "../src/wrapper/rust.hpp"

int main() {
    auto otg = ruckig_new_offline(2, 8);
    InputParameter input(2);
    input.current_position = {0.0, 0.0};
    input.target_position = {1.0, 1.0};
    input.max_velocity = input.max_acceleration = input.max_jerk = {1.0, 1.0};
    input.intermediate_positions = {{0.5, 0.5}};
    return !(otg->max_number_of_waypoints == 8 && otg->delta_time == -1.0 && otg->validate_input<false>(input));
}
