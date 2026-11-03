#include <iostream>
#include <ruckig/ruckig.hpp>

using namespace ruckig;

int main() {
    Ruckig<2> otg(0.01);
    InputParameter<2> input;
    input.current_position = {0.0, 0.0};
    input.target_position = {3.0, 1.5};
    input.intermediate_positions = {{1.0, 0.5}, {2.0, 1.0}};
    input.max_velocity = input.max_acceleration = {2.0, 2.0};
    input.max_jerk = {4.0, 4.0};
    input.min_position = {0.0, 0.0};
    input.max_position = {3.0, 1.5};
    OutputParameter<2> output;

    if (otg.calculate_waypoints(input, output.trajectory) != Result::Working) {
        return 1;
    }
    std::cout << "Stop at each waypoint: " << output.trajectory.get_duration() << " s\n";

    if (otg.calculate_waypoints(input, output.trajectory,
            {{0.5, 0.25}, {0.5, 0.25}}, {{0.0, 0.0}, {0.0, 0.0}}) != Result::Working) {
        return 2;
    }
    std::cout << "Prescribed pass-through states: " << output.trajectory.get_duration() << " s\n";
    Result result;
    do {
        result = otg.update(output.trajectory, output);
        if (result != Result::Working && result != Result::Finished) {
            return 3;
        }
    } while (result == Result::Working);
    std::cout << "Finished at " << output.new_position[0] << ", " << output.new_position[1] << '\n';
}
