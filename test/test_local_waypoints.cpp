#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <ruckig/ruckig.hpp>

using namespace ruckig;

template<size_t DOFs>
InputParameter<DOFs> make_input(size_t dofs = DOFs) {
    auto input = [dofs]() {
        if constexpr (DOFs == 0) {
            return InputParameter<DOFs>(dofs);
        } else {
            return InputParameter<DOFs>();
        }
    }();
    for (size_t i = 0; i < dofs; ++i) {
        input.current_position[i] = 0.0;
        input.target_position[i] = 3.0;
        input.max_velocity[i] = 2.0;
        input.max_acceleration[i] = 2.0;
        input.max_jerk[i] = 4.0;
        input.min_position[i] = -1.0;
        input.max_position[i] = 4.0;
    }
    return input;
}

TEST_CASE("local-stop-and-replay") {
    Ruckig<1> otg(0.01);
    auto input = make_input<1>();
    input.intermediate_positions = {{1.0}, {2.0}};
    Trajectory<1> trajectory;
    REQUIRE(otg.calculate_waypoints(input, trajectory) == Result::Working);
    const auto times = trajectory.get_intermediate_durations();
    REQUIRE(times.size() == 3);
    CHECK(times.back() == trajectory.get_duration());
    CHECK(trajectory.get_profiles().size() == 3);
    for (size_t i = 0; i < 2; ++i) {
        double p = 0.0, v = 0.0, a = 0.0;
        trajectory.at_time(times[i], p, v, a);
        CHECK(p == doctest::Approx(i + 1.0));
        CHECK(v == doctest::Approx(0.0));
        CHECK(a == doctest::Approx(0.0));
        CHECK(trajectory.get_first_time_at_position(0, i + 1.0).value() == doctest::Approx(times[i]));
    }
    OutputParameter<1> output;
    output.trajectory = trajectory;
    size_t steps = 0;
    Result status = Result::Working;
    while (status == Result::Working && ++steps < 2000) {
        status = otg.update(output.trajectory, output);
        CHECK_FALSE(output.new_calculation);
        CHECK(output.new_position[0] >= -1e-8);
        CHECK(output.new_position[0] <= 3.0 + 1e-8);
    }
    CHECK(status == Result::Finished);
    CHECK(output.time == trajectory.get_duration());
    CHECK(output.new_position[0] == doctest::Approx(3.0));
    CHECK(output.new_section == 3);
    CHECK(input.intermediate_positions.size() == 2);

    // Ordinary state-to-state recalculation must stop using the local chain storage.
    input.intermediate_positions.clear();
    REQUIRE(otg.calculate(input, trajectory) == Result::Working);
    CHECK(trajectory.get_profiles().size() == 1);
    double p = 0.0;
    trajectory.at_time(trajectory.get_duration(), p);
    CHECK(p == doctest::Approx(3.0));
}

TEST_CASE("local-specified-states-and-continuity") {
    Ruckig<1> otg(0.01);
    auto input = make_input<1>();
    input.intermediate_positions = {{1.0}, {2.0}};
    input.target_velocity = {0.25};
    const std::vector<std::array<double, 1>> velocities{{0.5}, {0.5}}, accelerations{{0.2}, {-0.2}};
    Trajectory<1> trajectory;
    REQUIRE(otg.calculate_waypoints(input, trajectory, velocities, accelerations) == Result::Working);
    const auto times = trajectory.get_intermediate_durations();
    for (size_t i = 0; i < 2; ++i) {
        double p = 0.0, v = 0.0, a = 0.0;
        trajectory.at_time(times[i], p, v, a);
        CHECK(p == doctest::Approx(i + 1.0));
        CHECK(v == doctest::Approx(velocities[i][0]));
        CHECK(a == doctest::Approx(accelerations[i][0]));
        double left_p = 0.0, left_v = 0.0, left_a = 0.0;
        trajectory.at_time(times[i] - 1e-7, left_p, left_v, left_a);
        CHECK(std::abs(left_p - p) < 1e-6);
        CHECK(std::abs(left_v - v) < 1e-6);
        CHECK(std::abs(left_a - a) < 1e-6);
    }
    OutputParameter<1> output;
    otg.delta_time = trajectory.get_duration() + 1.0;
    CHECK(otg.update(trajectory, output) == Result::Finished);
    CHECK(output.new_velocity[0] == doctest::Approx(0.25));
    CHECK(otg.update(output.trajectory, output) == Result::Finished);
    CHECK(output.new_position[0] == doctest::Approx(3.0));
}

TEST_CASE("local-dynamic-multi-axis-and-zero-sections") {
    Ruckig<> otg(2, 0.01);
    auto input = make_input<0>(2);
    input.intermediate_positions = {{0.0, 0.0}, {1.0, 0.5}, {1.0, 0.5}};
    input.synchronization = Synchronization::Phase;
    Trajectory<> trajectory(2);
    REQUIRE(otg.calculate_waypoints(input, trajectory) == Result::Working);
    const auto times = trajectory.get_intermediate_durations();
    REQUIRE(times.size() == 4);
    CHECK(times[0] == 0.0);
    CHECK(times[1] == times[2]);
    CHECK(trajectory.get_profiles()[1][0].t == trajectory.get_profiles()[1][1].t);
    std::vector<double> p(2), v(2), a(2);
    trajectory.at_time(0.0, p, v, a);
    CHECK(p == std::vector<double>{0.0, 0.0});
    trajectory.at_time(times[1], p, v, a);
    CHECK(p[0] == doctest::Approx(1.0));
    CHECK(p[1] == doctest::Approx(0.5));
    std::vector<Bound> extrema(2);
    trajectory.get_position_extrema(extrema);
    CHECK(extrema[0].min >= -1e-8);
    CHECK(extrema[0].max <= 3.0 + 1e-8);

    // Global time_after must be converted to the appropriate section's local clock.
    input.intermediate_positions = {{1.0, 1.0}, {0.0, 0.0}, {1.0, 1.0}};
    REQUIRE(otg.calculate_waypoints(input, trajectory) == Result::Working);
    const auto crossings = trajectory.get_intermediate_durations();
    const auto time = trajectory.get_first_time_at_position(0, 1.0, crossings[1]);
    REQUIRE(time.has_value());
    CHECK(*time == doctest::Approx(crossings[2]));
}

TEST_CASE("local-bounds-braking-and-failure-invalidation") {
    Ruckig<1> otg(0.01);
    auto input = make_input<1>();
    input.intermediate_positions = {{1.0}, {2.0}};
    input.current_velocity = {3.0};
    input.max_position = {10.0};
    Trajectory<1> trajectory;
    REQUIRE(otg.calculate_waypoints(input, trajectory) == Result::Working);
    const auto profiles = trajectory.get_profiles();
    REQUIRE(profiles[0][0].brake.duration > 0.0);
    const auto early = trajectory.get_first_time_at_position(0, 0.1);
    REQUIRE(early.has_value());
    CHECK(*early < profiles[0][0].brake.duration);
    double p = 0.0;
    trajectory.at_time(*early, p);
    CHECK(p == doctest::Approx(0.1));

    input.current_velocity = {0.0};
    input.max_position = {1.5};
    CHECK(otg.calculate_waypoints(input, trajectory) == Result::ErrorPositionalLimits);
    OutputParameter<1> output;
    CHECK(otg.update(trajectory, output) == Result::ErrorInvalidInput);
    CHECK(output.time == 0.0);
}

TEST_CASE("local-input-validation") {
    Ruckig<> otg(2, 0.01);
    auto input = make_input<0>(2);
    input.intermediate_positions = {{1.0, 1.0}, {2.0, 2.0}};
    const auto valid = input;
    Trajectory<> trajectory(2);
    CHECK(otg.calculate_waypoints(input, trajectory, {{0.0, 0.0}}) == Result::ErrorInvalidInput);
    CHECK(otg.calculate_waypoints(input, trajectory, {{0.0}, {0.0}}) == Result::ErrorInvalidInput);
    CHECK(otg.calculate_waypoints(input, trajectory, {}, {{0.0}, {0.0}}) == Result::ErrorInvalidInput);
    input.intermediate_positions[1][0] = std::nan("");
    CHECK(otg.calculate_waypoints(input, trajectory) == Result::ErrorInvalidInput);
    input = valid;
    input.intermediate_positions[0].clear();
    CHECK(otg.calculate_waypoints(input, trajectory) == Result::ErrorInvalidInput);
    input = valid;
    input.synchronization = Synchronization::None;
    CHECK(otg.calculate_waypoints(input, trajectory) == Result::ErrorInvalidInput);
    input = valid;
    input.control_interface = ControlInterface::Velocity;
    CHECK(otg.calculate_waypoints(input, trajectory) == Result::ErrorInvalidInput);
    input = valid;
    input.enabled[0] = false;
    CHECK(otg.calculate_waypoints(input, trajectory) == Result::ErrorInvalidInput);
    input = valid;
    input.per_section_max_jerk = std::vector<std::vector<double>>{{1.0, 1.0}};
    CHECK(otg.calculate_waypoints(input, trajectory) == Result::ErrorInvalidInput);
    input = valid;
    CHECK(otg.calculate_waypoints(input, trajectory, {{3.0, 0.0}, {0.0, 0.0}}) == Result::ErrorInvalidInput);
    input.intermediate_positions.clear();
    REQUIRE(otg.calculate_waypoints(input, trajectory) == Result::Working);
    CHECK(trajectory.get_profiles().size() == 1);
    RuckigThrow<0> throwing(2, 0.01);
    CHECK_THROWS_AS(throwing.calculate_waypoints(valid, trajectory, {{0.0}}), RuckigError);
}

TEST_CASE("local-duration-limit-and-stationary-chain") {
    Ruckig<1> otg(0.01);
    auto input = make_input<1>();
    input.target_position = {0.0};
    input.intermediate_positions = {{0.0}, {0.0}};
    Trajectory<1> trajectory;
    REQUIRE(otg.calculate_waypoints(input, trajectory) == Result::Working);
    CHECK(trajectory.get_duration() == 0.0);
    OutputParameter<1> output;
    CHECK(otg.update(trajectory, output) == Result::Finished);
    CHECK(output.new_position[0] == 0.0);
    input.target_position = {9000.0};
    input.intermediate_positions = {{3000.0}, {6000.0}};
    input.max_velocity = {1.0};
    input.min_position = {-std::numeric_limits<double>::infinity()};
    input.max_position = {std::numeric_limits<double>::infinity()};
    CHECK(otg.calculate_waypoints(input, trajectory) == Result::ErrorTrajectoryDuration);
}
