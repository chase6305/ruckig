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

template<size_t DOFs>
void check_sampled_limits(const InputParameter<DOFs>& input, const Trajectory<DOFs>& trajectory) {
    auto p = input.current_position, v = p, a = p, j = p;
    size_t section = 0;
    for (size_t sample = 0; sample <= 200; ++sample) {
        trajectory.at_time(trajectory.get_duration() * sample / 200.0, p, v, a, j, section);
        for (size_t d = 0; d < input.degrees_of_freedom; ++d) {
            CHECK(p[d] >= input.min_position[d] - 1e-8);
            CHECK(p[d] <= input.max_position[d] + 1e-8);
            CHECK(v[d] >= (input.min_velocity ? (*input.min_velocity)[d] : -input.max_velocity[d]) - 1e-8);
            CHECK(v[d] <= input.max_velocity[d] + 1e-8);
            CHECK(a[d] >= (input.min_acceleration ? (*input.min_acceleration)[d] : -input.max_acceleration[d]) - 1e-8);
            CHECK(a[d] <= input.max_acceleration[d] + 1e-8);
            CHECK(std::abs(j[d]) <= input.max_jerk[d] + 1e-8);
        }
    }
}

TEST_CASE("automatic-states-and-bounded-refinement") {
    RuckigThrow<1> otg(0.01);
    auto input = make_input<1>();
    input.intermediate_positions = {{1.0}, {2.0}};
    const auto original = input;
    Trajectory<1> baseline, coarse, optimized;
    REQUIRE(otg.calculate_waypoints(input, baseline) == Result::Working);
    REQUIRE(otg.optimize_waypoints(input, coarse, 0) == Result::Working);
    CHECK(coarse.get_duration() == baseline.get_duration());
    REQUIRE(otg.optimize_waypoints(input, coarse, 1) == Result::Working);
    REQUIRE(otg.optimize_waypoints(input, optimized) == Result::Working);
    CHECK(optimized.get_duration() <= coarse.get_duration());
    CHECK(optimized.get_duration() < baseline.get_duration() * 0.8);
    CHECK_FALSE(input != original);
    const auto times = optimized.get_intermediate_durations();
    for (size_t i = 0; i < input.intermediate_positions.size(); ++i) {
        double p = 0.0, v = 0.0, a = 0.0;
        optimized.at_time(times[i], p, v, a);
        CHECK(p == doctest::Approx(input.intermediate_positions[i][0]));
        CHECK(v > 0.1);
        CHECK(std::abs(a) > 0.1);
        double lp = 0.0, lv = 0.0, la = 0.0;
        optimized.at_time(times[i] - 1e-7, lp, lv, la);
        CHECK(std::abs(lp - p) < 1e-6);
        CHECK(std::abs(lv - v) < 1e-6);
        CHECK(std::abs(la - a) < 1e-6);
    }
    check_sampled_limits(input, optimized);
    Trajectory<1> repeated;
    REQUIRE(otg.optimize_waypoints(input, repeated) == Result::Working);
    CHECK(repeated.get_duration() == optimized.get_duration());
    OutputParameter<1> output;
    otg.delta_time = optimized.get_duration() + 1.0;
    CHECK(otg.update(optimized, output) == Result::Finished);
    CHECK(output.new_position[0] == doctest::Approx(3.0));
}

TEST_CASE("automatic-multi-axis-and-directional-limits") {
    Ruckig<2> otg(0.01);
    auto input = make_input<2>();
    input.intermediate_positions = {{1.0, 0.5}, {2.0, 1.0}};
    input.target_position = {3.0, 1.5};
    Trajectory<2> baseline, optimized;
    REQUIRE(otg.calculate_waypoints(input, baseline) == Result::Working);
    REQUIRE(otg.optimize_waypoints(input, optimized) == Result::Working);
    CHECK(optimized.get_duration() < baseline.get_duration());
    check_sampled_limits(input, optimized);

    input.intermediate_positions = {{-1.0, -0.5}, {-2.0, -1.0}};
    input.target_position = {-3.0, -1.5};
    input.min_position = {-4.0, -4.0};
    input.max_position = {0.0, 0.0};
    input.min_velocity = std::array<double, 2>{-0.75, -0.75};
    input.min_acceleration = std::array<double, 2>{-0.5, -0.5};
    REQUIRE(otg.calculate_waypoints(input, baseline) == Result::Working);
    REQUIRE(otg.optimize_waypoints(input, optimized) == Result::Working);
    CHECK(optimized.get_duration() <= baseline.get_duration());
    check_sampled_limits(input, optimized);
}

TEST_CASE("automatic-reversals-and-repeated-waypoints") {
    Ruckig<1> otg;
    auto input = make_input<1>();
    input.intermediate_positions = {{1.0}};
    input.target_position = {0.0};
    input.min_position = {0.0};
    input.max_position = {1.0};
    Trajectory<1> baseline, optimized;
    REQUIRE(otg.calculate_waypoints(input, baseline) == Result::Working);
    REQUIRE(otg.optimize_waypoints(input, optimized) == Result::Working);
    CHECK(optimized.get_duration() <= baseline.get_duration());
    double p = 0.0, v = 0.0, a = 0.0;
    optimized.at_time(optimized.get_intermediate_durations()[0], p, v, a);
    CHECK(p == doctest::Approx(1.0));
    CHECK(std::abs(v) < 1e-8);
    CHECK(a <= 0.0);
    check_sampled_limits(input, optimized);

    input.intermediate_positions = {{0.0}, {1.0}, {1.0}};
    REQUIRE(otg.calculate_waypoints(input, baseline) == Result::Working);
    REQUIRE(otg.optimize_waypoints(input, optimized) == Result::Working);
    CHECK(optimized.get_duration() <= baseline.get_duration());
    CHECK(optimized.get_intermediate_durations().size() == 4);
    check_sampled_limits(input, optimized);
}

TEST_CASE("automatic-feasible-seed-with-infeasible-stopping-baseline") {
    Ruckig<1> otg;
    auto input = make_input<1>();
    input.current_velocity = input.target_velocity = {1.0};
    input.max_velocity = {1.0};
    input.max_jerk = {0.0};
    input.intermediate_positions = {{0.5}, {1.5}};
    input.target_position = {2.0};
    Trajectory<1> trajectory;
    CHECK(otg.calculate_waypoints(input, trajectory) == Result::ErrorZeroLimits);
    REQUIRE(otg.optimize_waypoints(input, trajectory) == Result::Working);
    CHECK(trajectory.get_duration() == doctest::Approx(2.0));
    check_sampled_limits(input, trajectory);
}

TEST_CASE("automatic-validation-and-invalidation") {
    Ruckig<> otg(2, 0.01);
    auto input = make_input<0>(2);
    input.intermediate_positions = {{1.0, 1.0}};
    Trajectory<> trajectory(2);
    REQUIRE(otg.optimize_waypoints(input, trajectory) == Result::Working);
    CHECK(otg.optimize_waypoints(input, trajectory, 65) == Result::ErrorInvalidInput);
    OutputParameter<> output(2);
    CHECK(otg.update(trajectory, output) == Result::ErrorInvalidInput);
    input.max_position = {0.5, 0.5};
    CHECK(otg.optimize_waypoints(input, trajectory) == Result::ErrorPositionalLimits);
    CHECK(otg.update(trajectory, output) == Result::ErrorInvalidInput);
    input.intermediate_positions[0].clear();
    CHECK(otg.optimize_waypoints(input, trajectory) == Result::ErrorInvalidInput);
    RuckigThrow<0> throwing(2, 0.01);
    CHECK_THROWS_AS(throwing.optimize_waypoints(input, trajectory, 65), RuckigError);
}

TEST_CASE("short-saturated-jerk-sections") {
    Ruckig<1> otg;
    for (double sign: {-1.0, 1.0}) {
        auto input = make_input<1>();
        input.target_position = {sign * 3.0};
        input.min_position = {-4.0};
        Trajectory<1> reference, chain;
        REQUIRE(otg.calculate(input, reference) == Result::Working);
        std::vector<std::array<double, 1>> velocities, accelerations;
        std::vector<double> crossings;
        for (size_t i = 1; i < 20; ++i) {
            const double position = sign * 3.0 * i / 20;
            const auto time = reference.get_first_time_at_position(0, position);
            REQUIRE(time.has_value());
            double p = 0.0, v = 0.0, a = 0.0;
            reference.at_time(*time, p, v, a);
            input.intermediate_positions.push_back({position});
            velocities.push_back({v});
            accelerations.push_back({a});
            crossings.push_back(*time);
        }
        REQUIRE(otg.calculate_waypoints(input, chain, velocities, accelerations) == Result::Working);
        CHECK(chain.get_duration() == doctest::Approx(reference.get_duration()).epsilon(1e-9));
        CHECK(chain.get_independent_min_durations()[0] == doctest::Approx(reference.get_duration()).epsilon(1e-9));
        const auto times = chain.get_intermediate_durations();
        for (size_t i = 0; i < crossings.size(); ++i) {
            CHECK(times[i] == doctest::Approx(crossings[i]).epsilon(1e-9));
        }
        check_sampled_limits(input, chain);
    }
}

TEST_CASE("automatic-dense-straight-waypoints") {
    Ruckig<> otg(6);
    auto input = make_input<0>(6);
    const std::array<double, 6> coefficients{1.0, 0.8, 0.6, -0.4, -0.2, 0.0};
    for (size_t d = 0; d < 6; ++d) {
        input.target_position[d] = 3.0 * coefficients[d];
        input.min_position[d] = -4.0;
    }
    Trajectory<> endpoint(6), optimized(6);
    REQUIRE(otg.calculate(input, endpoint) == Result::Working);
    for (size_t segments: {3, 20, 200}) {
        input.intermediate_positions.clear();
        for (size_t i = 1; i < segments; ++i) {
            auto point = input.target_position;
            for (double& p: point) {
                p *= static_cast<double>(i) / segments;
            }
            input.intermediate_positions.push_back(point);
        }
        REQUIRE(otg.optimize_waypoints(input, optimized) == Result::Working);
        CHECK(optimized.get_duration() == doctest::Approx(endpoint.get_duration()).epsilon(1e-8));
        const auto times = optimized.get_intermediate_durations();
        REQUIRE(times.size() == segments);
        auto p = input.current_position, v = p, a = p;
        for (size_t i = 0; i < segments - 1; ++i) {
            optimized.at_time(times[i], p, v, a);
            for (size_t d = 0; d < 6; ++d) {
                CHECK(p[d] == doctest::Approx(input.intermediate_positions[i][d]).epsilon(1e-8));
            }
            CHECK(v[0] > 0.0);
        }
        check_sampled_limits(input, optimized);
    }
}

TEST_CASE("automatic-reference-must-visit-all-coordinates") {
    Ruckig<2> otg;
    auto input = make_input<2>();
    input.target_position = {3.0, 1.5};
    input.intermediate_positions = {{0.4, 0.2}, {0.8, 0.6}, {1.4, 0.4}, {2.0, 0.8}};
    Trajectory<2> baseline, optimized;
    REQUIRE(otg.calculate_waypoints(input, baseline) == Result::Working);
    REQUIRE(otg.optimize_waypoints(input, optimized) == Result::Working);
    CHECK(optimized.get_duration() <= baseline.get_duration());
    const auto times = optimized.get_intermediate_durations();
    REQUIRE(times.size() == 5);
    std::array<double, 2> p, v, a;
    for (size_t i = 0; i < input.intermediate_positions.size(); ++i) {
        optimized.at_time(times[i], p, v, a);
        for (size_t d = 0; d < 2; ++d) {
            CHECK(p[d] == doctest::Approx(input.intermediate_positions[i][d]).epsilon(1e-8));
        }
    }
    check_sampled_limits(input, optimized);
}
