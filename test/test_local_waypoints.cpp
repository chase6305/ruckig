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

TEST_CASE("repeated-moving-waypoints-share-a-time") {
    Ruckig<1> otg(0.01);
    auto input = make_input<1>();
    input.intermediate_positions = {{1.0}, {1.0}, {2.0}};
    Trajectory<1> trajectory;
    REQUIRE(otg.optimize_waypoints(input, trajectory) == Result::Working);
    CHECK(trajectory.get_duration() == doctest::Approx(3.0));
    const auto times = trajectory.get_intermediate_durations();
    REQUIRE(times.size() == 4);
    CHECK(times[0] == times[1]);
    double p = 0.0, v = 0.0, a = 0.0;
    trajectory.at_time(times[0], p, v, a);
    CHECK(p == doctest::Approx(1.0));
    CHECK(v > 1.0);
    CHECK(a > 0.1);
    const auto profiles = trajectory.get_profiles();
    CHECK(profiles[1][0].t_sum.back() == 0.0);
    CHECK(profiles[1][0].v.front() == profiles[1][0].v.back());
    check_sampled_limits(input, trajectory);

    // Equal positions alone do not permit skipping a velocity/acceleration change.
    input.intermediate_positions = {{1.0}, {1.0}};
    REQUIRE(otg.calculate_waypoints(input, trajectory, {{0.5}, {0.4}}, {{0.0}, {0.0}}) == Result::Working);
    const auto changed = trajectory.get_intermediate_durations();
    CHECK(changed[1] > changed[0]);

    input.current_velocity = input.target_velocity = {1.0};
    input.max_velocity = {1.0};
    input.max_jerk = {0.0};
    input.intermediate_positions = {{0.5}, {0.5}, {1.5}};
    input.target_position = {2.0};
    REQUIRE(otg.optimize_waypoints(input, trajectory) == Result::Working);
    CHECK(trajectory.get_duration() == doctest::Approx(2.0));
    CHECK(trajectory.get_intermediate_durations()[0] == trajectory.get_intermediate_durations()[1]);
    check_sampled_limits(input, trajectory);
}

TEST_CASE("automatic-curved-path-seeds") {
    Ruckig<2> otg;
    for (size_t count: {19, 39}) {
        auto input = make_input<2>();
        input.target_position = {1.0, 1.0};
        for (size_t i = 1; i <= count; ++i) {
            const double angle = std::acos(-1.0) * i / (2 * (count + 1));
            input.intermediate_positions.push_back({std::sin(angle), 1.0 - std::cos(angle)});
        }
        Trajectory<2> stopping, coarse, optimized;
        REQUIRE(otg.calculate_waypoints(input, stopping) == Result::Working);
        REQUIRE(otg.optimize_waypoints(input, coarse, 1) == Result::Working);
        REQUIRE(otg.optimize_waypoints(input, optimized) == Result::Working);
        CHECK(optimized.get_duration() < 3.5);
        CHECK(optimized.get_duration() <= coarse.get_duration());
        CHECK(optimized.get_duration() < stopping.get_duration());
        const auto times = optimized.get_intermediate_durations();
        std::array<double, 2> p, v, a;
        for (size_t i = 0; i < count; ++i) {
            optimized.at_time(times[i], p, v, a);
            for (size_t d = 0; d < 2; ++d) {
                CHECK(p[d] == doctest::Approx(input.intermediate_positions[i][d]).epsilon(1e-8));
            }
        }
        check_sampled_limits(input, optimized);
    }
}

TEST_CASE("automatic-curved-path-with-static-axis-and-directional-limits") {
    Ruckig<> otg(3);
    auto input = make_input<0>(3);
    input.target_position = {-1.0, 1.0, 0.0};
    input.min_position = {-2.0, -2.0, 0.0};
    input.min_velocity = std::vector<double>{-1.5, -0.75, 0.0};
    input.min_acceleration = std::vector<double>{-1.0, -0.75, 0.0};
    input.max_velocity[2] = input.max_acceleration[2] = input.max_jerk[2] = 0.0;
    for (size_t i = 1; i < 20; ++i) {
        const double angle = std::acos(-1.0) * i / 40;
        input.intermediate_positions.push_back({-std::sin(angle), 1.0 - std::cos(angle), 0.0});
    }
    Trajectory<> stopping(3), optimized(3);
    REQUIRE(otg.calculate_waypoints(input, stopping) == Result::Working);
    REQUIRE(otg.optimize_waypoints(input, optimized) == Result::Working);
    CHECK(optimized.get_duration() < stopping.get_duration());
    check_sampled_limits(input, optimized);
}

TEST_CASE("repeating-curved-points-does-not-change-the-selected-motion") {
    Ruckig<3> otg;
    auto input = make_input<3>();
    input.target_position = {1.0, 1.0, -0.5};
    for (size_t i = 1; i < 20; ++i) {
        const double angle = std::acos(-1.0) * i / 40;
        input.intermediate_positions.push_back({std::sin(angle), 1.0 - std::cos(angle), -0.5 * std::sin(angle)});
    }
    Trajectory<3> compact, expanded;
    REQUIRE(otg.optimize_waypoints(input, compact) == Result::Working);
    const auto points = input.intermediate_positions;
    for (size_t copies: {2, 5}) {
        input.intermediate_positions.clear();
        for (const auto& point: points) {
            for (size_t i = 0; i < copies; ++i) {
                input.intermediate_positions.push_back(point);
            }
        }
        REQUIRE(otg.optimize_waypoints(input, expanded) == Result::Working);
        CHECK(expanded.get_duration() == doctest::Approx(compact.get_duration()).epsilon(1e-8));
        const auto times = expanded.get_intermediate_durations();
        REQUIRE(times.size() == copies * points.size() + 1);
        std::array<double, 3> p, v, a;
        for (size_t i = 0; i < points.size(); ++i) {
            for (size_t copy = 0; copy < copies; ++copy) {
                CHECK(times[i * copies + copy] == times[i * copies]);
            }
            expanded.at_time(times[i * copies], p, v, a);
            for (size_t d = 0; d < 3; ++d) {
                CHECK(p[d] == doctest::Approx(points[i][d]).epsilon(1e-8));
            }
        }
        check_sampled_limits(input, expanded);
    }
}

TEST_CASE("curved-seeds-retain-the-existing-refinement-result") {
    Ruckig<2> otg;
    auto input = make_input<2>();
    input.min_position = {-10.0, -10.0};
    input.max_position = {10.0, 10.0};
    input.target_position = {-1.9081063959308153, 2.8159313174375047};
    input.intermediate_positions = {
        {0.09030236784863338, 1.0180337400898734},
        {0.0003459426247829356, 1.8623312038147626},
        {-0.38127533656138, 2.4255174673022637},
        {-1.0545614697098558, 2.707592530552376},
    };
    Trajectory<2> trajectory;
    REQUIRE(otg.optimize_waypoints(input, trajectory) == Result::Working);
    CHECK(trajectory.get_duration() < 4.007);
    check_sampled_limits(input, trajectory);
}

TEST_CASE("automatic-endpoint-waypoints-reuse-prescribed-states") {
    Ruckig<2> otg;
    for (const auto ends: {std::array<bool, 2>{true, false}, {false, true}, {true, true}}) {
        auto input = make_input<2>();
        input.target_position = {3.0, 0.0};
        input.min_position = {-2.0, -2.0};
        input.max_position = {5.0, 2.0};
        input.intermediate_positions = {{1.0, 1.0}, {2.0, 0.5}};
        if (ends[0]) {
            input.current_velocity = {0.6, 0.0};
            input.current_acceleration = {0.2, 0.0};
        }
        if (ends[1]) {
            input.target_velocity = {0.4, 0.0};
            input.target_acceleration = {-0.1, 0.0};
        }
        Trajectory<2> reference, repeated;
        REQUIRE(otg.optimize_waypoints(input, reference) == Result::Working);
        const size_t prefix = ends[0] ? 3 : 0, suffix = ends[1] ? 4 : 0;
        input.intermediate_positions = {{1.0, 1.0}, {1.0, 1.0}, {2.0, 0.5}};
        input.intermediate_positions.insert(input.intermediate_positions.begin(), prefix, input.current_position);
        input.intermediate_positions.insert(input.intermediate_positions.end(), suffix, input.target_position);
        REQUIRE(otg.optimize_waypoints(input, repeated) == Result::Working);
        CHECK(repeated.get_duration() == doctest::Approx(reference.get_duration()).epsilon(1e-8));
        const auto times = repeated.get_intermediate_durations();
        REQUIRE(times.size() == input.intermediate_positions.size() + 1);
        CHECK(repeated.get_profiles().size() == times.size());
        for (size_t i = 0; i < prefix; ++i) {
            CHECK(times[i] == 0.0);
        }
        CHECK(times[prefix] == times[prefix + 1]);
        for (size_t i = times.size() - suffix - 1; i < times.size(); ++i) {
            CHECK(times[i] == repeated.get_duration());
        }
        std::array<double, 2> p, v, a, expected_p, expected_v, expected_a;
        repeated.at_time(0.0, p, v, a);
        CHECK(p == input.current_position);
        CHECK(v == input.current_velocity);
        CHECK(a == input.current_acceleration);
        for (size_t sample = 0; sample <= 40; ++sample) {
            const double time = reference.get_duration() * sample / 40.0;
            reference.at_time(time, expected_p, expected_v, expected_a);
            repeated.at_time(time, p, v, a);
            for (size_t d = 0; d < 2; ++d) {
                CHECK(p[d] == doctest::Approx(expected_p[d]).epsilon(1e-8));
                CHECK(v[d] == doctest::Approx(expected_v[d]).epsilon(1e-8));
                CHECK(a[d] == doctest::Approx(expected_a[d]).epsilon(1e-8));
            }
        }
        for (size_t d = 0; d < 2; ++d) {
            CHECK(p[d] == doctest::Approx(input.target_position[d]).epsilon(1e-8));
            CHECK(v[d] == doctest::Approx(input.target_velocity[d]).epsilon(1e-8));
            CHECK(a[d] == doctest::Approx(input.target_acceleration[d]).epsilon(1e-8));
        }
        check_sampled_limits(input, repeated);

        Trajectory<2> stopping, zero_iterations;
        REQUIRE(otg.calculate_waypoints(input, stopping) == Result::Working);
        REQUIRE(otg.optimize_waypoints(input, zero_iterations, 0) == Result::Working);
        CHECK(zero_iterations.get_duration() == stopping.get_duration());
        CHECK(zero_iterations.get_intermediate_durations() == stopping.get_intermediate_durations());
        if (prefix > 0) {
            CHECK(zero_iterations.get_intermediate_durations().front() > 0.0);
        }
    }
}

TEST_CASE("automatic-coincident-waypoints-keep-endpoint-state-transitions") {
    Ruckig<1> otg;
    for (bool change_velocity: {true, false}) {
        auto input = make_input<1>();
        input.target_position = input.current_position;
        input.min_position = {-2.0};
        input.max_position = {2.0};
        if (change_velocity) {
            input.current_velocity = {0.5};
            input.target_velocity = {-0.5};
        } else {
            input.current_acceleration = {0.5};
            input.target_acceleration = {-0.5};
        }
        Trajectory<1> reference, repeated;
        REQUIRE(otg.calculate(input, reference) == Result::Working);
        input.intermediate_positions = {{0.0}, {0.0}, {0.0}};
        REQUIRE(otg.optimize_waypoints(input, repeated) == Result::Working);
        CHECK(repeated.get_duration() == doctest::Approx(reference.get_duration()));
        CHECK(repeated.get_duration() > 0.0);
        const auto times = repeated.get_intermediate_durations();
        REQUIRE(times.size() == 4);
        CHECK(times == std::vector<double>{0.0, 0.0, 0.0, repeated.get_duration()});
        double p = 0.0, v = 0.0, a = 0.0;
        repeated.at_time(0.0, p, v, a);
        CHECK(p == input.current_position[0]);
        CHECK(v == input.current_velocity[0]);
        CHECK(a == input.current_acceleration[0]);
        repeated.at_time(repeated.get_duration(), p, v, a);
        CHECK(p == doctest::Approx(input.target_position[0]));
        CHECK(v == doctest::Approx(input.target_velocity[0]));
        CHECK(a == doctest::Approx(input.target_acceleration[0]));
        check_sampled_limits(input, repeated);
    }
}

TEST_CASE("automatic-endpoint-waypoints-preserve-initial-braking") {
    RuckigThrow<2> otg;
    Ruckig<2> evaluator;
    for (const auto state: {std::array<double, 2>{3.0, 0.0}, {0.0, 3.0}, {3.0, 3.0}, {1.9, 1.0}}) {
        auto input = make_input<2>();
        input.current_velocity = {state[0], 0.0};
        input.current_acceleration = {state[1], 0.0};
        input.target_position = {3.0, 0.0};
        input.min_position = {-10.0, -10.0};
        input.max_position = {10.0, 10.0};
        input.intermediate_positions = {{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.5}};
        std::vector<std::array<double, 2>> velocities(3, {0.0, 0.0}), accelerations(3, {0.0, 0.0});
        velocities[0] = input.current_velocity;
        accelerations[0] = input.current_acceleration;
        Trajectory<2> invalid, stopping, optimized;
        CHECK(evaluator.calculate_waypoints(input, invalid, velocities, accelerations) == Result::ErrorInvalidInput);
        REQUIRE(otg.calculate_waypoints(input, stopping) == Result::Working);
        REQUIRE(otg.optimize_waypoints(input, optimized) == Result::Working);
        CHECK(optimized.get_duration() <= stopping.get_duration());
        const auto times = optimized.get_intermediate_durations();
        REQUIRE(times.size() == 4);
        CHECK(times.front() == stopping.get_intermediate_durations().front());
        CHECK(times.front() > 0.0);
        CHECK(optimized.get_profiles().front()[0].brake.duration > 0.0);
        std::array<double, 2> p, v, a, j;
        size_t section = 0;
        for (size_t sample = 0; sample <= 200; ++sample) {
            const double time = optimized.get_duration() * sample / 200.0;
            optimized.at_time(time, p, v, a, j, section);
            for (size_t d = 0; d < 2; ++d) {
                CHECK(p[d] >= input.min_position[d] - 1e-8);
                CHECK(p[d] <= input.max_position[d] + 1e-8);
                CHECK(std::abs(j[d]) <= input.max_jerk[d] + 1e-8);
                if (time >= times.front()) {
                    CHECK(std::abs(v[d]) <= input.max_velocity[d] + 1e-8);
                    CHECK(std::abs(a[d]) <= input.max_acceleration[d] + 1e-8);
                }
            }
        }
    }
}

TEST_CASE("local-filter-one-axis-extrema-and-finite-guards") {
    Ruckig<1> otg;
    InputParameter<1> input;
    input.current_position = {0.0};
    input.target_position = {4.0};
    input.intermediate_positions = {{1.0}, {2.0}, {1.0}, {3.0}};
    CHECK(otg.filter_intermediate_positions(input, {0.0}) == std::vector<std::array<double, 1>>{{2.0}, {1.0}});
    CHECK(otg.filter_intermediate_positions(input, {1.0}).empty());
    input.target_position = {-4.0};
    input.intermediate_positions = {{-1.0}, {-2.0}, {-1.0}, {-3.0}};
    CHECK(otg.filter_intermediate_positions(input, {0.0}) == std::vector<std::array<double, 1>>{{-2.0}, {-1.0}});
    input.target_position = {1.0};
    input.intermediate_positions = {{0.0}, {0.0}, {1.0}, {1.0}};
    CHECK(otg.filter_intermediate_positions(input, {0.0}).empty());

    const double tiny = std::numeric_limits<double>::denorm_min();
    const double huge = std::numeric_limits<double>::max();
    input.target_position = {tiny};
    input.intermediate_positions = {{0.0}};
    CHECK(otg.filter_intermediate_positions(input, {0.0}).empty());
    CHECK(otg.filter_intermediate_positions(input, {tiny}).empty());
    // A nonfinite normalized radius conservatively retains the original point.
    CHECK(otg.filter_intermediate_positions(input, {1.0}) == input.intermediate_positions);
    input.intermediate_positions = {{huge}};
    CHECK(otg.filter_intermediate_positions(input, {0.0}) == input.intermediate_positions);
    input.current_position = {-huge};
    input.target_position = {huge};
    input.intermediate_positions = {{0.0}};
    CHECK(otg.filter_intermediate_positions(input, {0.0}) == input.intermediate_positions);
    input.current_position = {huge};
    input.target_position = {huge};
    input.intermediate_positions = {{-huge}};
    CHECK(otg.filter_intermediate_positions(input, {huge}) == input.intermediate_positions);
}

TEST_CASE("local-filter-long-one-axis-route") {
    Ruckig<> otg(1);
    InputParameter<> input(1);
    input.current_position = {0.0};
    input.target_position = {10001.0};
    input.intermediate_positions.reserve(10000);
    for (size_t i = 1; i <= 10000; ++i) {
        input.intermediate_positions.push_back({static_cast<double>(i)});
    }
    CHECK(otg.filter_intermediate_positions(input, {0.0}).empty());
    CHECK(otg.filter_intermediate_positions(input, {0.25}).empty());
}
