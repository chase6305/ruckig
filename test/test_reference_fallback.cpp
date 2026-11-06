#include <doctest/doctest.h>

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <ruckig/ruckig.hpp>

using namespace ruckig;

namespace {
template<size_t D>
auto reconstruction_input(const std::vector<double>& times) {
    auto input = [] {
        if constexpr (D == 0) return InputParameter<D>(2);
        else return InputParameter<D>();
    }();
    for (size_t d = 0; d < 2; ++d) {
        input.current_position[d] = 0.0;
        input.current_velocity[d] = d == 0 ? -0.525 : 0.275;
        input.current_acceleration[d] = d == 0 ? 0.5 : -0.25;
        input.target_position[d] = 2.0 * input.current_velocity[d] + 2.0 * input.current_acceleration[d];
        input.target_velocity[d] = input.current_velocity[d] + 2.0 * input.current_acceleration[d];
        input.target_acceleration[d] = input.current_acceleration[d];
        input.max_velocity[d] = input.max_acceleration[d] = 1.0;
        input.max_jerk[d] = 0.0;
        input.min_position[d] = -1.0;
        input.max_position[d] = 1.0;
    }
    for (double time: times) {
        auto point = input.current_position;
        for (size_t d = 0; d < 2; ++d) {
            point[d] = input.current_velocity[d] * time + input.current_acceleration[d] * time * time / 2.0;
        }
        input.intermediate_positions.push_back(point);
    }
    return input;
}

template<size_t D, bool Throw = false>
auto reconstruction_otg() {
    if constexpr (D == 0) return Ruckig<D, StandardVector, Throw>(2, 0.01);
    else return Ruckig<D, StandardVector, Throw>(0.01);
}

template<size_t D>
auto reconstruction_trajectory() {
    if constexpr (D == 0) return Trajectory<D>(2);
    else return Trajectory<D>();
}

template<size_t D>
void check_reconstructed_motion(const InputParameter<D>& input, const Trajectory<D>& trajectory,
    const std::vector<double>& times) {
    const auto ends = trajectory.get_intermediate_durations();
    REQUIRE(ends.size() == times.size() + 1);
    REQUIRE(trajectory.get_profiles().size() == times.size() + 1);
    CHECK(trajectory.get_duration() == 2.0);
    for (size_t i = 0; i < times.size(); ++i) CHECK(ends[i] == times[i]);
    CHECK(ends.back() == 2.0);
    for (const auto& section: trajectory.get_profiles()) {
        for (const auto& profile: section) {
            for (double jerk: profile.j) CHECK(jerk == 0.0);
        }
    }
    std::vector<double> samples {0.0, 0.125, 0.5, 1.0, 1.05, 1.1, 1.5, 2.0};
    for (double time: times) {
        if (time > 0.0) samples.push_back(time - 1e-7);
        samples.push_back(time);
        if (time < 2.0) samples.push_back(time + 1e-7);
    }
    for (double time: samples) {
        auto p = input.current_position, v = p, a = p;
        trajectory.at_time(time, p, v, a);
        for (size_t d = 0; d < 2; ++d) {
            const double expected_p = input.current_velocity[d] * time + input.current_acceleration[d] * time * time / 2.0;
            const double expected_v = input.current_velocity[d] + input.current_acceleration[d] * time;
            CHECK(std::abs(p[d] - expected_p) < 1e-12);
            CHECK(std::abs(v[d] - expected_v) < 1e-12);
            CHECK(std::abs(a[d] - input.current_acceleration[d]) < 1e-12);
        }
    }
}

template<size_t D, bool Throw>
void reconstruction_cases() {
    for (const auto& times: std::vector<std::vector<double>>{{0.5, 1.0}, {0.5, 0.5, 1.0},
        {0.0, 0.0, 0.5, 1.0, 2.0, 2.0}, {0.0, 0.5, 0.5, 1.0, 2.0}}) {
        auto input = reconstruction_input<D>(times);
        auto solver = reconstruction_otg<D, Throw>();
        auto trajectory = reconstruction_trajectory<D>();
        // The constant-acceleration equation supplies an independent feasible oracle.
        std::vector<decltype(input.current_velocity)> velocities(times.size(), input.current_velocity);
        const std::vector<decltype(input.current_acceleration)> accelerations(times.size(), input.current_acceleration);
        for (size_t i = 0; i < times.size(); ++i) {
            for (size_t d = 0; d < 2; ++d) velocities[i][d] += input.current_acceleration[d] * times[i];
        }
        REQUIRE(solver.calculate_waypoints(input, trajectory, velocities, accelerations) == Result::Working);
        check_reconstructed_motion(input, trajectory, times);
        for (size_t iterations: {size_t(1), size_t(8), size_t(64)}) {
            INFO("waypoints: " << times.size() << ", iterations: " << iterations);
            REQUIRE(solver.optimize_waypoints(input, trajectory, iterations) == Result::Working);
            check_reconstructed_motion(input, trajectory, times);
        }
        CHECK(solver.optimize_waypoints(input, trajectory, 0) == Result::ErrorZeroLimits);
    }
}

template<size_t D, bool Throw>
void reconstruction_constraints() {
    const std::vector<double> times {0.5, 1.0};
    auto input = reconstruction_input<D>(times);
    input.min_position[0] = -0.275625 - 2e-8;
    input.max_position[0] = 2e-8;
    input.min_position[1] = -2e-8;
    input.max_position[1] = 0.15125 + 2e-8;
    input.per_section_minimum_duration = std::vector<double>{0.5, 0.5, 1.0};
    auto solver = reconstruction_otg<D, Throw>();
    auto trajectory = reconstruction_trajectory<D>();
    REQUIRE(solver.optimize_waypoints(input, trajectory) == Result::Working);
    check_reconstructed_motion(input, trajectory, times);

    // Both waypoints are inside this bound, but the unique zero-jerk motion
    // reaches y=.15125 between them and the target. A seed cannot bypass it.
    input.max_position[1] = 0.15;
    Result result = Result::Working;
    CHECK_NOTHROW(result = solver.optimize_waypoints(input, trajectory));
    CHECK(result != Result::Working);
    auto p = input.current_position;
    CHECK_THROWS_AS(trajectory.at_time(0.0, p), RuckigError);
    CHECK_THROWS_AS(trajectory.get_profiles(), RuckigError);
    input.max_position[1] = 0.15125 + 2e-8;
    REQUIRE(solver.optimize_waypoints(input, trajectory) == Result::Working);

    // The analytic endpoint states fix total duration at two seconds.
    input.per_section_minimum_duration = std::vector<double>{0.6, 0.5, 1.0};
    CHECK_NOTHROW(result = solver.optimize_waypoints(input, trajectory));
    CHECK(result != Result::Working);
    CHECK_THROWS_AS(trajectory.get_profiles(), RuckigError);

    // Compaction may share a state, but must retain the original timing rows.
    const std::vector<double> repeated_times {0.5, 0.5, 1.0};
    input = reconstruction_input<D>(repeated_times);
    input.per_section_minimum_duration = std::vector<double>{0.5, 0.0, 0.5, 1.0};
    REQUIRE(solver.optimize_waypoints(input, trajectory) == Result::Working);
    check_reconstructed_motion(input, trajectory, repeated_times);
    (*input.per_section_minimum_duration)[1] = 0.01;
    CHECK_NOTHROW(result = solver.optimize_waypoints(input, trajectory));
    CHECK(result != Result::Working);
    CHECK_THROWS_AS(trajectory.get_profiles(), RuckigError);
}

template<size_t D, bool Throw>
void zero_state_error_policy() {
    auto input = reconstruction_input<D>({});
    for (size_t d = 0; d < 2; ++d) {
        input.current_velocity[d] = input.current_acceleration[d] = 0.0;
        input.target_position[d] = 1.0;
        input.target_velocity[d] = input.target_acceleration[d] = 0.0;
        input.max_jerk[d] = 1.0;
        input.max_position[d] = 2.0;
    }
    // Matching the initial point proposes exactly zero intermediate states.
    input.intermediate_positions = {input.current_position};
    auto solver = reconstruction_otg<D, Throw>();
    auto trajectory = reconstruction_trajectory<D>();
    REQUIRE(solver.optimize_waypoints(input, trajectory) == Result::Working);
    input.max_position[0] = 0.9;
    Result result = Result::Working;
    CHECK_NOTHROW(result = solver.optimize_waypoints(input, trajectory));
    CHECK(result == Result::ErrorPositionalLimits);
    CHECK_THROWS_AS(trajectory.get_profiles(), RuckigError);
    input.max_position[0] = 2.0;
    REQUIRE(solver.optimize_waypoints(input, trajectory) == Result::Working);
    input.per_section_minimum_duration = std::vector<double>{0.0, std::numeric_limits<double>::quiet_NaN()};
    if constexpr (Throw) CHECK_THROWS_AS(solver.optimize_waypoints(input, trajectory), RuckigError);
    else CHECK(solver.optimize_waypoints(input, trajectory) == Result::ErrorInvalidInput);
    CHECK_THROWS_AS(trajectory.get_profiles(), RuckigError);
}
} // namespace

TEST_CASE("reference reconstruction recovers zero-jerk motion without changing original intervals") {
    reconstruction_cases<2, false>();
    reconstruction_cases<2, true>();
    reconstruction_cases<0, false>();
    reconstruction_cases<0, true>();
}

TEST_CASE("reference reconstruction preserves bounds and interval minimum durations") {
    reconstruction_constraints<2, false>();
    reconstruction_constraints<2, true>();
    reconstruction_constraints<0, false>();
    reconstruction_constraints<0, true>();
}

TEST_CASE("zero reference states preserve baseline failures and validation policy") {
    zero_state_error_policy<2, false>();
    zero_state_error_policy<2, true>();
    zero_state_error_policy<0, false>();
    zero_state_error_policy<0, true>();
}
