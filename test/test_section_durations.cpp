#include <doctest/doctest.h>

#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

#include <ruckig/ruckig.hpp>

using namespace ruckig;

namespace {
template<size_t D>
auto duration_input() {
    auto input = [] {
        if constexpr (D == 0) return InputParameter<D>(2);
        else return InputParameter<D>();
    }();
    for (size_t d = 0; d < input.degrees_of_freedom; ++d) {
        const double scale = static_cast<double>(d + 1);
        input.current_position[d] = 0.0;
        input.target_position[d] = 3.0 * scale;
        input.max_velocity[d] = input.max_acceleration[d] = 2.0 * scale;
        input.max_jerk[d] = 4.0 * scale;
        input.min_position[d] = -20.0 * scale;
        input.max_position[d] = 20.0 * scale;
    }
    return input;
}

template<size_t D, bool Throw = false>
auto duration_otg() {
    if constexpr (D == 0) return Ruckig<D, StandardVector, Throw>(2, 0.01);
    else return Ruckig<D, StandardVector, Throw>(0.01);
}

template<size_t D>
auto duration_trajectory() {
    if constexpr (D == 0) return Trajectory<D>(2);
    else return Trajectory<D>();
}

template<size_t D>
void duration_points(InputParameter<D>& input, const std::vector<double>& positions) {
    input.intermediate_positions.clear();
    for (double position: positions) {
        auto point = input.current_position;
        for (size_t d = 0; d < input.degrees_of_freedom; ++d) point[d] = position * static_cast<double>(d + 1);
        input.intermediate_positions.push_back(point);
    }
}

// Re-solve each original interval with the public scalar minimum-duration API.
// Target metadata preserves the chosen states without sampling roundoff.
template<size_t D>
void check_duration_oracles(const InputParameter<D>& input, const Trajectory<D>& trajectory) {
    const auto profiles = trajectory.get_profiles();
    const auto times = trajectory.get_intermediate_durations();
    const size_t count = input.intermediate_positions.size();
    REQUIRE(profiles.size() == count + 1);
    REQUIRE(times.size() == count + 1);
    auto scalar = input;
    scalar.intermediate_positions.clear();
    scalar.per_section_minimum_duration.reset();
    auto solver = duration_otg<D>();
    auto oracle = duration_trajectory<D>();
    double prefix = 0.0;
    for (size_t i = 0; i <= count; ++i) {
        INFO("original interval: " << i);
        if (input.per_section_minimum_duration) scalar.minimum_duration = (*input.per_section_minimum_duration)[i];
        scalar.target_position = i == count ? input.target_position : input.intermediate_positions[i];
        for (size_t d = 0; d < input.degrees_of_freedom; ++d) {
            scalar.target_velocity[d] = profiles[i][d].vf;
            scalar.target_acceleration[d] = profiles[i][d].af;
        }
        REQUIRE(solver.calculate(scalar, oracle) == Result::Working);
        if (input.per_section_minimum_duration) CHECK(oracle.get_duration() >= (*input.per_section_minimum_duration)[i]);
        const double start = prefix;
        prefix += oracle.get_duration();
        // Exact repeated addition is the stored cumulative-time contract.
        CHECK(times[i] == prefix);
        const auto oracle_profiles = oracle.get_profiles();
        for (size_t d = 0; d < input.degrees_of_freedom; ++d) {
            CHECK(profiles[i][d].t == oracle_profiles[0][d].t);
            CHECK(profiles[i][d].t_sum == oracle_profiles[0][d].t_sum);
            CHECK(profiles[i][d].brake.duration == oracle_profiles[0][d].brake.duration);
        }
        for (double fraction: {0.0, 0.37, 1.0}) {
            auto actual_p = input.current_position, actual_v = actual_p, actual_a = actual_p;
            auto expected_p = actual_p, expected_v = actual_p, expected_a = actual_p;
            trajectory.at_time(start + fraction * oracle.get_duration(), actual_p, actual_v, actual_a);
            oracle.at_time(fraction * oracle.get_duration(), expected_p, expected_v, expected_a);
            for (size_t d = 0; d < input.degrees_of_freedom; ++d) {
                CHECK(actual_p[d] == doctest::Approx(expected_p[d]).epsilon(1e-9));
                CHECK(actual_v[d] == doctest::Approx(expected_v[d]).epsilon(1e-9));
                CHECK(actual_a[d] == doctest::Approx(expected_a[d]).epsilon(1e-9));
            }
        }
        scalar.current_position = scalar.target_position;
        scalar.current_velocity = scalar.target_velocity;
        scalar.current_acceleration = scalar.target_acceleration;
    }
    CHECK(trajectory.get_duration() == prefix);
}

template<size_t D>
void fixed_duration_cases() {
    auto input = duration_input<D>();
    duration_points(input, {1.0, 2.0});
    input.per_section_minimum_duration = std::vector<double>{3.2, 0.4, 4.1};
    auto otg = duration_otg<D>();
    auto trajectory = duration_trajectory<D>();
    REQUIRE(otg.calculate_waypoints(input, trajectory) == Result::Working);
    check_duration_oracles(input, trajectory);

    input = duration_input<D>();
    duration_points(input, {0.0, 1.0, 1.0, 3.0});
    input.per_section_minimum_duration = std::vector<double>{2.5, 2.0, 3.2, 2.0, 2.1};
    auto v = input.current_position, a = v;
    for (size_t d = 0; d < input.degrees_of_freedom; ++d) {
        v[d] = 0.25 * static_cast<double>(d + 1);
        a[d] = 0.1 * static_cast<double>(d + 1);
    }
    input.current_velocity = input.target_velocity = v;
    input.current_acceleration = input.target_acceleration = a;
    const std::vector<decltype(v)> velocities(4, v), accelerations(4, a);
    REQUIRE(otg.calculate_waypoints(input, trajectory, velocities, accelerations) == Result::Working);
    check_duration_oracles(input, trajectory);
    const auto times = trajectory.get_intermediate_durations();
    CHECK(times[0] > 0.0);
    CHECK(times[2] > times[1]);
    CHECK(times.back() > times[3]);

    // Omitted timing remains the ordinary local scalar-chain behavior.
    input = duration_input<D>();
    duration_points(input, {1.0, 2.0});
    REQUIRE(otg.calculate_waypoints(input, trajectory) == Result::Working);
    check_duration_oracles(input, trajectory);
}

template<size_t D>
void automatic_duration_cases() {
    for (size_t geometry = 0; geometry < 3; ++geometry) {
        auto input = duration_input<D>();
        if (geometry == 0) {
            duration_points(input, {0.0, 1.0, 1.0, 2.0, 3.0});
            input.per_section_minimum_duration = std::vector<double>{0.5, 0.4, 3.0, 0.8, 0.4, 1.5};
        } else if (geometry == 1) {
            duration_points(input, {0.5, 1.0, 2.0});
            input.per_section_minimum_duration = std::vector<double>{2.2, 0.2, 2.3, 0.2};
        } else {
            duration_points(input, {1.0, 2.0, 2.5});
            if (input.degrees_of_freedom > 1) {
                input.intermediate_positions[0][1] = 0.5;
                input.intermediate_positions[1][1] = -0.5;
                input.intermediate_positions[2][1] = 0.25;
            }
            input.per_section_minimum_duration = std::vector<double>{0.2, 2.5, 0.2, 0.8};
        }
        auto otg = duration_otg<D>();
        auto baseline = duration_trajectory<D>();
        REQUIRE(otg.calculate_waypoints(input, baseline) == Result::Working);
        for (size_t iterations: {size_t(0), size_t(1), size_t(8)}) {
            INFO("geometry: " << geometry << ", iterations: " << iterations);
            auto trajectory = duration_trajectory<D>();
            REQUIRE(otg.optimize_waypoints(input, trajectory, iterations) == Result::Working);
            CHECK(trajectory.get_duration() <= baseline.get_duration());
            CHECK(trajectory.get_duration() >= std::accumulate(input.per_section_minimum_duration->begin(), input.per_section_minimum_duration->end(), 0.0));
            check_duration_oracles(input, trajectory);
            if (geometry == 0) {
                const auto times = trajectory.get_intermediate_durations();
                CHECK(times[0] > 0.0);
                CHECK(times[2] > times[1]);
                CHECK(times.back() > times[4]);
            }
        }
    }
}

template<size_t D, bool Throw>
void invalid_duration_cases() {
    auto input = duration_input<D>();
    duration_points(input, {1.0, 2.0});
    const std::vector<std::vector<double>> invalid {
        {}, {1.0}, {1.0, 1.0}, {1.0, 1.0, 1.0, 1.0}, {-0.1, 0.0, 0.0},
        {0.0, std::numeric_limits<double>::quiet_NaN(), 0.0},
        {0.0, std::numeric_limits<double>::infinity(), 0.0},
        {0.0, -std::numeric_limits<double>::infinity(), 0.0},
    };
    auto otg = duration_otg<D, Throw>();
    auto trajectory = duration_trajectory<D>();
    for (bool optimize: {false, true}) {
        for (const auto& minima: invalid) {
            input.per_section_minimum_duration.reset();
            REQUIRE(otg.calculate_waypoints(input, trajectory) == Result::Working);
            input.per_section_minimum_duration = minima;
            if constexpr (Throw) {
                CHECK_THROWS_AS(optimize ? otg.optimize_waypoints(input, trajectory) : otg.calculate_waypoints(input, trajectory), RuckigError);
            } else {
                CHECK((optimize ? otg.optimize_waypoints(input, trajectory) : otg.calculate_waypoints(input, trajectory)) == Result::ErrorInvalidInput);
            }
            auto position = input.current_position;
            CHECK_THROWS_AS(trajectory.at_time(0.0, position), RuckigError);
            CHECK_THROWS_AS(trajectory.get_profiles(), RuckigError);
            input.per_section_minimum_duration = std::vector<double>{0.0, 0.0, 0.0};
            REQUIRE(otg.calculate_waypoints(input, trajectory) == Result::Working);
        }
    }
    input.intermediate_positions.clear();
    for (const auto& minima: std::vector<std::vector<double>>{{}, {0.0, 0.0}, {-0.1}, {std::numeric_limits<double>::quiet_NaN()}, {std::numeric_limits<double>::infinity()}}) {
        input.per_section_minimum_duration = minima;
        if constexpr (Throw) CHECK_THROWS_AS(otg.calculate_waypoints(input, trajectory), RuckigError);
        else CHECK(otg.calculate_waypoints(input, trajectory) == Result::ErrorInvalidInput);
    }
}

template<size_t D>
void endpoint_duration_cases() {
    auto input = duration_input<D>();
    input.per_section_minimum_duration = std::vector<double>{4.0};
    auto otg = duration_otg<D>();
    for (double global: {0.0, 2.0, 5.0, std::numeric_limits<double>::quiet_NaN()}) {
        input.minimum_duration = global;
        auto scalar = input;
        scalar.per_section_minimum_duration.reset();
        scalar.minimum_duration = std::max(4.0, global);
        auto expected = duration_trajectory<D>();
        auto actual = duration_trajectory<D>();
        REQUIRE(otg.calculate(scalar, expected) == Result::Working);
        for (bool optimize: {false, true}) {
            REQUIRE((optimize ? otg.optimize_waypoints(input, actual) : otg.calculate_waypoints(input, actual)) == Result::Working);
            CHECK(actual.get_duration() == expected.get_duration());
            CHECK(actual.get_intermediate_durations() == expected.get_intermediate_durations());
        }
    }
    input.minimum_duration.reset();
    input.duration_discretization = DurationDiscretization::Discrete;
    input.per_section_minimum_duration = std::vector<double>{4.015};
    auto expected = duration_trajectory<D>();
    auto actual = duration_trajectory<D>();
    auto scalar = input;
    scalar.per_section_minimum_duration.reset();
    scalar.minimum_duration = 4.015;
    REQUIRE(otg.calculate(scalar, expected) == Result::Working);
    REQUIRE(otg.calculate_waypoints(input, actual) == Result::Working);
    CHECK(actual.get_duration() == expected.get_duration());
}
} // namespace

TEST_CASE("per-section minima match independent scalar interval oracles") {
    fixed_duration_cases<2>();
    fixed_duration_cases<0>();
}

TEST_CASE("automatic state selection preserves every original interval minimum") {
    automatic_duration_cases<2>();
    automatic_duration_cases<0>();
}

TEST_CASE("per-section minima validate values and invalidate failed trajectories") {
    invalid_duration_cases<2, false>();
    invalid_duration_cases<2, true>();
    invalid_duration_cases<0, false>();
    invalid_duration_cases<0, true>();
}

TEST_CASE("single interval combines per-section and global minima") {
    endpoint_duration_cases<2>();
    endpoint_duration_cases<0>();
}

TEST_CASE("stored section ends preserve scalar timing despite one ULP subtraction") {
    auto input = duration_input<1>();
    input.target_position = {0.0};
    input.intermediate_positions = {{0.0}};
    input.per_section_minimum_duration = std::vector<double>{0.01, 0.02};
    auto otg = duration_otg<1>();
    Trajectory<1> trajectory;
    REQUIRE(otg.calculate_waypoints(input, trajectory) == Result::Working);
    check_duration_oracles(input, trajectory);
    const auto times = trajectory.get_intermediate_durations();
    CHECK(times[0] == 0.01);
    CHECK(times[1] == 0.01 + 0.02);
    // Subtraction rounding is independent of each scalar duration guarantee.
    CHECK(times[1] - times[0] == std::nextafter(0.02, 0.0));
}

TEST_CASE("positive interval minima cannot disappear in cumulative rounding") {
    auto input = duration_input<1>();
    input.target_position = {0.0};
    input.intermediate_positions = {{0.0}};
    input.per_section_minimum_duration = std::vector<double>{1.0, 1e-20};
    auto otg = duration_otg<1>();
    Trajectory<1> trajectory;
    REQUIRE(otg.calculate_waypoints(input, trajectory) == Result::ErrorTrajectoryDuration);
    CHECK_THROWS_AS(trajectory.get_profiles(), RuckigError);
    const auto result = otg.optimize_waypoints(input, trajectory);
    REQUIRE((result == Result::Working || result == Result::ErrorTrajectoryDuration));
    if (result == Result::Working) {
        const auto times = trajectory.get_intermediate_durations();
        CHECK(times[0] > 0.0);
        CHECK(times[1] > times[0]);
    }
}

TEST_CASE("positive repeated-position minima do not imply a stationary wait") {
    auto input = duration_input<1>();
    input.target_position = {0.0};
    input.current_velocity = input.target_velocity = {0.5};
    input.max_jerk = {0.0};
    auto otg = duration_otg<1>();
    Trajectory<1> oracle, trajectory;
    input.minimum_duration = 1.0;
    REQUIRE(otg.calculate(input, oracle) == Result::ErrorZeroLimits);
    input.minimum_duration.reset();
    input.intermediate_positions = {{0.0}};
    const std::vector<std::array<double, 1>> velocities {{0.5}};
    REQUIRE(otg.calculate_waypoints(input, trajectory, velocities) == Result::Working);
    CHECK(trajectory.get_duration() == 0.0);
    input.per_section_minimum_duration = std::vector<double>{1.0, 1.0};
    CHECK(otg.calculate_waypoints(input, trajectory, velocities) == Result::ErrorZeroLimits);
    CHECK_THROWS_AS(trajectory.get_profiles(), RuckigError);
    CHECK(otg.optimize_waypoints(input, trajectory) == Result::ErrorZeroLimits);
}
