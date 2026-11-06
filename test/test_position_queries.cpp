#include <doctest/doctest.h>

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>
#include <vector>

#include <ruckig/ruckig.hpp>

using namespace ruckig;

namespace {
template<class T>
concept PublicRawPositionQuery = requires(const T& trajectory) {
    trajectory.template get_first_time_at_position_impl<false>(0, 0.0, 0.0);
};

template<class T>
concept PublicPositionQueryBoundsFlag = requires(const T& trajectory) {
    trajectory.template get_first_time_at_position<false>(0, 0.0, 0.0);
};

using PositionQuery = std::optional<double> (Trajectory<1>::*)(size_t, double, double) const;
static_assert(std::is_same_v<decltype(&Trajectory<1>::get_first_time_at_position), PositionQuery>);
static_assert(!PublicRawPositionQuery<Trajectory<1>> && !PublicRawPositionQuery<Trajectory<0>>);
static_assert(!PublicPositionQueryBoundsFlag<Trajectory<1>>);

InputParameter<1> position_query_input() {
    InputParameter<1> input;
    input.current_position = {0.0};
    input.target_position = {3.0};
    input.max_velocity = input.max_acceleration = {2.0};
    input.max_jerk = {1.0};
    return input;
}
} // namespace

TEST_CASE("position query windows preserve native half-ULP waypoint prefixes") {
    const double prefix = std::ldexp(1.0, -53);
    const double after = std::nextafter(1.0, std::numeric_limits<double>::infinity());
    for (double speed: {1.0, std::ldexp(1.0, 23)}) {
        INFO("speed: " << speed);
        auto input = position_query_input();
        input.current_position = {-speed * prefix};
        input.target_position = {2.0 * speed};
        input.current_velocity = input.target_velocity = {speed};
        input.max_velocity = {2.0 * speed};
        input.max_jerk = {0.0};
        input.intermediate_positions = {{0.0}, {speed}};
        const std::vector<std::array<double, 1>> velocities(2, {speed}), accelerations(2, {0.0});
        Ruckig<1> solver;
        Trajectory<1> trajectory;
        REQUIRE(solver.calculate_waypoints(input, trajectory, velocities, accelerations) == Result::Working);
        const std::vector<double> expected_ends {prefix, 1.0, 2.0};
        CHECK(trajectory.get_intermediate_durations() == expected_ends);

        // Local subtraction can round to a root whose global sum is below after.
        // Accept the clamped time only if its sampled position still matches.
        double sampled_position;
        trajectory.at_time(after, sampled_position);
        const auto crossing = trajectory.get_first_time_at_position(0, speed, after);
        if (speed == 1.0) {
            CHECK(std::abs(sampled_position - speed) < 1e-9);
            REQUIRE(crossing.has_value());
            CHECK(*crossing == after);
        } else {
            CHECK(std::abs(sampled_position - speed) > 1e-9);
            CHECK_FALSE(crossing.has_value());
        }
    }
}

TEST_CASE("position query windows keep native terminal matches in the closed interval") {
    auto input = position_query_input();
    input.intermediate_positions = {{1.0}, {2.0}};
    Ruckig<1> solver;
    Trajectory<1> trajectory;
    REQUIRE(solver.calculate_waypoints(input, trajectory) == Result::Working);
    const double duration = trajectory.get_duration();
    double terminal_position;
    trajectory.at_time(duration, terminal_position);
    const auto terminal = trajectory.get_first_time_at_position(0, terminal_position, duration);
    REQUIRE(terminal.has_value());
    CHECK(*terminal == duration);
    CHECK_FALSE(trajectory.get_first_time_at_position(0, terminal_position,
        std::nextafter(duration, std::numeric_limits<double>::infinity())).has_value());
    CHECK_FALSE(trajectory.get_first_time_at_position(0, terminal_position,
        std::numeric_limits<double>::max()).has_value());
    const double after = std::nextafter(duration, 0.0);
    const auto adjacent = trajectory.get_first_time_at_position(0, terminal_position, after);
    REQUIRE(adjacent.has_value());
    CHECK(*adjacent >= after);
    CHECK(*adjacent <= duration);
    CHECK(trajectory.get_first_time_at_position(0, 0.0, -1.0) == std::optional<double>(0.0));
}

TEST_CASE("position queries reject unavailable trajectories and invalid arguments") {
    Ruckig<1> solver;
    Trajectory<1> trajectory;
    CHECK_FALSE(trajectory.get_first_time_at_position(0, 0.0).has_value());
    auto input = position_query_input();
    REQUIRE(solver.calculate(input, trajectory) == Result::Working);
    CHECK_FALSE(trajectory.get_first_time_at_position(1, 0.0).has_value());
    for (double invalid: {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
        CHECK_FALSE(trajectory.get_first_time_at_position(0, invalid).has_value());
        CHECK_FALSE(trajectory.get_first_time_at_position(0, 0.0, invalid).has_value());
    }
    input.max_velocity = {-1.0};
    REQUIRE(solver.calculate(input, trajectory) == Result::ErrorInvalidInput);
    CHECK_FALSE(trajectory.get_first_time_at_position(0, 0.0).has_value());
}

TEST_CASE("position query windows include the origin of zero-duration trajectories") {
    auto input = position_query_input();
    input.target_position = {0.0};
    Ruckig<1> solver;
    Trajectory<1> trajectory;
    REQUIRE(solver.calculate_waypoints(input, trajectory) == Result::Working);
    REQUIRE(trajectory.get_duration() == 0.0);
    CHECK(trajectory.get_first_time_at_position(0, 0.0, -1.0) == std::optional<double>(0.0));
    CHECK(trajectory.get_first_time_at_position(0, 0.0, 0.0) == std::optional<double>(0.0));
    CHECK_FALSE(trajectory.get_first_time_at_position(0, 0.0,
        std::nextafter(0.0, std::numeric_limits<double>::infinity())).has_value());
}
