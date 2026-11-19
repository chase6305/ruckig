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

TEST_CASE("position queries retain small derivatives over long native profiles") {
    Ruckig<1> solver;
    for (double sign: {-1.0, 1.0}) {
        for (double jerk: {1e-17, 1e-16, 1e-15}) {
            INFO("sign: " << sign << ", jerk: " << jerk);
            auto input = position_query_input();
            input.target_position = {sign * 1e-7};
            input.max_velocity = input.max_acceleration = {1.0};
            input.max_jerk = {jerk};
            Trajectory<1> trajectory;
            REQUIRE(solver.calculate(input, trajectory) == Result::Working);
            for (double fraction: {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8}) {
                const double time = fraction * trajectory.get_duration();
                double position;
                trajectory.at_time(time, position);
                const auto crossing = trajectory.get_first_time_at_position(0, position);
                REQUIRE(crossing.has_value());
                CHECK(*crossing == doctest::Approx(time).epsilon(1e-10));
                double sampled_position;
                trajectory.at_time(*crossing, sampled_position);
                CHECK(std::abs(sampled_position - position) < 1e-9);
                const auto bounded = trajectory.get_first_time_at_position(0, position, 0.5 * time);
                REQUIRE(bounded.has_value());
                CHECK(*bounded >= 0.5 * time);
                CHECK(*bounded <= trajectory.get_duration());
            }
        }
    }
}

TEST_CASE("small positive velocity discriminants retain interior position extrema and limits") {
    Ruckig<1> solver;
    for (double scale: {1.0, 1000.0}) {
        for (double sign: {-1.0, 1.0}) {
            INFO("scale: " << scale << ", sign: " << sign);
            const double j = sign * 1e-17 * scale, v = -sign * 1e-10 * scale;
            const double duration = 6000.0;
            auto input = position_query_input();
            input.current_velocity = {v};
            std::tie(input.target_position[0], input.target_velocity[0], input.target_acceleration[0]) = integrate(duration, 0.0, v, 0.0, j);
            input.max_velocity = {1e-8 * scale};
            input.max_acceleration = {1e-10 * scale};
            input.max_jerk = {std::abs(j)};
            Trajectory<1> trajectory;
            REQUIRE(solver.calculate(input, trajectory) == Result::Working);
            const double turn = std::sqrt(-2 * v / j);
            const double expected = std::get<0>(integrate(turn, 0.0, v, 0.0, j));
            std::array<Bound, 1> extrema;
            trajectory.get_position_extrema(extrema);
            const auto& bound = extrema[0];
            CHECK((sign > 0.0 ? bound.min : bound.max) == doctest::Approx(expected).epsilon(1e-10).scale(0.0));
            CHECK((sign > 0.0 ? bound.t_min : bound.t_max) == doctest::Approx(turn).epsilon(1e-10));
            const double limit = 0.5 * (expected + input.target_position[0]);
            if (sign > 0.0) input.min_position = {limit};
            else input.max_position = {limit};
            REQUIRE(solver.calculate(input, trajectory) == Result::ErrorPositionalLimits);
            CHECK_THROWS_AS(trajectory.at_time(turn, input.current_position), RuckigError);
            CHECK_NOTHROW(trajectory.get_position_extrema(extrema));
        }
    }
}

TEST_CASE("position extrema keep a turning point lost by quadratic cancellation") {
    for (double sign: {-1.0, 1.0}) {
        Bound bound {std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), 0.0, 0.0};
        Profile::check_step_for_position_extremum(0.0, 1e-15, 0.0, -sign * 1e16, sign * 1e32, sign, bound);
        CHECK((sign > 0.0 ? bound.min : bound.max) == doctest::Approx(-sign * 0.5));
        CHECK((sign > 0.0 ? bound.t_min : bound.t_max) == doctest::Approx(1e-16).scale(0.0));
    }
}

TEST_CASE("position extrema retain turning points after discriminant overflow or underflow") {
    for (double scale: {1e-200, 1e200}) {
        for (double sign: {-1.0, 1.0}) {
            Bound bound {std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), 0.0, 0.0};
            Profile::check_step_for_position_extremum(0.0, 2.0, 0.0, -sign * scale, sign * scale, sign * scale, bound);
            const double expected = sign * scale * (4.0 / 3.0 - std::sqrt(3.0));
            CHECK((sign > 0.0 ? bound.min : bound.max) == doctest::Approx(expected).epsilon(1e-12).scale(0.0));
            CHECK((sign > 0.0 ? bound.t_min : bound.t_max) == doctest::Approx(std::sqrt(3.0) - 1.0));
        }
    }
}

TEST_CASE("position extrema include a brake with only its second phase active") {
    Ruckig<1> solver;
    for (double sign: {-1.0, 1.0}) {
        InputParameter<1> input;
        input.current_position = {0.0};
        input.current_velocity = {2.0 * sign};
        input.current_acceleration = {-sign};
        input.target_position = {3.0 * sign};
        input.max_velocity = input.max_acceleration = input.max_jerk = {1.0};
        Trajectory<1> trajectory;
        REQUIRE(solver.calculate(input, trajectory) == Result::Working);
        const auto brake = trajectory.get_profiles()[0][0].brake;
        REQUIRE(brake.t[0] == 0.0);
        REQUIRE(brake.t[1] > 0.0);
        std::array<Bound, 1> extrema;
        trajectory.get_position_extrema(extrema);
        CHECK((sign > 0.0 ? extrema[0].min : extrema[0].max) == 0.0);
        CHECK((sign > 0.0 ? extrema[0].t_min : extrema[0].t_max) == 0.0);
        const auto crossing = trajectory.get_first_time_at_position(0, sign);
        REQUIRE(crossing.has_value());
        CHECK(*crossing == doctest::Approx(2.0 - std::sqrt(2.0)));
        if (sign > 0.0) input.min_position = {0.5};
        else input.max_position = {-0.5};
        CHECK(solver.calculate(input, trajectory) == Result::ErrorPositionalLimits);
    }
}

TEST_CASE("position-query conditioning avoids intermediate time-power overflow and underflow") {
    for (const auto& coefficients: std::vector<std::array<double, 6>> {
        {1e-300, 0.0, 0.0, -1.0, 1e100, 1e100},
        {1e300, 0.0, 0.0, -1.0, 1e-100, 1e-100},
        {0.0, 1e-300, 0.0, -1.0, 1e150, 1e150},
        {0.0, 0.0, 1e-300, -1.0, 1e300, 1e300},
    }) {
        const auto& c = coefficients;
        std::vector<double> roots;
        for (double root: roots::solve_cubic_scaled(c[0], c[1], c[2], c[3], c[4])) roots.push_back(root);
        REQUIRE(roots.size() == 1);
        CHECK(roots.front() == doctest::Approx(c[5]).epsilon(1e-12).scale(0.0));
    }
}

TEST_CASE("position queries match native sampling across spatial and temporal scales") {
    Ruckig<1> solver;
    for (double space: {1e-4, 1.0, 1e4}) {
        for (double time_scale: {1e-3, 1.0, 100.0}) {
            for (double sign: {-1.0, 1.0}) {
                INFO("space: " << space << ", time scale: " << time_scale << ", sign: " << sign);
                auto input = position_query_input();
                input.target_position = {sign * space};
                input.max_velocity = {2.0 * space / time_scale};
                input.max_acceleration = {2.0 * space / (time_scale * time_scale)};
                input.max_jerk = {4.0 * space / (time_scale * time_scale * time_scale)};
                Trajectory<1> trajectory;
                REQUIRE(solver.calculate(input, trajectory) == Result::Working);
                for (size_t sample = 1; sample < 20; ++sample) {
                    const double time = trajectory.get_duration() * sample / 20.0;
                    double position;
                    trajectory.at_time(time, position);
                    for (double after: {0.0, 0.75 * time}) {
                        const auto crossing = trajectory.get_first_time_at_position(0, position, after);
                        REQUIRE(crossing.has_value());
                        CHECK(*crossing >= after);
                        CHECK(*crossing <= trajectory.get_duration());
                        double actual;
                        trajectory.at_time(*crossing, actual);
                        CHECK(std::abs(actual - position) < 1e-9);
                    }
                }
            }
        }
    }
}
