#include <doctest/doctest.h>

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <httplib/httplib.h>
#include <ruckig/ruckig.hpp>

using namespace ruckig;

namespace {
Profile query_phase(double p, double v, double a, double j, double duration) {
    Profile profile {};
    profile.t[3] = duration;
    profile.j[3] = j;
    const auto end = integrate(duration, p, v, a, j);
    for (size_t step = 0; step < 7; ++step) profile.t_sum[step] = step < 3 ? 0.0 : duration;
    for (size_t step = 0; step < 8; ++step) {
        profile.p[step] = step < 4 ? p : std::get<0>(end);
        profile.v[step] = step < 4 ? v : std::get<1>(end);
        profile.a[step] = step < 4 ? a : std::get<2>(end);
    }
    profile.pf = std::get<0>(end);
    profile.vf = std::get<1>(end);
    profile.af = std::get<2>(end);
    return profile;
}

// Exercise the production post/decoder using structurally accepted responses.
// Phase supports can extend beyond logical sections; only global query bounds
// and root fallback behavior are asserted here.
Trajectory<1> decoded_query_trajectory(const std::vector<Profile>& profiles, const std::vector<double>& ends) {
    std::vector<std::array<Profile, 1>> rows;
    for (const auto& profile: profiles) rows.push_back({profile});
    nlohmann::json response {{"result", "Result.Working"}, {"profiles", rows},
        {"duration", ends.back()}, {"cumulative_times", ends}};
    httplib::test_response = std::make_shared<httplib::Response>(httplib::Response {200, response.dump()});
    Ruckig<1> solver(0.01, 4);
    InputParameter<1> input;
    input.current_position = {0.0};
    input.target_position = {profiles.back().pf};
    input.max_velocity = input.max_acceleration = {2.0};
    input.max_jerk = {4.0};
    input.intermediate_positions = {{0.0}};
    Trajectory<1> trajectory(4);
    const auto posts_before = httplib::test_posts;
    REQUIRE(solver.calculate(input, trajectory) == Result::Working);
    CHECK(httplib::test_posts == posts_before + 1);
    return trajectory;
}
} // namespace

TEST_CASE("cloud query bounds validate clamped roots against the sampled boundary") {
    const double end = std::nextafter(1.0, 0.0);
    const auto terminal = decoded_query_trajectory({query_phase(0.0, 1.0, 0.0, 0.0, 1.0)}, {end});
    CHECK(terminal.get_first_time_at_position(0, 1.0) == std::optional<double>(end));

    const auto truncated = decoded_query_trajectory({query_phase(0.0, 1.0, 0.0, 0.0, 2.0)}, {0.5});
    CHECK_FALSE(truncated.get_first_time_at_position(0, 1.0).has_value());
    CHECK(truncated.get_first_time_at_position(0, 0.25) == std::optional<double>(0.25));
}

TEST_CASE("cloud query bounds retry later roots and endpoint fallbacks after a rejected clamp") {
    const double prefix = std::ldexp(1.0, -53);
    const double after = std::nextafter(1.0, std::numeric_limits<double>::infinity());
    const auto later_root = decoded_query_trajectory({query_phase(0.0, 0.0, 0.0, 0.0, 0.0),
        query_phase(2.0, -3.0, 2.0, 0.0, 3.0), query_phase(9.0, 0.0, 0.0, 0.0, 0.0)}, {prefix, 0.5, 4.0});
    CHECK(later_root.get_first_time_at_position(0, 0.0, after) == std::optional<double>(2.0));

    const auto endpoint = decoded_query_trajectory({query_phase(0.0, 0.0, 0.0, 0.0, 0.0),
        query_phase(0.0, 1e-16, -2e-16, 0.0, 2.0), query_phase(9.0, 0.0, 0.0, 0.0, 0.0)}, {prefix, 0.5, 4.0});
    CHECK(endpoint.get_first_time_at_position(0, 0.0, after) == std::optional<double>(2.0));

    const double duration = 1000.0;
    const auto tiny = query_phase(0.0, 0.0, 0.0, 1e-17, duration);
    const double end = std::nextafter(duration, 0.0);
    const auto terminal = decoded_query_trajectory({tiny}, {end});
    CHECK(terminal.get_first_time_at_position(0, tiny.pf) == std::optional<double>(end));
}

TEST_CASE("cloud query bounds validate a finite boundary after phase-time overflow") {
    auto huge = query_phase(0.0, 0.0, 0.0, 0.0, 1.0);
    huge.t.fill(1e308);
    huge.t_sum.fill(1.0);
    huge.p.fill(0.0);
    huge.p.back() = huge.pf = 1.0;
    const auto trajectory = decoded_query_trajectory({huge, query_phase(1.0, 0.0, 0.0, 0.0, 0.0)}, {1.0, 2.0});
    const auto crossing = trajectory.get_first_time_at_position(0, 1.0, 0.5);
    REQUIRE(crossing.has_value());
    CHECK(std::isfinite(*crossing));
    CHECK(*crossing == 2.0);
}

TEST_CASE("cloud query windows include zero-duration responses") {
    const auto zero = query_phase(7.0, 0.0, 0.0, 0.0, 0.0);
    const auto trajectory = decoded_query_trajectory({zero, zero}, {0.0, 0.0});
    CHECK(trajectory.get_first_time_at_position(0, 7.0, -1.0) == std::optional<double>(0.0));
    CHECK(trajectory.get_first_time_at_position(0, 7.0, 0.0) == std::optional<double>(0.0));
    CHECK_FALSE(trajectory.get_first_time_at_position(0, 7.0,
        std::nextafter(0.0, std::numeric_limits<double>::infinity())).has_value());
    CHECK_FALSE(trajectory.get_first_time_at_position(0, 8.0).has_value());
}
