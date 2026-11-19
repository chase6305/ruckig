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
// Phase supports can differ from logical sections. Queries must use the same
// time supports as production sampling, including closed terminal boundaries.
Trajectory<1> decoded_query_trajectory(const std::vector<Profile>& profiles, const std::vector<double>& ends,
    std::array<double, 2> bounds = {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()},
    Result expected = Result::Working) {
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
    input.min_position = {bounds[0]};
    input.max_position = {bounds[1]};
    input.intermediate_positions = {{0.0}};
    Trajectory<1> trajectory(4);
    const auto posts_before = httplib::test_posts;
    REQUIRE(solver.calculate(input, trajectory) == expected);
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

TEST_CASE("cloud query windows discard roots from sections before the lower bound") {
    const double prefix = std::ldexp(1.0, -53);
    const double after = std::nextafter(1.0, std::numeric_limits<double>::infinity());
    const auto later_root = decoded_query_trajectory({query_phase(0.0, 0.0, 0.0, 0.0, 0.0),
        query_phase(2.0, -3.0, 2.0, 0.0, 3.0), query_phase(9.0, 0.0, 0.0, 0.0, 0.0)}, {prefix, 0.5, 4.0});
    CHECK_FALSE(later_root.get_first_time_at_position(0, 0.0, after).has_value());
    double sampled;
    later_root.at_time(2.0, sampled);
    CHECK(sampled == 9.0);

    const auto endpoint = decoded_query_trajectory({query_phase(0.0, 0.0, 0.0, 0.0, 0.0),
        query_phase(0.0, 1e-16, -2e-16, 0.0, 2.0), query_phase(9.0, 0.0, 0.0, 0.0, 0.0)}, {prefix, 0.5, 4.0});
    CHECK_FALSE(endpoint.get_first_time_at_position(0, 0.0, after).has_value());

    const double duration = 1000.0;
    const auto tiny = query_phase(0.0, 0.0, 0.0, 1e-17, duration);
    const double end = std::nextafter(duration, 0.0);
    const auto terminal = decoded_query_trajectory({tiny}, {end});
    const auto crossing = terminal.get_first_time_at_position(0, tiny.pf);
    REQUIRE(crossing.has_value());
    CHECK(*crossing >= 0.0);
    CHECK(*crossing <= end);
    CHECK(*crossing == doctest::Approx(end));
    terminal.at_time(*crossing, sampled);
    CHECK(std::abs(sampled - tiny.pf) < 1e-9);
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
    CHECK(*crossing == 1.0);
}

TEST_CASE("cloud position queries follow logical sections and sampled phase boundaries") {
    const auto truncated = decoded_query_trajectory({query_phase(0.0, 1.0, 0.0, 0.0, 2.0),
        query_phase(10.0, 0.0, 0.0, 0.0, 0.0)}, {0.5, 2.0});
    CHECK_FALSE(truncated.get_first_time_at_position(0, 1.0).has_value());
    CHECK(truncated.get_first_time_at_position(0, 0.25) == std::optional<double>(0.25));
    CHECK(truncated.get_first_time_at_position(0, 10.0) == std::optional<double>(0.5));
    CHECK(truncated.get_first_time_at_position(0, 10.0, 1.0) == std::optional<double>(1.0));

    auto phase = query_phase(0.0, 1.0, 0.0, 0.0, 1.0);
    phase.t.fill(0.0);
    phase.t_sum.fill(1.0);
    phase.p.fill(9.0);
    phase.p[0] = 0.0;
    phase.p.back() = phase.pf = 1.0;
    phase.v.fill(0.0);
    phase.v[0] = 1.0;
    const auto different_supports = decoded_query_trajectory({phase}, {1.0});
    CHECK(different_supports.get_first_time_at_position(0, 0.5) == std::optional<double>(0.5));
}

TEST_CASE("cloud position queries use the sampled brake duration and terminal state") {
    auto profile = query_phase(2.0, 1.0, 0.0, 0.0, 1.0);
    profile.brake.duration = 2.0;
    profile.brake.t = {0.25, 0.25};
    profile.brake.p = {0.0, 0.25};
    profile.brake.v = {1.0, 1.0};
    const auto braking = decoded_query_trajectory({profile}, {3.0});
    CHECK(braking.get_first_time_at_position(0, 1.0) == std::optional<double>(1.0));
    CHECK(braking.get_first_time_at_position(0, 2.5) == std::optional<double>(2.5));

    const auto terminal = decoded_query_trajectory({query_phase(0.0, 0.0, 0.0, 6.0, 2.0)}, {0.5});
    CHECK(terminal.get_first_time_at_position(0, 3.5) == std::optional<double>(0.5));
    CHECK_FALSE(terminal.get_first_time_at_position(0, 1.0).has_value());
}

TEST_CASE("position queries distinguish close cubic crossings in time order") {
    const double amplitude = std::ldexp(1.0, 30), delta = std::ldexp(1.0, -12);
    // A * (t - 1/2) * ((t - 1/2)^2 - delta^2), with exactly
    // representable coefficients and three nearby, representable crossings.
    const auto phase = query_phase(amplitude * (-0.125 + 0.5 * delta * delta),
        amplitude * (0.75 - delta * delta), -3.0 * amplitude, 6.0 * amplitude, 1.0);
    const auto trajectory = decoded_query_trajectory({phase}, {1.0});
    for (const auto& window: std::array<std::array<double, 2>, 3> {{{0.49, 0.5 - delta}, {0.4998, 0.5}, {0.5001, 0.5 + delta}}}) {
        const auto crossing = trajectory.get_first_time_at_position(0, 0.0, window[0]);
        REQUIRE(crossing.has_value());
        CHECK(*crossing == doctest::Approx(window[1]).epsilon(1e-9));
        double position;
        trajectory.at_time(*crossing, position);
        CHECK(std::abs(position) < 1e-9);
    }
    CHECK_FALSE(trajectory.get_first_time_at_position(0, 0.0, 0.5003).has_value());
}

TEST_CASE("position queries retain small crossings inside enormous cloud phases") {
    const auto trajectory = decoded_query_trajectory({query_phase(0.0, 0.0, 1.0, 0.0, 1e100)}, {1e100});
    CHECK(trajectory.get_first_time_at_position(0, 0.5) == std::optional<double>(1.0));
    CHECK(trajectory.get_first_time_at_position(0, 2.0) == std::optional<double>(2.0));
}

TEST_CASE("position queries refine crossings across very wide time ranges") {
    for (const auto& sample: std::array<std::array<double, 3>, 2> {{{6e-300, 1e200, 1e100}, {6e300, 1.0, 1e-100}}}) {
        const auto trajectory = decoded_query_trajectory({query_phase(0.0, 0.0, 0.0, sample[0], sample[1])}, {sample[1]});
        double position;
        trajectory.at_time(sample[2], position);
        CHECK(position == doctest::Approx(1.0));
        for (double after: {0.0, 0.5 * sample[2]}) {
            const auto crossing = trajectory.get_first_time_at_position(0, position, after);
            REQUIRE(crossing.has_value());
            CHECK(*crossing >= after);
            CHECK(*crossing <= trajectory.get_duration());
            CHECK(*crossing == doctest::Approx(sample[2]).epsilon(1e-8).scale(0.0));
            double actual;
            trajectory.at_time(*crossing, actual);
            CHECK(std::abs(actual - position) < 1e-9);
        }
    }
}

TEST_CASE("cloud extrema and position limits follow sampled phase and braking supports") {
    const double infinity = std::numeric_limits<double>::infinity();
    auto extended = query_phase(0.0, 1.0, 0.0, 0.0, 0.5);
    for (size_t step = 3; step < 7; ++step) extended.t_sum[step] = 2.0;
    const auto extended_trajectory = decoded_query_trajectory({extended}, {2.0});
    std::array<Bound, 1> extrema;
    extended_trajectory.get_position_extrema(extrema);
    CHECK(extrema[0].min == 0.0);
    CHECK(extrema[0].max == std::nextafter(2.0, 0.0));
    CHECK(extrema[0].t_max == std::nextafter(2.0, 0.0));
    decoded_query_trajectory({extended}, {2.0}, {-infinity, 1.0}, Result::ErrorPositionalLimits);

    const auto truncated = query_phase(0.0, 1.0, 0.0, 0.0, 2.0);
    const auto truncated_trajectory = decoded_query_trajectory({truncated}, {0.5}, {-infinity, 1.0});
    truncated_trajectory.get_position_extrema(extrema);
    CHECK(extrema[0].min == 0.0);
    CHECK(extrema[0].max == 0.5);
    CHECK(extrema[0].t_max == 0.5);

    auto braking = query_phase(0.5, 0.0, 0.0, 0.0, 0.5);
    braking.brake.duration = 2.0;
    braking.brake.t = {0.25, 0.25};
    braking.brake.p = {0.0, 0.25};
    braking.brake.v = {1.0, 1.0};
    braking.brake.a = {0.0, -1.0};
    const auto braking_trajectory = decoded_query_trajectory({braking}, {2.5});
    braking_trajectory.get_position_extrema(extrema);
    CHECK(extrema[0].max == 0.75);
    CHECK(extrema[0].t_max == doctest::Approx(1.25));
    decoded_query_trajectory({braking}, {2.5}, {-infinity, 0.6}, Result::ErrorPositionalLimits);
}

TEST_CASE("cloud extrema locate rounded phase transitions by the sampler comparison") {
    const double prefix = 0.0018219466777782895, begin = 0.0012339725813650534, end = 0.0019848046019890486;
    auto profile = query_phase(2550.5042361533315, 2731174.6480475105, -169424696.66111404, -2523730255021.1367, end - begin);
    for (size_t step = 0; step < 7; ++step) profile.t_sum[step] = step < 3 ? begin : end;
    profile.p[0] = profile.v[0] = profile.a[0] = profile.j[0] = 0.0;
    profile.p.back() = profile.v.back() = profile.a.back() = profile.pf = profile.vf = profile.af = 0.0;
    const double duration = prefix + end;
    const auto trajectory = decoded_query_trajectory({query_phase(0.0, 0.0, 0.0, 0.0, prefix), profile}, {prefix, duration});
    const double adjacent = std::nextafter(duration, 0.0), before = std::nextafter(adjacent, 0.0);
    double adjacent_position, before_position;
    trajectory.at_time(adjacent, adjacent_position);
    trajectory.at_time(before, before_position);
    REQUIRE(adjacent_position == 0.0);
    REQUIRE(before_position > 4000.0);
    std::array<Bound, 1> extrema;
    trajectory.get_position_extrema(extrema);
    CHECK(extrema[0].max == before_position);
    CHECK(extrema[0].t_max == before);
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

TEST_CASE("a new target calculation discards cloud post-trajectory metadata") {
    auto profile = query_phase(0.0, 1.0, 0.0, 0.0, 1.0);
    profile.accel.duration = 0.5;
    profile.accel.t = {0.5, 0.0};
    profile.accel.p = {1.0, 1.5};
    profile.accel.v = {1.0, 1.0};
    auto trajectory = decoded_query_trajectory({profile}, {1.5});
    REQUIRE(trajectory.get_profiles()[0][0].accel.duration == 0.5);

    Ruckig<1> solver;
    InputParameter<1> input;
    input.current_position = {0.0};
    input.target_position = {3.0};
    input.max_velocity = input.max_acceleration = {2.0};
    input.max_jerk = {4.0};
    Trajectory<1> expected;
    REQUIRE(solver.calculate(input, expected) == Result::Working);
    REQUIRE(solver.calculate(input, trajectory) == Result::Working);
    CHECK(trajectory.get_duration() == expected.get_duration());
    CHECK(trajectory.get_profiles()[0][0].accel.duration == 0.0);
    CHECK(trajectory.get_independent_min_durations() == expected.get_independent_min_durations());
}
