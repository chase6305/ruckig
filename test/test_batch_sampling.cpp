#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <random>
#include <span>
#include <utility>
#include <vector>

#include <ruckig/ruckig.hpp>

using namespace ruckig;

namespace {
// No iterators or contiguous storage: batch sampling needs only size and index.
template<class T, size_t N>
class SamplingVector {
    std::deque<T> values = std::deque<T>(N);
public:
    void resize(size_t n) { values.resize(n); }
    size_t size() const { return values.size(); }
    T& operator[](size_t i) { return values[i]; }
    const T& operator[](size_t i) const { return values[i]; }
    bool operator==(const SamplingVector& other) const { return values == other.values; }
};

template<class Traj, class Vector>
void compare_sampling(const Traj& trajectory, const Vector& row) {
    std::vector<double> times {-1.0, -0.0, 0.0};
    const auto add_boundary = [&](double time) {
        times.push_back(std::nextafter(time, -std::numeric_limits<double>::infinity()));
        times.push_back(time);
        times.push_back(time);
        times.push_back(std::nextafter(time, std::numeric_limits<double>::infinity()));
    };
    double start = 0.0;
    const auto ends = trajectory.get_intermediate_durations();
    const auto profiles = trajectory.get_profiles();
    for (size_t section = 0; section < ends.size(); ++section) {
        add_boundary(ends[section]);
        for (size_t dof = 0; dof < trajectory.degrees_of_freedom; ++dof) {
            const auto& p = profiles[section][dof];
            if (p.brake.duration > 0.0) add_boundary(start + p.brake.t[0]);
            add_boundary(start + p.brake.duration);
            for (double t: p.t_sum) add_boundary(start + p.brake.duration + t);
        }
        start = ends[section];
    }
    for (size_t i = 0; i <= 100; ++i) times.push_back(trajectory.get_duration() * i / 100.0);
    times.push_back(trajectory.get_duration() + 1.0);
    std::sort(times.begin(), times.end());
    std::mt19937 generator(42);
    for (size_t order = 0; order < 3; ++order) {
        if (order == 1) std::reverse(times.begin(), times.end());
        if (order == 2) std::shuffle(times.begin(), times.end(), generator);
        INFO("time order: " << order);
        std::vector<Vector> positions(times.size(), row), velocities = positions, accelerations = positions;
        trajectory.at_times(times, positions, velocities, accelerations);
        for (size_t i = 0; i < times.size(); ++i) {
            auto p = row, v = row, a = row;
            trajectory.at_time(times[i], p, v, a);
            CHECK(positions[i] == p);
            CHECK(velocities[i] == v);
            CHECK(accelerations[i] == a);
        }
    }
}

template<size_t N, template<class, size_t> class V = StandardVector>
void sampling_cases() {
    auto input = [] {
        if constexpr (N == 0) return InputParameter<N, V>(2);
        else return InputParameter<N, V>();
    }();
    auto trajectory = [] {
        if constexpr (N == 0) return Trajectory<N, V>(2);
        else return Trajectory<N, V>();
    }();
    auto solver = [] {
        if constexpr (N == 0) return Ruckig<N, V>(2, 0.01);
        else return Ruckig<N, V>(0.01);
    }();
    for (size_t d = 0; d < 2; ++d) {
        input.current_position[d] = 0.0;
        input.target_position[d] = d == 0 ? 3.0 : -1.5;
        input.max_velocity[d] = input.max_acceleration[d] = 2.0;
        input.max_jerk[d] = 4.0;
    }
    for (size_t shape = 0; shape < 6; ++shape) {
        INFO("shape: " << shape);
        auto trial = input;
        if (shape == 1) trial.current_velocity[0] = 3.0; // Braking before the main profile.
        if (shape == 2) trial.synchronization = Synchronization::None;
        if (shape == 3) trial.enabled[1] = false;
        if (shape >= 2 && shape <= 3) {
            trial.current_velocity[1] = -0.25;
            trial.current_acceleration[1] = 0.1;
        }
        if (shape == 4) {
            for (double fraction: {0.0, 0.25, 0.25, 0.5, 1.0}) {
                auto point = input.target_position;
                for (size_t d = 0; d < 2; ++d) point[d] *= fraction;
                trial.intermediate_positions.push_back(point);
            }
        }
        if (shape == 5) trial.target_position = trial.current_position;
        REQUIRE(solver.calculate_waypoints(trial, trajectory) == Result::Working);
        compare_sampling(trajectory, input.current_position);
        // A second call, copy, and move start with an independent section hint.
        auto copied = trajectory;
        auto moved = std::move(copied);
        const std::vector<double> times {trajectory.get_duration(), 0.0};
        std::vector<V<double, N>> p(2, input.current_position), v = p, a = p;
        moved.at_times(times, p, v, a);
        CHECK(p.back() == input.current_position);
    }
}
} // namespace

TEST_CASE("batch sampling matches scalar sampling at phase and section boundaries in any order") {
    sampling_cases<2>();
    sampling_cases<0>();
    sampling_cases<2, SamplingVector>();
    sampling_cases<0, SamplingVector>();
}

TEST_CASE("batch sampling validates buffers and preserves empty-batch behavior") {
    Trajectory<> trajectory(2);
    std::vector<double> times;
    std::vector<std::vector<double>> p, v, a;
    CHECK_NOTHROW(trajectory.at_times(times, p, v, a));
    times = {0.0};
    p = v = a = {{0.0, 0.0}};
    CHECK_THROWS_AS(trajectory.at_times(times, p, v, a), RuckigError);
    InputParameter<> input(2);
    input.current_position = {0.0, 0.0};
    input.target_position = {1.0, 2.0};
    input.max_velocity = input.max_acceleration = input.max_jerk = {1.0, 1.0};
    REQUIRE(Ruckig<>(2).calculate(input, trajectory) == Result::Working);
    CHECK_NOTHROW(trajectory.at_times(times, p, v, a));
    p[0].clear();
    CHECK_THROWS_AS(trajectory.at_times(times, p, v, a), RuckigError);
    p = v;
    v[0].clear();
    CHECK_THROWS_AS(trajectory.at_times(times, p, v, a), RuckigError);
    v = p;
    a[0].clear();
    CHECK_THROWS_AS(trajectory.at_times(times, p, v, a), RuckigError);
    a = p;
    for (double invalid: {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
        times[0] = invalid;
        CHECK_THROWS_AS(trajectory.at_times(times, p, v, a), RuckigError);
    }
    // Contiguous views into caller-owned rows keep working with the fast path.
    double pb[2], vb[2], ab[2];
    std::array<std::span<double>, 1> ps {pb}, vs {vb}, as {ab};
    times[0] = 0.0;
    trajectory.at_times(times, ps, vs, as);
    CHECK(pb[0] == 0.0);
    CHECK(pb[1] == 0.0);
}

TEST_CASE("disabled axes extrapolate their current state before and after the trajectory") {
    const auto check = []<size_t N>() {
        auto input = [] {
            if constexpr (N == 0) return InputParameter<N>(2);
            else return InputParameter<N>();
        }();
        auto solver = [] {
            if constexpr (N == 0) return Ruckig<N>(2, 0.01);
            else return Ruckig<N>(0.01);
        }();
        auto trajectory = [] {
            if constexpr (N == 0) return Trajectory<N>(2);
            else return Trajectory<N>();
        }();
        input.current_position = {0.0, 10.0};
        input.current_velocity = {0.0, 2.0};
        input.current_acceleration = {0.0, 0.5};
        input.target_position = {1.0, -100.0};
        input.max_velocity = input.max_acceleration = input.max_jerk = {1.0, 1.0};
        const std::vector<double> times {-2.0, -0.5, 0.0, 0.5, 4.0};
        const std::array<double, 5> expected_p {7.0, 9.0625, 10.0, 11.0625, 22.0};
        const std::array<double, 5> expected_v {1.0, 1.75, 2.0, 2.25, 4.0};
        for (bool all_disabled: {false, true}) {
            input.enabled = {!all_disabled, false};
            for (double offset: {0.0, 10.0}) {
                input.current_position[1] = 10.0 + offset;
                REQUIRE(solver.calculate(input, trajectory) == Result::Working);
                std::vector<decltype(input.current_position)> ps(times.size(), input.current_position), vs = ps, as = ps;
                trajectory.at_times(times, ps, vs, as);
                for (size_t i = 0; i < times.size(); ++i) {
                    auto p = input.current_position, v = p, a = p, j = p;
                    size_t section;
                    trajectory.at_time(times[i], p, v, a, j, section);
                    CHECK(p[1] == expected_p[i] + offset);
                    CHECK(v[1] == expected_v[i]);
                    CHECK(a[1] == 0.5);
                    CHECK(j[1] == 0.0);
                    CHECK(ps[i][1] == expected_p[i] + offset);
                    CHECK(vs[i][1] == expected_v[i]);
                    CHECK(as[i][1] == 0.5);
                }
            }
        }
    };
    check.template operator()<2>();
    check.template operator()<0>();
}
