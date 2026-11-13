#include <doctest/doctest.h>

#include <array>
#include <vector>

#include <ruckig/ruckig.hpp>

using namespace ruckig;

namespace {
template<class T, size_t N>
class IndexedVector {
    std::vector<T> values = std::vector<T>(N);
public:
    void resize(size_t size) { values.resize(size); }
    size_t size() const { return values.size(); }
    T& operator[](size_t i) { return values[i]; }
    const T& operator[](size_t i) const { return values[i]; }
    bool operator==(const IndexedVector& other) const { return values == other.values; }
};

// vector<bool> is not indexable through bool&, so keep its storage independent.
template<size_t N>
class IndexedVector<bool, N> {
    std::vector<unsigned char> values = std::vector<unsigned char>(N);
public:
    void resize(size_t size) { values.resize(size); }
    size_t size() const { return values.size(); }
    unsigned char& operator[](size_t i) { return values[i]; }
    bool operator[](size_t i) const { return values[i] != 0; }
    bool operator==(const IndexedVector& other) const { return values == other.values; }
};

InputParameter<2> refinement_input(size_t shape) {
    InputParameter<2> input;
    input.current_position = {0.0, 0.0};
    input.target_position = {3.0, 1.5};
    input.max_velocity = input.max_acceleration = {2.0, 2.0};
    input.max_jerk = {4.0, 4.0};
    input.min_position = {-10.0, -10.0};
    input.max_position = {10.0, 10.0};
    input.intermediate_positions = {{0.4, 0.2}, {0.8, 0.6}, {1.4, 0.4}, {2.0, 0.8}};
    if (shape == 1) {
        input.intermediate_positions = {{0.5, 1.0}, {1.0, 0.0}, {1.5, 1.0}};
        input.target_position = {2.0, 0.0};
    } else if (shape == 2) {
        input.current_velocity = {2.2, -0.15};
        input.target_velocity = {0.25, 0.1};
        input.target_acceleration = {0.05, -0.03};
    } else if (shape == 3) {
        input.min_velocity = std::array<double, 2>{-0.75, -1.25};
        input.min_acceleration = std::array<double, 2>{-1.0, -0.6};
    } else if (shape == 4) {
        input.intermediate_positions = {{1.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}};
        input.target_position = {3.0, 0.0};
    } else if (shape == 5) {
        input.per_section_minimum_duration = std::vector<double>{0.5, 0.3, 1.2, 0.1, 0.7};
    }
    return input;
}

template<size_t N, template<class, size_t> class V = StandardVector>
void check_embedded_refinement(size_t dofs) {
    auto solver = [dofs] {
        if constexpr (N == 0) return Ruckig<N, V>(dofs, 0.01);
        else return Ruckig<N, V>(0.01);
    }();
    auto trajectory = [dofs] {
        if constexpr (N == 0) return Trajectory<N, V>(dofs);
        else return Trajectory<N, V>();
    }();
    for (size_t shape = 0; shape < 6; ++shape) {
        for (auto synchronization: {Synchronization::Time, Synchronization::Phase}) {
            auto source = refinement_input(shape);
            source.synchronization = synchronization;
            auto input = [dofs] {
                if constexpr (N == 0) return InputParameter<N, V>(dofs);
                else return InputParameter<N, V>();
            }();
            for (size_t d = 0; d < dofs; ++d) {
                input.current_position[d] = input.target_position[d] = 0.125;
                input.max_velocity[d] = input.max_acceleration[d] = 2.0;
                input.max_jerk[d] = 4.0;
                input.min_position[d] = -10.0;
                input.max_position[d] = 10.0;
            }
            const auto copy_active = [dofs](auto& destination, const auto& values) {
                destination[0] = values[0];
                destination[dofs - 1] = values[1];
            };
            copy_active(input.current_position, source.current_position);
            copy_active(input.current_velocity, source.current_velocity);
            copy_active(input.current_acceleration, source.current_acceleration);
            copy_active(input.target_position, source.target_position);
            copy_active(input.target_velocity, source.target_velocity);
            copy_active(input.target_acceleration, source.target_acceleration);
            input.synchronization = synchronization;
            input.per_section_minimum_duration = source.per_section_minimum_duration;
            if (source.min_velocity) {
                input.min_velocity = input.current_position;
                input.min_acceleration = input.current_position;
                for (size_t d = 0; d < dofs; ++d) {
                    (*input.min_velocity)[d] = (*input.min_acceleration)[d] = -2.0;
                }
                copy_active(*input.min_velocity, *source.min_velocity);
                copy_active(*input.min_acceleration, *source.min_acceleration);
            }
            for (const auto& point: source.intermediate_positions) {
                auto embedded = input.current_position;
                copy_active(embedded, point);
                input.intermediate_positions.push_back(embedded);
            }
            const auto original = input;
            for (size_t iterations: {size_t(1), size_t(8)}) {
                INFO("shape=" << shape << ", DoFs=" << dofs << ", iterations=" << iterations);
                Ruckig<2> oracle(0.01);
                Trajectory<2> expected;
                REQUIRE(oracle.optimize_waypoints(source, expected, iterations) == Result::Working);
                REQUIRE(solver.optimize_waypoints(input, trajectory, iterations) == Result::Working);
                CHECK_FALSE(input != original);
                CHECK(trajectory.get_duration() == expected.get_duration());
                CHECK(trajectory.get_intermediate_durations() == expected.get_intermediate_durations());
                const auto profiles = trajectory.get_profiles();
                const auto expected_profiles = expected.get_profiles();
                REQUIRE(profiles.size() == expected_profiles.size());
                for (size_t i = 0; i < profiles.size(); ++i) {
                    for (size_t axis = 0; axis < 2; ++axis) {
                        const auto& actual = profiles[i][axis == 0 ? 0 : dofs - 1];
                        const auto& reference = expected_profiles[i][axis];
                        CHECK(actual.t == reference.t);
                        CHECK(actual.t_sum == reference.t_sum);
                        CHECK(actual.p == reference.p);
                        CHECK(actual.v == reference.v);
                        CHECK(actual.a == reference.a);
                        CHECK(actual.j == reference.j);
                        CHECK(actual.brake.duration == reference.brake.duration);
                    }
                }
                auto p = input.current_position, v = p, a = p;
                std::array<double, 2> ep, ev, ea;
                for (size_t sample = 0; sample <= 16; ++sample) {
                    const double time = expected.get_duration() * sample / 16.0;
                    trajectory.at_time(time, p, v, a);
                    expected.at_time(time, ep, ev, ea);
                    CHECK(p[0] == ep[0]); CHECK(p[dofs - 1] == ep[1]);
                    CHECK(v[0] == ev[0]); CHECK(v[dofs - 1] == ev[1]);
                    CHECK(a[0] == ea[0]); CHECK(a[dofs - 1] == ea[1]);
                    for (size_t d = 1; d + 1 < dofs; ++d) {
                        CHECK(p[d] == 0.125);
                        CHECK(v[d] == 0.0);
                        CHECK(a[d] == 0.0);
                    }
                }
            }
        }
    }
}
} // namespace

TEST_CASE("waypoint refinement preserves embedded motion with stationary axes") {
    check_embedded_refinement<6>(6);
    check_embedded_refinement<0>(32);
    check_embedded_refinement<0, IndexedVector>(6);
}
