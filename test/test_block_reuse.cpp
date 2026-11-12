#include <doctest/doctest.h>

#include <cmath>
#include <limits>

#include <ruckig/ruckig.hpp>

using namespace ruckig;

namespace {
Block stale_block() {
    Block block {};
    block.a = Block::Interval(0.1, 100.0);
    block.b = Block::Interval(0.2, 200.0);
    block.a->profile = block.b->profile = Profile{};
    return block;
}

void check_unblocked(const Block& block, double duration) {
    CHECK(block.t_min == duration);
    CHECK_FALSE(block.a.has_value());
    CHECK_FALSE(block.b.has_value());
    CHECK_FALSE(block.is_blocked(duration));
    CHECK_FALSE(block.is_blocked(duration + 1.0));
}

void check_fixed_duration(const Block& block, double duration) {
    CHECK(block.t_min == duration);
    REQUIRE(block.a.has_value());
    CHECK(block.a->left == duration);
    CHECK(block.a->right == std::numeric_limits<double>::infinity());
    CHECK_FALSE(block.b.has_value());
    CHECK_FALSE(block.is_blocked(duration));
    CHECK(block.is_blocked(std::nextafter(duration, std::numeric_limits<double>::infinity())));
}

template<size_t N, bool Throw = false>
auto reuse_solver() {
    if constexpr (N == 0) return Ruckig<N, StandardVector, Throw>(2, 0.01);
    else return Ruckig<N, StandardVector, Throw>(0.01);
}

template<size_t N, bool Throw>
void check_reused_solvers() {
    for (size_t mode = 0; mode < 5; ++mode) {
        INFO("solver transition: " << mode);
        auto input = [] {
            if constexpr (N == 0) return InputParameter<N>(2);
            else return InputParameter<N>();
        }();
        auto actual = [] {
            if constexpr (N == 0) return Trajectory<N>(2);
            else return Trajectory<N>();
        }();
        auto expected = actual;
        auto reused = reuse_solver<N, Throw>();
        auto fresh = reuse_solver<N, Throw>();
        input.current_position = {0.0, 0.0};
        input.target_position = {1.0, 0.5};
        input.current_velocity = input.target_velocity = {0.5, 0.0};
        input.max_velocity = input.max_acceleration = {2.0, 2.0};
        input.max_jerk = {0.0, 4.0};
        if (mode >= 3) {
            input.control_interface = ControlInterface::Velocity;
            input.current_velocity = {0.0, 0.0};
            input.target_velocity = {1.0, 0.25};
            input.current_acceleration = input.target_acceleration = {0.5, 0.0};
        }
        REQUIRE(reused.calculate(input, actual) == Result::Working);
        CHECK(actual.get_duration() == 2.0);

        input.current_velocity = input.current_acceleration = input.target_acceleration = {0.0, 0.0};
        if (mode >= 3) input.target_velocity = {0.0, 5.0};
        else {
            input.target_position = {0.0, 3.0};
            input.target_velocity = {0.0, 0.0};
        }
        if (mode == 1) {
            input.max_jerk[0] = std::numeric_limits<double>::infinity();
            input.max_velocity[0] = 0.0;
        } else if (mode == 2) {
            input.max_jerk[0] = input.max_acceleration[0] = std::numeric_limits<double>::infinity();
            input.target_position[0] = 1.0;
        } else if (mode == 4) {
            input.max_jerk[0] = std::numeric_limits<double>::infinity();
        }
        REQUIRE(fresh.calculate(input, expected) == Result::Working);
        REQUIRE(reused.calculate(input, actual) == Result::Working);
        CHECK(actual.get_duration() == expected.get_duration());
        CHECK(actual.get_independent_min_durations() == expected.get_independent_min_durations());
        const auto a = actual.get_profiles(), b = expected.get_profiles();
        for (size_t d = 0; d < 2; ++d) {
            CHECK(a[0][d].t == b[0][d].t);
            CHECK(a[0][d].t_sum == b[0][d].t_sum);
            CHECK(a[0][d].p == b[0][d].p);
            CHECK(a[0][d].v == b[0][d].v);
            CHECK(a[0][d].a == b[0][d].a);
            CHECK(a[0][d].j == b[0][d].j);
        }
    }
}
} // namespace

TEST_CASE("single-profile solvers replace old blocked intervals") {
    Profile input {};
    input.set_boundary(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    auto block = stale_block();
    SUBCASE("third-order position at rest") {
        PositionThirdOrderStep1 step(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, -2.0, 2.0, -2.0, 0.0);
        REQUIRE(step.get_profile(input, block));
        check_unblocked(block, 0.0);
    }
    SUBCASE("third-order position with a newly fixed duration") {
        input.set_boundary(0.0, 0.5, 0.0, 1.0, 0.5, 0.0);
        PositionThirdOrderStep1 step(0.0, 0.5, 0.0, 1.0, 0.5, 0.0, 2.0, -2.0, 2.0, -2.0, 0.0);
        REQUIRE(step.get_profile(input, block));
        check_fixed_duration(block, 2.0);
    }
    SUBCASE("second-order position with zero velocity limits") {
        PositionSecondOrderStep1 step(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, -2.0);
        REQUIRE(step.get_profile(input, block));
        check_unblocked(block, 0.0);
    }
    SUBCASE("first-order position") {
        input.pf = 1.0;
        PositionFirstOrderStep1 step(0.0, 1.0, 2.0, -2.0);
        REQUIRE(step.get_profile(input, block));
        check_unblocked(block, 0.5);
    }
    SUBCASE("third-order velocity with zero acceleration") {
        VelocityThirdOrderStep1 step(0.0, 0.0, 0.0, 0.0, 2.0, -2.0, 0.0);
        REQUIRE(step.get_profile(input, block));
        check_unblocked(block, 0.0);
    }
    SUBCASE("third-order velocity with a newly fixed duration") {
        input.set_boundary(0.0, 0.0, 0.5, 0.0, 1.0, 0.5);
        VelocityThirdOrderStep1 step(0.0, 0.5, 1.0, 0.5, 2.0, -2.0, 0.0);
        REQUIRE(step.get_profile(input, block));
        check_fixed_duration(block, 2.0);
    }
    SUBCASE("second-order velocity") {
        input.vf = 1.0;
        VelocitySecondOrderStep1 step(0.0, 1.0, 2.0, -2.0);
        REQUIRE(step.get_profile(input, block));
        check_unblocked(block, 0.5);
    }
}

TEST_CASE("reusing a solver does not retain another motion's blocked intervals") {
    check_reused_solvers<2, false>();
    check_reused_solvers<0, true>();
}

TEST_CASE("a moving zero-jerk axis still cannot extend its fixed duration") {
    InputParameter<2> input;
    input.current_position = {0.0, 0.0};
    input.target_position = {1.0, 3.0};
    input.current_velocity = input.target_velocity = {0.5, 0.0};
    input.max_velocity = input.max_acceleration = {2.0, 2.0};
    input.max_jerk = {0.0, 4.0};
    Ruckig<2> solver;
    Trajectory<2> trajectory;
    CHECK(solver.calculate(input, trajectory) == Result::ErrorZeroLimits);
}
