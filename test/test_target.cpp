#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

#include <deque>
#include <cstring>
#include <new>
#include <random>
#include <optional>
#include "randomizer.hpp"

#include <ruckig/error.hpp>
#include <ruckig/ruckig.hpp>


using namespace ruckig;


int seed {42};
size_t number_trajectories {256 * 1024}; // Some user variable you want to be able to set
size_t position_random_3, random_3_high, random_discrete_3, random_direction_3, position_second_random_3;
size_t velocity_random_3, velocity_random_discrete_3, velocity_second_random_3;

std::normal_distribution<double> position_dist {0.0, 4.0};
std::normal_distribution<double> dynamic_dist {0.0, 0.8};
std::uniform_real_distribution<double> limit_dist {0.08, 16.0};
std::uniform_real_distribution<double> limit_dist_high {10.0, 1000000.0};
std::uniform_real_distribution<double> min_limit_dist {-16.0, -0.08};


template<class T, size_t DOFs>
class MinimalRuntimeVector {
    std::deque<T> data;

public:
    MinimalRuntimeVector() {}
    MinimalRuntimeVector(std::initializer_list<T> a) {
        data.resize(a.size());
        std::copy_n(a.begin(), a.size(), std::begin(data));
    }

    T operator[](size_t i) const {
        return data[i];
    }

    T& operator[](size_t i) {
        return data[i];
    }

    size_t size() const {
        return data.size();
    }

    void resize(size_t size) {
        data.resize(size);
    }

    bool operator==(const MinimalRuntimeVector<T, DOFs>& rhs) const {
        for (size_t dof = 0; dof < data.size(); ++dof) {
            if (data[dof] != rhs[dof]) {
                return false;
            }
        }
        return true;
    }
};


template<size_t DOFs, class OTGType>
void check_duration(OTGType& ruckig, InputParameter<DOFs>& input, double duration) {
    OutputParameter<DOFs> output;

    while (ruckig.update(input, output) == Result::Working) {
        input.current_position = output.new_position;
        input.current_velocity = output.new_velocity;
        input.current_acceleration = output.new_acceleration;
    }

    CHECK( output.trajectory.get_duration() == doctest::Approx(duration) );
}


template<size_t DOFs, class OTGType>
void check_calculation(OTGType& ruckig, InputParameter<DOFs>& input) {
    OutputParameter<DOFs> output;

    CAPTURE( input );

    auto result = ruckig.update(input, output);
    if (result == Result::ErrorTrajectoryDuration) {
        return;
    }

    CHECK( (result == Result::Working || (result == Result::Finished && output.trajectory.get_duration() < 0.005)) );
    CHECK( output.trajectory.get_duration() >= 0.0 );

    for (size_t dof = 0; dof < ruckig.degrees_of_freedom; ++dof) {
        CHECK_FALSE( (std::isnan(output.new_position[dof]) || std::isnan(output.new_velocity[dof]) || std::isnan(output.new_acceleration[dof])) );
    }
}


template<size_t DOFs, class OTGType>
size_t step_through_and_check_calculation(OTGType& ruckig, InputParameter<DOFs>& input, size_t max_number_checks) {
    OutputParameter<DOFs> output;
    OTGType ruckig_second {0.001};

    check_calculation<DOFs, OTGType>(ruckig, input);

    size_t number_checks {1};
    while (ruckig.update(input, output) == Result::Working) {
        input.current_position = output.new_position;
        input.current_velocity = output.new_velocity;
        input.current_acceleration = output.new_acceleration;

        // Or randomize input with small noise here?
        check_calculation<DOFs, OTGType>(ruckig_second, input);

        number_checks += 1;
        if (number_checks == max_number_checks) {
            break;
        }
    }

    return number_checks;
}


template<size_t DOFs, class OTGType, class OTGCompType>
void check_comparison(OTGType& ruckig, InputParameter<DOFs>& input, OTGCompType& ruckig_comparison) {
    OutputParameter<DOFs> output;

    CAPTURE( input );

    auto result = ruckig.update(input, output);
    if (result == Result::ErrorTrajectoryDuration) {
        return;
    }

    CHECK( (result == Result::Working || (result == Result::Finished && output.trajectory.get_duration() < 0.005)) );

    OutputParameter<DOFs> output_comparison;
    auto result_comparison = ruckig_comparison.update(input, output_comparison);
    CHECK( output.trajectory.get_duration() <= doctest::Approx(output_comparison.trajectory.get_duration()) );
}


template<class T>
bool array_eq(const T& first, const T& second) {
    for (size_t dof = 0; dof < first.size(); ++dof) {
        if (first[dof] != doctest::Approx(second[dof])) {
            return false;
        }
    }
    return true;
}


TEST_CASE("trajectory") {
    RuckigThrow<DynamicDOFs> ruckig {3, 0.005};
    InputParameter input {3};
    OutputParameter output {3};

    input.current_position = {0.0, -2.0, 0.0};
    input.current_velocity = {0.0, 0.0, 0.0};
    input.current_acceleration = {0.0, 0.0, 0.0};
    input.target_position = {1.0, -3.0, 2.0};
    input.target_velocity = {0.0, 0.3, 0.0};
    input.target_acceleration = {0.0, 0.0, 0.0};
    input.max_velocity = {1.0, 1.0, 1.0};
    input.max_acceleration = {1.0, 1.0, 1.0};
    input.max_jerk = {1.0, 1.0, 1.0};

    Trajectory traj {3};
    auto result = ruckig.calculate(input, traj);

    CHECK( result == Result::Working );
    CHECK( traj.get_duration() == doctest::Approx(4.0) );

    result = ruckig.update(input, output);

    CHECK( result == Result::Working );
    CHECK( output.trajectory.get_duration() == doctest::Approx(4.0) );

    SUBCASE("at-time") {
        std::vector<double> new_position(3), new_velocity(3), new_acceleration(3), new_jerk(3);
        output.trajectory.at_time(0.0, new_position, new_velocity, new_acceleration);
        CHECK( array_eq(new_position, input.current_position) );
        CHECK( array_eq(new_velocity, input.current_velocity) );
        CHECK( array_eq(new_acceleration, input.current_acceleration) );

        output.trajectory.at_time(output.trajectory.get_duration(), new_position, new_velocity, new_acceleration);
        CHECK( array_eq(new_position, input.target_position) );
        CHECK( array_eq(new_velocity, input.target_velocity) );
        CHECK( array_eq(new_acceleration, input.target_acceleration) );

        size_t new_section;
        output.trajectory.at_time(2.0, new_position, new_velocity, new_acceleration, new_jerk, new_section);
        CHECK( array_eq(new_position, {0.5, -2.6871268303, 1.0}) );
        CHECK( array_eq(new_jerk, {0.0, 0.0, -1.0}) );
        CHECK( new_section == 0 );

        output.trajectory.at_time(5.0, new_position, new_velocity, new_acceleration, new_jerk, new_section);
        CHECK( array_eq(new_jerk, {0.0, 0.0, 0.0}) );
        CHECK( new_section == 1 );
    }

    SUBCASE("at-time-single-dof") {
        RuckigThrow<1> ruckig {0.005};
        InputParameter<1> input;
        OutputParameter<1> output;

        input.current_position = {0.0};
        input.target_position = {1.0};
        input.max_velocity = {1.0};
        input.max_acceleration = {1.0};
        input.max_jerk = {1.0};

        Trajectory<1> traj;
        auto result = ruckig.calculate(input, traj);

        CHECK( result == Result::Working );
        CHECK( traj.get_duration() == doctest::Approx(3.1748) );

        std::array<double, 1> new_position, new_velocity, new_acceleration;
        traj.at_time(0.0, new_position, new_velocity, new_acceleration);
        CHECK( array_eq(new_position, input.current_position) );

        double new_position_element, new_velocity_element, new_acceleration_element;
        traj.at_time(0.0, new_position_element, new_velocity_element, new_acceleration_element);
        CHECK( new_position_element == doctest::Approx(input.current_position[0]) );

        traj.at_time(3.1748 / 2, new_position_element, new_velocity_element, new_acceleration_element);
        CHECK( new_position_element == doctest::Approx(0.5) );
    }

    SUBCASE("independent-min-durations") {
        const auto independent_min_durations = output.trajectory.get_independent_min_durations();
        CHECK( independent_min_durations[0] == doctest::Approx(3.1748021039) );
        CHECK( independent_min_durations[1] == doctest::Approx(3.6860977315) );
        CHECK( independent_min_durations[2] == doctest::Approx(output.trajectory.get_duration()) );
    }

    SUBCASE("first-time-at-position") {
        CHECK( output.trajectory.get_first_time_at_position(0, 0.0).value() == doctest::Approx(0.0) );
        CHECK( output.trajectory.get_first_time_at_position(0, 0.5).value() == doctest::Approx(2.0) );
        CHECK( output.trajectory.get_first_time_at_position(0, 1.0).value() == doctest::Approx(4.0) );
        CHECK( output.trajectory.get_first_time_at_position(1, -3.0).value() == doctest::Approx(2.6004877902) );
        CHECK( output.trajectory.get_first_time_at_position(1, -3.1).value() == doctest::Approx(2.8644154489) );
        CHECK( output.trajectory.get_first_time_at_position(2, 0.05).value() == doctest::Approx(0.6694329501) );

        CHECK_FALSE( output.trajectory.get_first_time_at_position(0, -1.0) );
        CHECK_FALSE( output.trajectory.get_first_time_at_position(1, -3.4) );
        CHECK_FALSE( output.trajectory.get_first_time_at_position(6, 0.0) );
    }

    input.current_position = {0.0, -2.0, 0.0};
    input.current_velocity = {0.0, 0.0, 0.0};
    input.current_acceleration = {0.0, 0.0, 0.0};
    input.target_position = {1.0, -3.0, 2.0};
    input.target_velocity = {2.0, 0.3, 0.0};
    input.target_acceleration = {0.0, 0.0, 0.0};
    input.max_velocity = {1.0, 1.0, 1.0};
    input.max_acceleration = {1.0, 1.0, 1.0};
    input.max_jerk = {1.0, 1.0, 1.0};

    CHECK_THROWS_WITH_AS( ruckig.update(input, output), doctest::Contains("exceeds its maximum velocity limit"), RuckigError);
    CHECK_FALSE( output.new_calculation );

    input.target_velocity = {0.2, -0.3, 0.8};
    result = ruckig.update(input, output);

    CHECK( result == Result::Working );
    CHECK( output.new_calculation );

    input.minimum_duration = 12.0;
    result = ruckig.update(input, output);

    CHECK( result == Result::Working );
    CHECK( output.trajectory.get_duration() == doctest::Approx(12.0) );


    input.current_position = {1300.0, 0.0, 0.02};
    input.current_velocity = {1200.0, 0.0, 0.0};
    input.current_acceleration = {0.0, 0.0, 0.0};
    input.target_position = {1400.0, 0.0, 0.02};
    input.target_velocity = {0.0, 0.0, 0.0};
    input.target_acceleration = {0.0, 0.0, 0.0};
    input.max_velocity = {800.0, 1.0, 1.0};
    input.max_acceleration = {40000.0, 1.0, 1.0};
    input.max_jerk = {200000.0, 1.0, 1.0};
    input.minimum_duration = std::nullopt;
    result = ruckig.update(input, output);

    CHECK( result == Result::Working );
    CHECK( output.trajectory.get_duration() == doctest::Approx(0.167347) );

    const auto independent_min_durations = output.trajectory.get_independent_min_durations();
    CHECK( independent_min_durations[0] == doctest::Approx(output.trajectory.get_duration()) );
    CHECK( independent_min_durations[1] == doctest::Approx(0.0) );
    CHECK( independent_min_durations[2] == doctest::Approx(0.0) );
}

TEST_CASE("output-is-initialized-before-first-update") {
    const auto check = []<class Output, class... Args>(Args... args) {
        alignas(Output) unsigned char storage[sizeof(Output)];
        std::memset(storage, 0xA5, sizeof(storage));
        Output* output;
        if constexpr (sizeof...(Args) == 0) {
            output = ::new (static_cast<void*>(storage)) Output;
        } else {
            output = ::new (static_cast<void*>(storage)) Output(args...);
        }
        const double zero = 0.0;
        // Compare object representations without reading uninitialized doubles
        // in a regressed implementation. Pre-filled storage makes this reliable.
        CHECK(std::memcmp(&output->calculation_duration, &zero, sizeof(zero)) == 0);
        for (size_t dof = 0; dof < output->degrees_of_freedom; ++dof) {
            CHECK(std::memcmp(&output->new_position[dof], &zero, sizeof(zero)) == 0);
            CHECK(std::memcmp(&output->new_velocity[dof], &zero, sizeof(zero)) == 0);
            CHECK(std::memcmp(&output->new_acceleration[dof], &zero, sizeof(zero)) == 0);
            CHECK(std::memcmp(&output->new_jerk[dof], &zero, sizeof(zero)) == 0);
        }
        output->~Output();
    };
    check.template operator()<OutputParameter<3>>();
    check.template operator()<OutputParameter<>>(3);
    check.template operator()<OutputParameter<0, MinimalRuntimeVector>>(3);
#ifdef WITH_CLOUD_CLIENT
    check.template operator()<OutputParameter<3>>(2);
    check.template operator()<OutputParameter<>>(3, 2);
    check.template operator()<OutputParameter<0, MinimalRuntimeVector>>(3, 2);
#endif
}

TEST_CASE("vector-dimensions") {
    InputParameter<> input(2);
    input.current_position = {0.0, 0.0};
    input.target_position = {1.0, 1.0};
    input.max_velocity = input.max_acceleration = input.max_jerk = {1.0, 1.0};

    const auto valid = input;
    for (auto member: {&InputParameter<>::current_position, &InputParameter<>::current_velocity,
            &InputParameter<>::current_acceleration, &InputParameter<>::target_position,
            &InputParameter<>::target_velocity, &InputParameter<>::target_acceleration,
            &InputParameter<>::max_velocity, &InputParameter<>::max_acceleration,
            &InputParameter<>::max_jerk, &InputParameter<>::min_position, &InputParameter<>::max_position}) {
        for (size_t size: {0, 1, 3}) {
            input = valid;
            (input.*member).resize(size);
            CHECK_FALSE(input.validate<false>());
            CHECK_THROWS_AS(input.validate(), RuckigError);
        }
    }
    input = valid;
    input.min_velocity = std::vector<double>{-1.0};
    CHECK_FALSE(input.validate<false>());
    input = valid;
    input.min_acceleration = std::vector<double>{-1.0};
    CHECK_FALSE(input.validate<false>());
    input = valid;
    input.enabled.resize(1);
    CHECK_FALSE(input.validate<false>());
    input = valid;
    input.per_dof_control_interface = std::vector<ControlInterface>{ControlInterface::Position};
    CHECK_FALSE(input.validate<false>());
    input = valid;
    input.per_dof_synchronization = std::vector<Synchronization>{Synchronization::Time};
    CHECK_FALSE(input.validate<false>());
    input = valid;
    input.intermediate_positions = {{0.5}};
    CHECK_FALSE(input.validate<false>());
    CHECK_FALSE(InputParameter<>(0).validate<false>());

    Ruckig<> otg(2, 0.01);
    Trajectory<> trajectory(1);
    CHECK(otg.calculate(valid, trajectory) == Result::ErrorInvalidInput);
    CHECK_FALSE(otg.validate_input<false>(InputParameter<>(1)));
    OutputParameter<> output(1);
    CHECK(otg.update(valid, output) == Result::ErrorInvalidInput);
}

TEST_CASE("batch-sampling") {
    Ruckig<> otg(1);
    InputParameter<> input(1);
    input.current_position = {0.0};
    input.target_position = {1.0};
    input.max_velocity = input.max_acceleration = input.max_jerk = {1.0};
    Trajectory<> trajectory(1);
    REQUIRE(otg.calculate(input, trajectory) == Result::Working);
    const std::vector<double> times{0.0, 0.5, trajectory.get_duration()};
    std::vector<std::vector<double>> p(3, std::vector<double>(1)), v = p, a = p;
    trajectory.at_times(times, p, v, a);
    for (size_t i = 0; i < times.size(); ++i) {
        std::vector<double> pi(1), vi(1), ai(1);
        trajectory.at_time(times[i], pi, vi, ai);
        CHECK(p[i] == pi);
        CHECK(v[i] == vi);
        CHECK(a[i] == ai);
    }
    p.pop_back();
    CHECK_THROWS_AS(trajectory.at_times(times, p, v, a), RuckigError);
    p.emplace_back();
    CHECK_THROWS_AS(trajectory.at_times(times, p, v, a), RuckigError);
}

TEST_CASE("local-position-limits") {
    Ruckig<1> otg(0.01);
    InputParameter<1> input;
    Trajectory<1> trajectory;
    OutputParameter<1> output;
    input.current_position = {0.0};
    input.target_position = {1.0};
    input.max_velocity = input.max_acceleration = input.max_jerk = {1.0};
    input.min_position = {0.0};
    input.max_position = {1.0};
    REQUIRE(otg.calculate(input, trajectory) == Result::Working);

    // Reject an out-of-bounds target without emitting a new command.
    input.max_position = {0.5};
    output.new_position = {42.0};
    CHECK(otg.update(input, output) == Result::ErrorPositionalLimits);
    CHECK(output.time == 0.0);
    CHECK(output.new_position[0] == 42.0);
    CHECK(otg.update(output.trajectory, output) == Result::ErrorInvalidInput);
    input.max_position = {1.0};
    CHECK(otg.update(input, output) == Result::Working);

    // Both endpoints are in range, but the initial velocity requires overshoot.
    input.min_position = {-10.0};
    input.max_position = {0.2};
    input.target_position = {-1.0};
    input.current_velocity = {1.0};
    CHECK(otg.calculate(input, trajectory) == Result::ErrorPositionalLimits);
    input.max_jerk = {std::numeric_limits<double>::infinity()};
    CHECK(otg.calculate(input, trajectory) == Result::ErrorPositionalLimits);
    std::array<Bound, 1> extrema;
    trajectory.get_position_extrema(extrema);
    CHECK(extrema[0].max == doctest::Approx(0.5));
    CHECK(extrema[0].t_max == doctest::Approx(1.0));

    // Braking starts outside the velocity limit and must also respect position bounds.
    input.current_velocity = {2.0};
    input.max_jerk = {1.0};
    CHECK(otg.calculate(input, trajectory) == Result::ErrorPositionalLimits);

    input.current_velocity = {-2.0};
    input.target_position = {1.0};
    input.min_position = {-0.2};
    input.max_position = {10.0};
    CHECK(otg.calculate(input, trajectory) == Result::ErrorPositionalLimits);

    input.max_position = {std::nan("")};
    CHECK_FALSE(input.validate<false>());
    CHECK_THROWS_AS(input.validate(), RuckigError);
    input.max_position = {-20.0};
    CHECK_FALSE(input.validate<false>());

    // Velocity control also has an absolute position along the generated motion.
    input.control_interface = ControlInterface::Velocity;
    input.min_position = {-10.0};
    input.current_velocity = {1.0};
    input.target_velocity = {0.0};
    input.max_position = {0.25};
    CHECK(otg.calculate(input, trajectory) == Result::ErrorPositionalLimits);
    input.max_position = {2.0};
    CHECK(otg.calculate(input, trajectory) == Result::Working);

    // A disabled axis can continue moving while the other axis reaches its target.
    Ruckig<2> multi(0.01);
    InputParameter<2> multi_input;
    Trajectory<2> multi_trajectory;
    multi_input.current_position = {0.0, 0.0};
    multi_input.target_position = {0.0, 10.0};
    multi_input.current_velocity = {1.0, 0.0};
    multi_input.max_velocity = multi_input.max_acceleration = multi_input.max_jerk = {1.0, 1.0};
    multi_input.enabled = {false, true};
    multi_input.max_position = {1.0, 10.0};
    CHECK(multi.calculate(multi_input, multi_trajectory) == Result::ErrorPositionalLimits);
    multi_input.max_position = {20.0, 10.0};
    CHECK(multi.calculate(multi_input, multi_trajectory) == Result::Working);
    multi_input.enabled[0] = true;
    multi_input.per_dof_synchronization = {Synchronization::None, Synchronization::Time};
    multi_input.target_velocity[0] = 1.0;
    multi_input.max_position[0] = 1.0;
    CHECK(multi.calculate(multi_input, multi_trajectory) == Result::ErrorPositionalLimits);

    // Disabled single-axis profiles must have a valid terminal state as well.
    input.control_interface = ControlInterface::Position;
    input.enabled[0] = false;
    input.current_position = {0.5};
    input.max_position = {2.0};
    REQUIRE(otg.calculate(input, trajectory) == Result::Working);
    CHECK(otg.update(trajectory, output) == Result::Finished);
    CHECK(output.new_position[0] == 0.5);
}

TEST_CASE("offline-trajectory-execution") {
    Ruckig<1> otg(0.3);
    InputParameter<1> input;
    Trajectory<1> trajectory;
    OutputParameter<1> output;
    input.current_position = {0.0};
    input.target_position = {1.0};
    input.target_velocity = {0.5};
    input.max_velocity = input.max_acceleration = input.max_jerk = {1.0};
    CHECK(otg.update(trajectory, output) == Result::ErrorInvalidInput);
    REQUIRE(otg.calculate(input, trajectory) == Result::Working);
    output.time = 0.4;
    Result result = Result::Working;
    for (size_t i = 0; result == Result::Working && i < 100; ++i) {
        result = otg.update(trajectory, output);
        double p = 0.0, v = 0.0, a = 0.0;
        trajectory.at_time(output.time, p, v, a);
        CHECK(output.new_position[0] == p);
        CHECK(output.new_velocity[0] == v);
        CHECK(output.new_acceleration[0] == a);
        CHECK_FALSE(output.new_calculation);
    }
    REQUIRE(result == Result::Finished);
    CHECK(output.time == trajectory.get_duration());
    CHECK(output.new_position[0] == doctest::Approx(1.0));
    CHECK(output.new_velocity[0] == doctest::Approx(0.5));
    CHECK(otg.update(output.trajectory, output) == Result::Finished);
    CHECK(output.new_position[0] == doctest::Approx(1.0));

    for (double bad_cycle: {0.0, -1.0, std::nan(""), std::numeric_limits<double>::infinity()}) {
        otg.delta_time = bad_cycle;
        CHECK(otg.update(trajectory, output) == Result::ErrorInvalidInput);
        CHECK(otg.update(input, output) == Result::ErrorInvalidInput);
    }
    otg.delta_time = 0.1;
    output.time = -1.0;
    CHECK(otg.update(trajectory, output) == Result::ErrorInvalidInput);
    CHECK(otg.update(input, output) == Result::ErrorInvalidInput);
    output.time = std::nan("");
    CHECK(otg.update(trajectory, output) == Result::ErrorInvalidInput);
    CHECK(otg.update(input, output) == Result::ErrorInvalidInput);
    output.time = 0.0;
    Ruckig<> dynamic(2, 0.1);
    OutputParameter<> wrong_output(1);
    Trajectory<> wrong_trajectory(1);
    CHECK(dynamic.update(wrong_trajectory, wrong_output) == Result::ErrorInvalidInput);
    Ruckig<> dynamic_single(1, 0.1);
    InputParameter<> dynamic_input(1);
    dynamic_input.current_position = {0.0};
    dynamic_input.target_position = {1.0};
    dynamic_input.max_velocity = dynamic_input.max_acceleration = dynamic_input.max_jerk = {1.0};
    REQUIRE(dynamic_single.calculate(dynamic_input, wrong_trajectory) == Result::Working);
    wrong_output.new_jerk.clear();
    CHECK(dynamic_single.update(wrong_trajectory, wrong_output) == Result::ErrorInvalidInput);

    // Zero-duration trajectories finish at their initial/terminal state.
    input.target_position = input.current_position;
    input.target_velocity = {0.0};
    REQUIRE(otg.calculate(input, trajectory) == Result::Working);
    CHECK(trajectory.get_duration() == 0.0);
    CHECK(otg.update(trajectory, output) == Result::Finished);
    CHECK(output.time == 0.0);
    CHECK(output.new_position[0] == 0.0);

    // A fresh output object cannot reuse another object's cached trajectory.
    input.target_position = {1.0};
    output = OutputParameter<1>();
    REQUIRE(otg.update(input, output) == Result::Working);
    output.pass_to_input(input);
    OutputParameter<1> fresh_output;
    REQUIRE(otg.update(input, fresh_output) == Result::Working);
    CHECK(fresh_output.new_calculation);
}

TEST_CASE("pass-to-input-keeps-only-unpassed-waypoints") {
    Ruckig<1> otg;
    InputParameter<1> input;
    input.current_position = {0.0};
    input.target_position = {4.0};
    input.intermediate_positions = {{1.0}, {2.0}, {3.0}};
    input.max_velocity = input.max_acceleration = {2.0};
    input.max_jerk = {4.0};
    const auto original = input;
    OutputParameter<1> output;
    REQUIRE(otg.calculate_waypoints(input, output.trajectory) == Result::Working);
    const auto sample = [&](double time, bool new_calculation) {
        const size_t previous = output.new_section;
        output.trajectory.at_time(time, output.new_position, output.new_velocity,
            output.new_acceleration, output.new_jerk, output.new_section);
        output.new_calculation = new_calculation;
        output.did_section_change = output.new_section > previous;
        output.time = time;
    };
    sample(5.0, true);
    REQUIRE(output.new_section == 2);
    output.pass_to_input(input);
    CHECK(input.intermediate_positions == std::vector<std::array<double, 1>>{{3.0}});
    CHECK(input.current_position == output.new_position);
    output.pass_to_input(input);
    CHECK(input.intermediate_positions == std::vector<std::array<double, 1>>{{3.0}});
    sample(8.0, false);
    output.pass_to_input(input);
    CHECK(input.intermediate_positions.empty());

    input = original;
    output.new_section = 0;
    for (double time: {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0}) {
        sample(time, false);
        output.pass_to_input(input);
        CHECK(input.intermediate_positions.size() == 3 - std::min<size_t>(output.new_section, 3));
    }

    // The previous trajectory ended in section 4. A new calculation can pass
    // zero-duration sections while its section index is lower than before.
    input = original;
    input.target_position = {3.0};
    input.intermediate_positions = {{0.0}, {0.0}, {1.0}, {2.0}};
    REQUIRE(otg.calculate_waypoints(input, output.trajectory) == Result::Working);
    sample(0.01, true);
    REQUIRE(output.new_section == 2);
    REQUIRE_FALSE(output.did_section_change);
    output.pass_to_input(input);
    CHECK(input.intermediate_positions == std::vector<std::array<double, 1>>{{1.0}, {2.0}});
    output.pass_to_input(input);
    CHECK(input.intermediate_positions == std::vector<std::array<double, 1>>{{1.0}, {2.0}});

    input = original;
    input.target_position = {0.0};
    input.intermediate_positions = {{0.0}, {0.0}};
    REQUIRE(otg.calculate_waypoints(input, output.trajectory) == Result::Working);
    sample(0.0, true);
    output.pass_to_input(input);
    CHECK(input.intermediate_positions.empty());

    // Equal positions with different prescribed states are distinct visits.
    input = original;
    input.intermediate_positions = {{1.0}, {1.0}, {2.0}};
    REQUIRE(otg.calculate_waypoints(input, output.trajectory, {{0.5}, {-0.5}, {0.0}}) == Result::Working);
    const auto visits = output.trajectory.get_intermediate_durations();
    REQUIRE(visits[1] > visits[0]);
    sample(visits[0], true);
    output.pass_to_input(input);
    CHECK(input.intermediate_positions == std::vector<std::array<double, 1>>{{1.0}, {2.0}});
    sample((visits[0] + visits[1]) / 2.0, false);
    output.pass_to_input(input);
    CHECK(input.intermediate_positions == std::vector<std::array<double, 1>>{{1.0}, {2.0}});
    sample(visits[1], false);
    output.pass_to_input(input);
    CHECK(input.intermediate_positions == std::vector<std::array<double, 1>>{{2.0}});

    // Preserve the helper's existing behavior for manually supplied states
    // that have no successfully calculated trajectory.
    input = original;
    OutputParameter<1> manual;
    manual.new_position = {0.5};
    manual.new_velocity = manual.new_acceleration = {0.0};
    manual.did_section_change = true;
    manual.pass_to_input(input);
    CHECK(input.intermediate_positions == std::vector<std::array<double, 1>>{{2.0}, {3.0}});
}

TEST_CASE("input-validation") {
    RuckigThrow<DynamicDOFs> ruckig {2};
    InputParameter input {2};

    const double nan = std::nan("");

    input.current_position = {0.0, -2.0};
    input.current_velocity = {0.0, 0.0};
    input.current_acceleration = {0.0, 0.0};
    input.target_position = {1.0, -3.0};
    input.target_velocity = {0.0, 0.3};
    input.target_acceleration = {0.0, 0.0};
    input.max_velocity = {1.0, 1.0};
    input.max_acceleration = {1.0, 1.0};
    input.max_jerk = {1.0, 1.0};

    CHECK( ruckig.validate_input(input) );
    CHECK( ruckig.validate_input<false>(input) );

    input.max_jerk = {1.0, nan};
    CHECK_THROWS_WITH_AS( ruckig.validate_input(input), doctest::Contains("maximum jerk limit"), RuckigError);
    CHECK_FALSE( ruckig.validate_input<false>(input) );

    input.max_jerk = {1.0, 1.0};
    input.current_position = {1.0, nan};
    CHECK_THROWS_WITH_AS( ruckig.validate_input(input), doctest::Contains("current position"), RuckigError);
    CHECK_FALSE( ruckig.validate_input<false>(input) );

    input.current_position = {1.0, 1.0};
    input.max_acceleration = {1.0, -1.0};
    CHECK_THROWS_WITH_AS( ruckig.validate_input(input), doctest::Contains("maximum acceleration limit"), RuckigError);
    CHECK_FALSE( ruckig.validate_input<false>(input) );

    input.max_acceleration = {1.0, 1.0};
    input.max_velocity = {1.0, -1.0};
    CHECK_THROWS_WITH_AS( ruckig.validate_input(input), doctest::Contains("maximum velocity limit"), RuckigError);
    CHECK_FALSE( ruckig.validate_input<false>(input) );

    input.max_velocity = {1.0, 1.0};
    input.target_velocity = {0.0, 1.3};
    CHECK( ruckig.validate_input(input, false, false) );
    CHECK_THROWS_WITH_AS( ruckig.validate_input(input), doctest::Contains("exceeds its maximum velocity limit"), RuckigError);
    CHECK_THROWS_WITH_AS( ruckig.validate_input(input, false, true), doctest::Contains("exceeds its maximum velocity limit"), RuckigError);
    CHECK_FALSE( ruckig.validate_input<false>(input, false, true) );
    CHECK_FALSE( ruckig.validate_input<false>(input) );

    input.target_velocity = {0.0, 0.3};
    input.current_velocity = {2.0, 0.0};
    CHECK( ruckig.validate_input(input, false, false) );
    CHECK_THROWS_WITH_AS( ruckig.validate_input(input, true, false), doctest::Contains("exceeds its maximum velocity limit"), RuckigError);
    CHECK_FALSE( ruckig.validate_input<false>(input, true, false) );
    CHECK_FALSE( ruckig.validate_input<false>(input, true, true) );
    CHECK( ruckig.validate_input(input) );

    input.current_velocity = {1.0, 0.0};
    input.current_acceleration = {-1.0, 0.0};
    CHECK( ruckig.validate_input(input) );
    CHECK( ruckig.validate_input(input, true, true) );

    input.current_velocity = {1.0, 0.0};
    input.current_acceleration = {1.0, 0.0};
    CHECK( ruckig.validate_input(input) );
    CHECK_THROWS_WITH_AS( ruckig.validate_input(input, true, true), doctest::Contains("will exceed its maximum velocity limit"), RuckigError);
    CHECK_FALSE( ruckig.validate_input<false>(input, true, true) );

    input.current_velocity = {0.72, 0.0};
    input.current_acceleration = {0.72, 0.0};
    CHECK( ruckig.validate_input(input, true, true) );

    input.current_velocity = {0.0, 0.0};
    input.current_acceleration = {0.0, 0.0};
    input.target_velocity = {0.0, 0.72};
    input.target_acceleration = {0.0, 0.72};
    CHECK( ruckig.validate_input(input) );
    CHECK( ruckig.validate_input<false>(input) );

    input.target_velocity = {0.0, 1.0};
    input.target_acceleration = {0.0, 1.0};
    CHECK( ruckig.validate_input(input) );

    input.target_velocity = {0.0, 1.0};
    input.target_acceleration = {0.0, -0.0001};
    CHECK_THROWS_WITH_AS( ruckig.validate_input(input), doctest::Contains("will exceed its maximum velocity"), RuckigError);
    CHECK_FALSE( ruckig.validate_input<false>(input) );
}

TEST_CASE("enabled") {
    RuckigThrow<DynamicDOFs> ruckig {3, 0.005};
    InputParameter input {3};
    OutputParameter output {3};

    input.enabled = {true, false, false};
    input.current_position = {0.0, -2.0, 0.0};
    input.current_velocity = {0.0, 0.1, 0.0};
    input.current_acceleration = {0.0, 0.0, -0.2};
    input.target_position = {1.0, -3.0, 2.0};
    input.max_velocity = {1.0, 1.0, 1.0};
    input.max_acceleration = {1.0, 1.0, 1.0};
    input.max_jerk = {1.0, 1.0, 1.0};

    Result result = ruckig.update(input, output);
    CHECK( result == Result::Working );
    CHECK( output.trajectory.get_duration() == doctest::Approx(3.1748021039) );

    std::vector<double> new_position(3), new_velocity(3), new_acceleration(3);
    output.trajectory.at_time(0.0, new_position, new_velocity, new_acceleration);
    array_eq(new_position, input.current_position);
    array_eq(new_velocity, input.current_velocity);
    array_eq(new_acceleration, input.current_acceleration);

    output.trajectory.at_time(output.trajectory.get_duration(), new_position, new_velocity, new_acceleration);
    array_eq(new_position, {input.target_position[0], -1.6825197896, -1.0079368399});

    // Make sure that disabled DoFs overwrite prior blocks
    input.enabled = {true, true, true};
    input.current_position = {0.0, 0.0, 0.0};
    input.target_position = {100.0, -3000.0, 2000.0};
    input.target_velocity = {1.0, 1.0, 1.0};
    result = ruckig.update(input, output);

    input.enabled = {false, false, true};
    input.current_position = {0.0, -2.0, 0.0};
    input.current_velocity = {0.0, 0.2, 0.0};
    input.current_acceleration = {0.0, 0.2, 0.0};
    input.target_position = {1.0, -3.0, 2.0};
    input.target_velocity = {0.0, 0.0, 0.2};
    input.target_acceleration = {0.0, 0.0, -0.1};
    input.max_velocity = {1.0, 1.0, 1.0};
    input.max_acceleration = {1.0, 1.0, 1.0};
    input.max_jerk = {1.0, 1.0, 1.0};

    result = ruckig.update(input, output);
    CHECK( result == Result::Working );
    CHECK( output.trajectory.get_duration() == doctest::Approx(3.6578610221) );
    const auto independent = output.trajectory.get_independent_min_durations();
    CHECK( independent[0] == 0.0 );
    CHECK( independent[1] == 0.0 );
    CHECK( independent[2] > 0.0 );
}

TEST_CASE("phase-synchronization") {
    RuckigThrow<3> ruckig {0.005};
    InputParameter<3> input;
    OutputParameter<3> output;
    Trajectory<3> traj;
    std::array<double, 3> new_position, new_velocity, new_acceleration;

    input.current_position = {0.0, -2.0, 0.0};
    input.target_position = {1.0, -3.0, 2.0};
    input.max_velocity = {1.0, 1.0, 1.0};
    input.max_acceleration = {1.0, 1.0, 1.0};
    input.max_jerk = {1.0, 1.0, 1.0};
    input.synchronization = Synchronization::Phase;
    auto result = ruckig.calculate(input, traj);
    CHECK( result == Result::Working );
    CHECK( traj.get_duration() == doctest::Approx(4.0) );
    CHECK( array_eq(traj.get_profiles()[0][0].t, traj.get_profiles()[0][1].t) );
    CHECK( array_eq(traj.get_profiles()[0][0].t, traj.get_profiles()[0][2].t) );

    result = ruckig.update(input, output);
    output.trajectory.at_time(1.0, new_position, new_velocity, new_acceleration);
    CHECK( result == Result::Working );
    CHECK( output.trajectory.get_duration() == doctest::Approx(4.0) );
    CHECK( array_eq(new_position, {0.0833333333, -2.0833333333, 0.1666666667}) );
    CHECK( array_eq(output.trajectory.get_profiles()[0][0].t, output.trajectory.get_profiles()[0][1].t) );
    CHECK( array_eq(output.trajectory.get_profiles()[0][0].t, output.trajectory.get_profiles()[0][2].t) );

    input.current_position = {0.0, -2.0, 0.0};
    input.target_position = {10.0, -3.0, 2.0};
    input.max_velocity = {10.0, 2.0, 1.0};
    input.max_acceleration = {10.0, 2.0, 1.0};
    input.max_jerk = {10.0, 2.0, 1.0};
    result = ruckig.update(input, output);
    output.trajectory.at_time(1.0, new_position, new_velocity, new_acceleration);
    CHECK( result == Result::Working );
    CHECK( output.trajectory.get_duration() == doctest::Approx(4.0) );
    CHECK( array_eq(new_position, {0.8333333333, -2.0833333333, 0.1666666667}) );
    CHECK( array_eq(output.trajectory.get_profiles()[0][0].t, output.trajectory.get_profiles()[0][1].t) );
    CHECK( array_eq(output.trajectory.get_profiles()[0][0].t, output.trajectory.get_profiles()[0][2].t) );

    // Test equal start and target state
    input.current_position = {1.0, -2.0, 3.0};
    input.target_position = {1.0, -2.0, 3.0};
    result = ruckig.update(input, output);
    output.trajectory.at_time(0.0, new_position, new_velocity, new_acceleration);
    CHECK( result == Result::Finished );
    CHECK( output.trajectory.get_duration() == doctest::Approx(0.0) );
    CHECK( array_eq(new_position, {1.0, -2.0, 3.0}) );
    CHECK( array_eq(output.trajectory.get_profiles()[0][0].t, output.trajectory.get_profiles()[0][1].t) );
    CHECK( array_eq(output.trajectory.get_profiles()[0][0].t, output.trajectory.get_profiles()[0][2].t) );

    input.current_position = {0.0, 0.0, 0.0};
    input.current_velocity = {0.0, 0.0, 0.0};
    input.current_acceleration = {0.0, 0.0, 0.0};
    input.target_position = {0.0, 0.0, 0.0};
    input.target_velocity = {0.2, 0.3, 0.4};
    input.target_acceleration = {0.0, 0.0, 0.0};
    input.max_velocity = {1.0, 1.0, 1.0};
    input.max_acceleration = {1.0, 1.0, 1.0};
    input.max_jerk = {1.0, 1.0, 1.0};
    result = ruckig.calculate(input, traj);
    CHECK( result == Result::Working );
    CHECK( array_eq(traj.get_profiles()[0][0].t, traj.get_profiles()[0][1].t) );
    CHECK( array_eq(traj.get_profiles()[0][0].t, traj.get_profiles()[0][2].t) );

    input.current_position = {0.0, 0.0, 0.0};
    input.current_velocity = {0.0, 0.0, 0.0};
    input.current_acceleration = {0.0, 0.0, 0.0};
    input.target_position = {0.0, 0.0, 0.01};
    input.target_velocity = {0.2, 0.3, 0.4};
    input.target_acceleration = {0.0, 0.0, 0.0};
    result = ruckig.calculate(input, traj);
    CHECK( result == Result::Working );
    CHECK_FALSE( array_eq(traj.get_profiles()[0][0].t, traj.get_profiles()[0][1].t) );
    CHECK_FALSE( array_eq(traj.get_profiles()[0][0].t, traj.get_profiles()[0][2].t) );

    input.current_position = {0.0, 0.0, 0.0};
    input.current_velocity = {0.4, 0.15, 0.2};
    input.current_acceleration = {0.8, 0.3, 0.4};
    input.target_position = {0.0, 0.0, 0.0};
    input.target_velocity = {0.0, 0.0, 0.0};
    input.target_acceleration = {0.0, 0.0, 0.0};
    result = ruckig.calculate(input, traj);
    CHECK( result == Result::Working );
    CHECK( array_eq(traj.get_profiles()[0][0].t, traj.get_profiles()[0][1].t) );
    CHECK( array_eq(traj.get_profiles()[0][0].t, traj.get_profiles()[0][2].t) );

    input.max_velocity = {1.0, 0.2, 1.0};
    result = ruckig.calculate(input, traj);
    CHECK( result == Result::Working );
    CHECK_FALSE( array_eq(traj.get_profiles()[0][0].t, traj.get_profiles()[0][1].t) );
    CHECK_FALSE( array_eq(traj.get_profiles()[0][0].t, traj.get_profiles()[0][2].t) );

    input.current_position = {0.0, 0.02, 1.0};
    input.current_velocity = {-0.2, 0.15, 0.2};
    input.current_acceleration = {-0.4, 0.3, 0.4};
    input.target_position = {0.03, 0.0, 0.0};
    input.target_velocity = {-0.02, 0.015, 0.02};
    input.target_acceleration = {0.0, 0.0, 0.0};
    input.max_velocity = {1.0, 1.0, 1.0};
    input.max_acceleration = {1.0, 1.0, 1.0};
    input.max_jerk = {1.0, 1.0, 1.0};
    input.control_interface = ControlInterface::Velocity;
    result = ruckig.calculate(input, traj);
    CHECK( result == Result::Working );
    CHECK( array_eq(traj.get_profiles()[0][0].t, traj.get_profiles()[0][1].t) );
    CHECK( array_eq(traj.get_profiles()[0][0].t, traj.get_profiles()[0][2].t) );

    input.max_jerk = {1.0, 0.1, 1.0};
    result = ruckig.calculate(input, traj);
    CHECK( result == Result::Working );
    CHECK( array_eq(traj.get_profiles()[0][0].t, traj.get_profiles()[0][1].t) );
    CHECK( array_eq(traj.get_profiles()[0][0].t, traj.get_profiles()[0][2].t) );

    input.target_acceleration = {0.01, 0.0, 0.0};
    result = ruckig.calculate(input, traj);
    CHECK( result == Result::Working );
    CHECK_FALSE( array_eq(traj.get_profiles()[0][0].t, traj.get_profiles()[0][1].t) );
    CHECK_FALSE( array_eq(traj.get_profiles()[0][0].t, traj.get_profiles()[0][2].t) );

    input.current_position = {0.0, 0.0, 0.0};
    input.current_velocity = {0.0, 0.0, 0.0};
    input.current_acceleration = {0.0, 0.0, 0.0};
    input.target_position = {0.0, 0.0, 0.0};
    input.target_velocity = {0.0, 0.0, 0.0};
    input.target_acceleration = {0.0, 0.0, 0.0};
    result = ruckig.calculate(input, traj);
    CHECK( result == Result::Working );
}

TEST_CASE("zero-limits") {
    RuckigThrow<DynamicDOFs> ruckig {3, 0.005};
    InputParameter input {3};
    OutputParameter output {3};

    input.current_position = {0.0, -2.0, 0.0};
    input.current_velocity = {0.2, 0.0, 0.0};
    input.current_acceleration = {0.0, 0.0, 0.0};
    input.target_position = {1.0, -3.0, 0.0};
    input.target_velocity = {0.2, 0.0, 0.0};
    input.target_acceleration = {0.0, 0.0, 0.0};
    input.max_velocity = {1.0, 1.0, 1.0};
    input.max_acceleration = {0.0, 1.0, 0.0};
    input.max_jerk = {0.0, 1.0, 0.0};

    auto result = ruckig.update(input, output);

    CHECK( result == Result::Working );
    CHECK( output.trajectory.get_duration() == doctest::Approx(5.0) );


    input.current_position = {0.0, -2.0, 0.0};
    input.current_velocity = {-0.2, 0.0, 0.0};
    input.current_acceleration = {1.0, 0.0, 0.0};
    input.target_position = {0.4, -3.0, 0.0};
    input.target_velocity = {0.8, 0.0, 0.0};
    input.target_acceleration = {1.0, 0.0, 0.0};
    input.max_velocity = {1.0, 200.0, 1.0};
    input.max_acceleration = {1.0, 200.0, 0.0};
    input.max_jerk = {0.0, 200.0, 0.0};

    CHECK_THROWS_WITH_AS( ruckig.update(input, output), doctest::Contains("zero limits conflict in step 1"), RuckigError);


    input.target_position = {0.3, -3.0, 0.0};
    input.max_velocity = {1.0, 2.0, 1.0};
    input.max_acceleration = {1.0, 2.0, 0.0};
    input.max_jerk = {0.0, 2.0, 0.0};

    CHECK_THROWS_WITH_AS( ruckig.update(input, output), doctest::Contains("zero limits conflict with other"), RuckigError);


    input.control_interface = ControlInterface::Velocity;
    input.current_position = {0.0, -2.0, 0.0};
    input.current_velocity = {-0.2, 0.0, 0.0};
    input.current_acceleration = {1.0, 0.0, 0.2};
    input.target_position = {0.4, -3.0, 0.0};
    input.target_velocity = {0.9, 0.5, 0.4};
    input.target_acceleration = {1.0, 0.0, 0.2};
    input.max_velocity = {1.0, 2.0, 1.0};
    input.max_acceleration = {1.0, 2.0, 6.0};
    input.max_jerk = {0.0, 2.0, 0.0};

    CHECK_THROWS_WITH_AS( ruckig.update(input, output), doctest::Contains("zero limits conflict with other"), RuckigError);

    input.max_jerk = {1.0, 2.0, 0.0};

    result = ruckig.update(input, output);

    CHECK( result == Result::Working );
    CHECK( output.trajectory.get_duration() == doctest::Approx(2.0) );

    input.max_jerk = {0.0, 2.0, 20.0};

    result = ruckig.update(input, output);

    CHECK( result == Result::Working );
    CHECK( output.trajectory.get_duration() == doctest::Approx(1.1) );
}

TEST_CASE("custom-vector-type") {
    SUBCASE("DOFs run-time") {
        RuckigThrow<DynamicDOFs, MinimalRuntimeVector> ruckig {3, 0.005};
        InputParameter<DynamicDOFs, MinimalRuntimeVector> input {3};
        OutputParameter<DynamicDOFs, MinimalRuntimeVector> output {3};

        input.current_position = {0.0, -2.0, 0.0};
        input.current_velocity = {0.0, 0.0, 0.0};
        input.current_acceleration = {0.0, 0.0, 0.0};
        input.target_position = {1.0, -3.0, 2.0};
        input.target_velocity = {0.0, 0.3, 0.0};
        input.target_acceleration = {0.0, 0.0, 0.0};
        input.max_velocity = {1.0, 1.0, 1.0};
        input.max_acceleration = {1.0, 1.0, 1.0};
        input.max_jerk = {1.0, 1.0, 1.0};

        auto result = ruckig.update(input, output);

        CHECK( result == Result::Working );
        CHECK( output.trajectory.get_duration() == doctest::Approx(4.0) );

        MinimalRuntimeVector<double, DynamicDOFs> new_position, new_velocity, new_acceleration;
        new_position.resize(3);
        new_velocity.resize(3);
        new_acceleration.resize(3);
        output.trajectory.at_time(0.0, new_position, new_velocity, new_acceleration);
        CHECK( new_position[0] == doctest::Approx(input.current_position[0]) );
        CHECK( new_position[1] == doctest::Approx(input.current_position[1]) );
        CHECK( new_position[2] == doctest::Approx(input.current_position[2]) );
    }
}

TEST_CASE("random-discrete-3") {
    const size_t DOFs = 3;
    RuckigThrow<DOFs> ruckig {0.005};
    InputParameter<DOFs> input;

    std::uniform_int_distribution<int> position_discrete_dist(-1, 1);
    std::uniform_int_distribution<int> dynamic_discrete_dist(-1, 1);
    std::uniform_int_distribution<int> limit_discrete_dist(1, 2);

    Randomizer<DOFs, decltype(position_discrete_dist)> p { position_discrete_dist, seed };
    Randomizer<DOFs, decltype(dynamic_discrete_dist)> d { dynamic_discrete_dist, seed + 1 };
    Randomizer<DOFs, decltype(limit_discrete_dist)> l { limit_discrete_dist, seed + 2 };

    for (size_t i = 0; i < random_discrete_3; ++i) {
        if (i < random_discrete_3 / 2) {
            input.synchronization = Synchronization::Phase;
        } else {
            input.synchronization = Synchronization::Time;
        }

        p.fill(input.current_position);
        d.fill(input.current_velocity);
        d.fill(input.current_acceleration);
        p.fill(input.target_position);
        d.fill(input.target_velocity);
        d.fill(input.target_acceleration);
        l.fill(input.max_velocity, input.target_velocity);
        l.fill(input.max_acceleration, input.target_acceleration);
        l.fill(input.max_jerk);

        if (!ruckig.validate_input<false>(input)) {
            --i;
            continue;
        }

        check_calculation(ruckig, input);
    }
}

TEST_CASE("velocity-random-3") {
    const size_t DOFs = 3;
    RuckigThrow<DOFs> ruckig {0.005};
    InputParameter<DOFs> input;
    input.control_interface = ControlInterface::Velocity;

    Randomizer<DOFs, decltype(position_dist)> p { position_dist, seed + 3 };
    Randomizer<DOFs, decltype(dynamic_dist)> d { dynamic_dist, seed + 4 };
    Randomizer<DOFs, decltype(limit_dist)> l { limit_dist, seed + 5 };

    input.current_position = {0.0, 0.0, 0.0};

    for (size_t i = 0; i < velocity_random_3; ++i) {
        d.fill_or_zero(input.current_velocity, 0.9);
        d.fill_or_zero(input.current_acceleration, 0.8);
        d.fill_or_zero(input.target_velocity, 0.7);
        d.fill_or_zero(input.target_acceleration, 0.6);
        l.fill(input.max_acceleration, input.target_acceleration);
        l.fill(input.max_jerk);

        check_calculation(ruckig, input);
    }
}

TEST_CASE("velocity-random-discrete-3") {
    const size_t DOFs = 3;
    RuckigThrow<DOFs> ruckig {0.005};
    InputParameter<DOFs> input;
    input.control_interface = ControlInterface::Velocity;

    std::uniform_int_distribution<int> position_discrete_dist(-1, 1);
    std::uniform_int_distribution<int> dynamic_discrete_dist(-1, 1);
    std::uniform_int_distribution<int> limit_discrete_dist(1, 2);

    Randomizer<DOFs, decltype(position_discrete_dist)> p { position_discrete_dist, seed };
    Randomizer<DOFs, decltype(dynamic_discrete_dist)> d { dynamic_discrete_dist, seed + 1 };
    Randomizer<DOFs, decltype(limit_discrete_dist)> l { limit_discrete_dist, seed + 2 };

    for (size_t i = 0; i < velocity_random_discrete_3; ++i) {
        if (i < velocity_random_discrete_3 / 2) {
            input.synchronization = Synchronization::Phase;
        } else {
            input.synchronization = Synchronization::Time;
        }

        p.fill(input.current_position);
        d.fill(input.current_velocity);
        d.fill(input.current_acceleration);
        p.fill(input.target_position);
        d.fill(input.target_velocity);
        d.fill(input.target_acceleration);
        l.fill(input.max_velocity, input.target_velocity);
        l.fill(input.max_acceleration, input.target_acceleration);
        l.fill(input.max_jerk);

        if (!ruckig.validate_input<false>(input)) {
            --i;
            continue;
        }

        check_calculation(ruckig, input);
    }
}

TEST_CASE("velocity-second-random-3") {
    const size_t DOFs = 3;
    RuckigThrow<DOFs> ruckig {0.005};
    InputParameter<DOFs> input;
    input.control_interface = ControlInterface::Velocity;

    Randomizer<DOFs, decltype(position_dist)> p { position_dist, seed + 3 };
    Randomizer<DOFs, decltype(dynamic_dist)> d { dynamic_dist, seed + 4 };
    Randomizer<DOFs, decltype(limit_dist)> l { limit_dist, seed + 5 };
    Randomizer<DOFs, decltype(min_limit_dist)> min_l { min_limit_dist, seed + 6 };

    input.current_position = {0.0, 0.0, 0.0};
    // input.min_acceleration = {0.0, 0.0, 0.0}; // To "activate" std::optional

    for (size_t i = 0; i < velocity_second_random_3; ++i) {
        d.fill_or_zero(input.current_velocity, 0.9);
        d.fill_or_zero(input.current_acceleration, 0.8);
        d.fill_or_zero(input.target_velocity, 0.7);
        d.fill_or_zero(input.target_acceleration, 0.6);
        l.fill(input.max_acceleration, input.target_acceleration);
        // min_l.fill_min(*input.min_acceleration, input.target_acceleration);

        check_calculation(ruckig, input);
    }
}

TEST_CASE("random-3-high") {
    const size_t DOFs = 3;
    RuckigThrow<DOFs> ruckig {0.005};
    InputParameter<DOFs> input;

    Randomizer<DOFs, decltype(position_dist)> p { position_dist, seed + 3 };
    Randomizer<DOFs, decltype(dynamic_dist)> d { dynamic_dist, seed + 4 };
    Randomizer<DOFs, decltype(limit_dist_high)> l { limit_dist_high, seed + 5 };

    for (size_t i = 0; i < random_3_high; ++i) {
        p.fill(input.current_position);
        d.fill_or_zero(input.current_velocity, 0.1);
        d.fill_or_zero(input.current_acceleration, 0.1);
        p.fill(input.target_position);
        d.fill_or_zero(input.target_velocity, 0.1);
        d.fill_or_zero(input.target_acceleration, 0.1);
        l.fill(input.max_velocity, input.target_velocity);
        l.fill(input.max_acceleration, input.target_acceleration);
        l.fill(input.max_jerk);

        if (!ruckig.validate_input<false>(input)) {
            --i;
            continue;
        }

        check_calculation(ruckig, input);
    }
}

TEST_CASE("random-direction-3") {
    const size_t DOFs = 3;
    RuckigThrow<DOFs> ruckig {0.005};
    InputParameter<DOFs> input;

    Randomizer<DOFs, decltype(position_dist)> p { position_dist, seed + 3 };
    Randomizer<DOFs, decltype(dynamic_dist)> d { dynamic_dist, seed + 4 };
    Randomizer<DOFs, decltype(limit_dist)> l { limit_dist, seed + 5 };
    Randomizer<DOFs, decltype(min_limit_dist)> min_l { min_limit_dist, seed + 5 };

    // To "activate" std::optional
    input.min_velocity = StandardVector<double, DOFs> {0.0, 0.0, 0.0};
    input.min_acceleration = StandardVector<double, DOFs> {0.0, 0.0, 0.0};

    for (size_t i = 0; i < random_direction_3; ++i) {
        p.fill(input.current_position);
        d.fill_or_zero(input.current_velocity, 0.9);
        d.fill_or_zero(input.current_acceleration, 0.8);
        p.fill(input.target_position);
        d.fill_or_zero(input.target_velocity, 0.7);
        d.fill_or_zero(input.target_acceleration, 0.6);
        l.fill(input.max_velocity, input.target_velocity);
        l.fill(input.max_acceleration, input.target_acceleration);
        l.fill(input.max_jerk);
        min_l.fill_min(*input.min_velocity, input.target_velocity);
        min_l.fill_min(*input.min_acceleration, input.target_acceleration);

        if (!ruckig.validate_input<false>(input)) {
            --i;
            continue;
        }

        check_calculation(ruckig, input);
    }
}

TEST_CASE("position-random-3") {
    const size_t DOFs = 3;
    RuckigThrow<DOFs> ruckig {0.005};
    InputParameter<DOFs> input;

    Randomizer<DOFs, decltype(position_dist)> p { position_dist, seed + 3 };
    Randomizer<DOFs, decltype(dynamic_dist)> d { dynamic_dist, seed + 4 };
    Randomizer<DOFs, decltype(limit_dist)> l { limit_dist, seed + 5 };

    for (size_t i = 0; i < position_random_3; ++i) {
        if (i < position_random_3 / 2) {
            input.synchronization = Synchronization::Phase;
        } else {
            input.synchronization = Synchronization::Time;
        }

        if (i < position_random_3 / 20) {
            input.duration_discretization = DurationDiscretization::Discrete;
        } else {
            input.duration_discretization = DurationDiscretization::Continuous;
        }

        p.fill(input.current_position);
        d.fill_or_zero(input.current_velocity, 0.9);
        d.fill_or_zero(input.current_acceleration, 0.8);
        p.fill(input.target_position);
        d.fill_or_zero(input.target_velocity, 0.7);
        d.fill_or_zero(input.target_acceleration, 0.6);
        l.fill(input.max_velocity, input.target_velocity);
        l.fill(input.max_acceleration, input.target_acceleration);
        l.fill(input.max_jerk);

        // if (i % 1000 == 0) {
        //     const double factor = 1e3;
        //     for (size_t d = 0; d < DOFs; ++d) {
        //         input.current_position[d] *= factor;
        //         input.current_velocity[d] *= factor;
        //         input.current_acceleration[d] *= factor;
        //         input.target_position[d] *= factor;
        //         input.target_velocity[d] *= factor;
        //         input.target_acceleration[d] *= factor;
        //         input.max_velocity[d] *= factor;
        //         input.max_acceleration[d] *= factor;
        //         input.max_jerk[d] *= factor;
        //     }
        // }

        if (!ruckig.validate_input<false>(input)) {
            --i;
            continue;
        }

        check_calculation(ruckig, input);
    }
}

TEST_CASE("position-second-random-3") {
    const size_t DOFs = 3;
    RuckigThrow<DOFs> ruckig {0.005};
    InputParameter<DOFs> input;

    Randomizer<DOFs, decltype(position_dist)> p { position_dist, seed + 3 };
    Randomizer<DOFs, decltype(dynamic_dist)> d { dynamic_dist, seed + 4 };
    Randomizer<DOFs, decltype(limit_dist)> l { limit_dist, seed + 5 };

    for (size_t i = 0; i < position_second_random_3; ++i) {
        if (i < position_second_random_3 / 2) {
            input.synchronization = Synchronization::Phase;
        } else {
            input.synchronization = Synchronization::Time;
        }

        if (i < position_second_random_3 / 20) {
            input.duration_discretization = DurationDiscretization::Discrete;
        } else {
            input.duration_discretization = DurationDiscretization::Continuous;
        }

        p.fill(input.current_position);
        d.fill_or_zero(input.current_velocity, 0.9);
        d.fill_or_zero(input.current_acceleration, 0.8);
        p.fill(input.target_position);
        d.fill_or_zero(input.target_velocity, 0.7);
        d.fill_or_zero(input.target_acceleration, 0.6);
        l.fill(input.max_velocity, input.target_velocity);
        l.fill(input.max_acceleration, input.target_acceleration);

        if (!ruckig.validate_input<false>(input)) {
            --i;
            continue;
        }

        check_calculation(ruckig, input);
    }
}


TEST_CASE("coincident-extremal-roots") {
    Ruckig<1> otg;
    for (double sign: {-1.0, 1.0}) {
        InputParameter<1> input;
        input.current_position = {sign};
        input.target_position = {sign * 2.0};
        input.current_velocity = input.target_velocity = {sign * 1.8693742147505434};
        input.current_acceleration = {sign * 1.0222554876329362};
        input.target_acceleration = {sign * -1.0222554876329362};
        input.max_velocity = input.max_acceleration = {2.0};
        input.max_jerk = {4.0};
        Trajectory<1> trajectory;
        REQUIRE(otg.calculate(input, trajectory) == Result::Working);
        CHECK(trajectory.get_duration() == doctest::Approx(0.5111277438164681).epsilon(1e-10));
        double p = 0.0, v = 0.0, a = 0.0;
        trajectory.at_time(trajectory.get_duration(), p, v, a);
        CHECK(p == doctest::Approx(input.target_position[0]).epsilon(1e-9));
        CHECK(v == doctest::Approx(input.target_velocity[0]).epsilon(1e-9));
        CHECK(a == doctest::Approx(input.target_acceleration[0]).epsilon(1e-9));
    }
}


TEST_CASE("identical-moving-state-needs-no-time") {
    Ruckig<2> otg(0.01);
    InputParameter<2> input;
    input.current_position = input.target_position = {1.0, -2.0};
    input.current_velocity = input.target_velocity = {0.75, -0.5};
    input.current_acceleration = input.target_acceleration = {0.25, -0.125};
    input.max_velocity = input.max_acceleration = {2.0, 2.0};
    input.max_jerk = {4.0, 4.0};
    Trajectory<2> trajectory;
    REQUIRE(otg.calculate(input, trajectory) == Result::Working);
    CHECK(trajectory.get_duration() == 0.0);
    CHECK(trajectory.get_independent_min_durations() == std::array<double, 2>{0.0, 0.0});
    std::array<double, 2> p, v, a;
    trajectory.at_time(0.0, p, v, a);
    CHECK(p == input.target_position);
    CHECK(v == input.target_velocity);
    CHECK(a == input.target_acceleration);
    OutputParameter<2> output;
    CHECK(otg.update(trajectory, output) == Result::Finished);
    CHECK(output.time == 0.0);
    CHECK(output.new_velocity == input.target_velocity);

    input.minimum_duration = 4.0;
    REQUIRE(otg.calculate(input, trajectory) == Result::Working);
    CHECK(trajectory.get_duration() >= 4.0);
    trajectory.at_time(trajectory.get_duration(), p, v, a);
    for (size_t d = 0; d < 2; ++d) {
        CHECK(p[d] == doctest::Approx(input.target_position[d]));
        CHECK(v[d] == doctest::Approx(input.target_velocity[d]));
        CHECK(a[d] == doctest::Approx(input.target_acceleration[d]));
    }
    input.minimum_duration.reset();
    input.max_position = {0.5, 0.0};
    CHECK(otg.calculate(input, trajectory) == Result::ErrorPositionalLimits);
    CHECK(otg.update(trajectory, output) == Result::ErrorInvalidInput);
}


TEST_CASE("trajectory-sampling-requires-valid-state-and-time") {
    Ruckig<1> otg;
    InputParameter<1> input;
    input.current_position = {0.0};
    input.target_position = {1.0};
    input.max_velocity = input.max_acceleration = input.max_jerk = {1.0};
    Trajectory<1> trajectory;
    double p = 0.0, v = 0.0, a = 0.0;
    CHECK(trajectory.get_duration() == 0.0);
    CHECK_THROWS_AS(trajectory.at_time(0.0, p, v, a), RuckigError);
    CHECK_THROWS_AS(trajectory.get_profiles(), RuckigError);
    CHECK_THROWS_AS(trajectory.get_intermediate_durations(), RuckigError);
    CHECK_THROWS_AS(trajectory.get_independent_min_durations(), RuckigError);
    CHECK_FALSE(trajectory.get_first_time_at_position(0, 0.0).has_value());
    REQUIRE(otg.calculate(input, trajectory) == Result::Working);
    for (const double time: {std::nan(""), std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
        CHECK_THROWS_AS(trajectory.at_time(time, p, v, a), RuckigError);
    }
    trajectory.at_time(-0.1, p, v, a);
    CHECK(std::isfinite(p));
    CHECK(std::isfinite(v));
    CHECK(std::isfinite(a));
    CHECK_FALSE(trajectory.get_first_time_at_position(0, std::nan("")).has_value());
    input.max_position = {0.5};
    CHECK(otg.calculate(input, trajectory) == Result::ErrorPositionalLimits);
    CHECK_THROWS_AS(trajectory.at_time(0.0, p, v, a), RuckigError);
    CHECK_FALSE(trajectory.get_first_time_at_position(0, 0.0).has_value());
}


TEST_CASE("trajectory-storage-dimensions-are-validated") {
    Ruckig<> otg(2, 0.01), other(1, 0.01);
    InputParameter<> input(2), wrong_input(1);
    input.current_position = {0.0, 0.0};
    input.target_position = {1.0, 1.0};
    input.max_velocity = input.max_acceleration = input.max_jerk = {1.0, 1.0};
    wrong_input.current_position = {0.0};
    wrong_input.target_position = {1.0};
    wrong_input.max_velocity = wrong_input.max_acceleration = wrong_input.max_jerk = {1.0};
    OutputParameter<> output(2);
    Trajectory<> wrong(1);
    REQUIRE(otg.update(input, output) == Result::Working);
    output.pass_to_input(input);
    REQUIRE(other.calculate(wrong_input, wrong) == Result::Working);
    output.trajectory = wrong;
    output.new_calculation = true;
    CHECK(otg.update(input, output) == Result::ErrorInvalidInput);
    CHECK_FALSE(output.new_calculation);

    wrong.degrees_of_freedom = 2;
    std::vector<double> p(2), v(2), a(2);
    std::vector<Bound> extrema(2);
    CHECK_THROWS_AS(wrong.at_time(0.0, p, v, a), RuckigError);
    CHECK_THROWS_AS(wrong.get_position_extrema(extrema), RuckigError);
    CHECK_FALSE(wrong.get_first_time_at_position(1, 0.0).has_value());
    CHECK(otg.calculate(input, wrong) == Result::ErrorInvalidInput);

    Ruckig<1> fixed(0.01);
    InputParameter<1> fixed_input;
    fixed_input.current_position = {0.0};
    fixed_input.target_position = {1.0};
    fixed_input.max_velocity = fixed_input.max_acceleration = fixed_input.max_jerk = {1.0};
    OutputParameter<1> fixed_output;
    REQUIRE(fixed.update(fixed_input, fixed_output) == Result::Working);
    fixed_output.pass_to_input(fixed_input);
    fixed_output.trajectory.degrees_of_freedom = 2;
    CHECK(fixed.update(fixed_input, fixed_output) == Result::ErrorInvalidInput);
    double sp = 0.0, sv = 0.0, sa = 0.0;
    CHECK_THROWS_AS(fixed_output.trajectory.at_time(0.0, sp, sv, sa), RuckigError);
}


TEST_CASE("waypoint-filter-respects-stationary-axes-and-shapes") {
    Ruckig<> otg(2);
    InputParameter<> input(2);
    input.current_position = {0.0, 0.0};
    input.target_position = {1.0, 0.0};
    input.intermediate_positions = {{0.5, 10.0}};
    CHECK(otg.filter_intermediate_positions(input, {0.1, 0.1}) == input.intermediate_positions);
    input.intermediate_positions = {{0.5, 0.05}};
    CHECK(otg.filter_intermediate_positions(input, {0.1, 0.1}).empty());
    input.target_position = {-1.0, 0.0};
    input.intermediate_positions = {{-0.5, 0.0}};
    CHECK(otg.filter_intermediate_positions(input, {0.0, 0.0}).empty());
    input.intermediate_positions = {{0.5, 0.0}};
    CHECK(otg.filter_intermediate_positions(input, {0.1, 0.1}) == input.intermediate_positions);
    input.target_position = {0.0, 0.0};
    input.intermediate_positions = {{10.0, 0.0}};
    CHECK(otg.filter_intermediate_positions(input, {0.1, 0.1}) == input.intermediate_positions);
    CHECK_THROWS_AS(otg.filter_intermediate_positions(input, {}), RuckigError);
    CHECK_THROWS_AS(otg.filter_intermediate_positions(input, {-0.1, 0.1}), RuckigError);
    CHECK_THROWS_AS(otg.filter_intermediate_positions(input, {std::nan(""), 0.1}), RuckigError);
    input.intermediate_positions[0] = {1.0};
    CHECK_THROWS_AS(otg.filter_intermediate_positions(input, {0.1, 0.1}), RuckigError);
    input.intermediate_positions = {{std::nan(""), 0.0}};
    CHECK_THROWS_AS(otg.filter_intermediate_positions(input, {0.1, 0.1}), RuckigError);
}


TEST_CASE("constant-jerk-root-regressions") {
    const double cases[][10] = {
        {0.0, -2.0850355161874063e-13, 118.616146584562, 187.46063869767866, 294.78329096470617, 281.93180232186,
            3455.0549787534787, 563.86360464372001, 110.95566291393718, 1.4719001396438314},
        {3.155314916971538, 1.9990117600263974e-11, -113.82348475427857, -316.66694012340406, -369.88090913139985, -258.5351615575571,
            4410.2693402580417, 517.07032311511421, 72.840531573419611, 1.9866916629708615},
        {927128.67972676596, 6.7713971391177015e-08, -0.00046409091929022723, 927128.67971456447, 6.7713971391177015e-08, 0.00046409091929022723,
            0.00046118371669927728, 0.00092818183858045446, 0.0023343967388862997, 0.39761100721177067},
        {0.013967472830951747, -2.4918711163498024e-13, -5.5240543959232176e-15, 0.013967472830951257, -2.4918711163498024e-13, 5.5240543959232176e-15,
            4.9840130271738365e-13, std::numeric_limits<double>::infinity(), 5.6343795521780221e-12, 0.0019608385785042978},
        {0.0, 7.6011034658127291e-13 / 16, 1435411.6539722653 / 16, 2280801.3169312915 / 16, 3328582.401853486 / 16, 2880202.5819926639 / 16,
            42085255.471448816 / 16, std::numeric_limits<double>::infinity(), 936608.97406142496 / 16, 1.5425764305410616},
        {0.0, 0.0, 38.15356476767716, 6.759093744921358e-11, 7.181692293159091e-5, 38.153564767775883,
            111019495.0686374, 76.30712953555177, 5.244824806997274e-5, 1.8822899e-6},
        {0.0, 0.0, 0.0032415181449892984, 2.9516579557608209e-10, 1.3833194707271217e-6, 0.0032415237193457176,
            3.2176418224019225, 0.0064830474386914352, 1.3062342718971396e-5, 0.00042675012738607781},
    };
    Ruckig<1> otg;
    Trajectory<1> trajectory;
    for (const auto& values: cases) {
        InputParameter<1> input;
        input.current_position = {values[0]};
        input.current_velocity = {values[1]};
        input.current_acceleration = {values[2]};
        input.target_position = {values[3]};
        input.target_velocity = {values[4]};
        input.target_acceleration = {values[5]};
        input.max_velocity = {values[6]};
        input.max_acceleration = {values[7]};
        input.max_jerk = {values[8]};
        CAPTURE(input);
        REQUIRE(otg.calculate(input, trajectory) == Result::Working);
        // Small translated displacements lose some precision in their input coordinates.
        CHECK(trajectory.get_duration() <= values[9] + 1e-5 * std::max(1.0, values[9]));
    }
}


TEST_CASE("constant-jerk-feasibility-corpus") {
    std::mt19937_64 rng(20261103);
    std::uniform_real_distribution<double> unit(0.0, 1.0), signed_unit(-1.0, 1.0);
    const auto logrange = [&](double lo, double hi) { return std::pow(10.0, lo + (hi - lo) * unit(rng)); };
    Ruckig<1> otg;
    Trajectory<1> trajectory;
    for (size_t k = 0; k < 5000; ++k) {
        const double time = logrange(-2.0, 1.0), max_jerk = logrange(-1.0, 2.0);
        const double jerk = signed_unit(rng) > 0.0 ? max_jerk : -max_jerk;
        const double position = k % 7 == 0 ? signed_unit(rng) * 10.0 : 0.0;
        const double velocity = signed_unit(rng) * 3.0;
        double acceleration = signed_unit(rng) * max_jerk * time;
        if (k % 5 == 0) acceleration = -jerk * time / 2.0;
        if (k % 11 == 0) acceleration = 0.0;
        InputParameter<1> input;
        input.current_position = {position};
        input.current_velocity = {velocity};
        input.current_acceleration = {acceleration};
        input.target_acceleration = {acceleration + time * jerk};
        input.target_velocity = {velocity + time * (acceleration + time * jerk / 2.0)};
        input.target_position = {position + time * (velocity + time * (acceleration / 2.0 + time * jerk / 6.0))};
        const double max_acceleration = 2.0 * std::max(std::abs(acceleration), std::abs(input.target_acceleration[0]));
        input.max_jerk = {max_jerk};
        input.max_velocity = {2.0 * std::max({std::abs(velocity), std::abs(input.target_velocity[0]),
            std::abs(velocity - acceleration * acceleration / (2.0 * jerk))}) + max_acceleration * max_acceleration / max_jerk};
        input.max_acceleration = {k % 3 == 0 ? std::numeric_limits<double>::infinity() : max_acceleration};
        CAPTURE(k);
        CAPTURE(input);
        REQUIRE(input.validate<false>(true, true));
        REQUIRE(otg.calculate(input, trajectory) == Result::Working);
        CHECK(std::isfinite(trajectory.get_duration()));
        double p = 0.0, v = 0.0, a = 0.0;
        trajectory.at_time(trajectory.get_duration(), p, v, a);
        CHECK(std::abs(p - input.target_position[0]) < 1e-8);
        CHECK(std::abs(v - input.target_velocity[0]) < 1e-8);
        CHECK(std::abs(a - input.target_acceleration[0]) < 1e-10);
    }
}


TEST_CASE("degenerate-roots-preserve-synchronization-gaps") {
    Ruckig<2> otg;
    InputParameter<2> input;
    const double scale = 1.0 / 128;
    input.current_position = {0.0, 0.0};
    input.current_velocity = {-2.0850355161874063e-13 * scale, 0.0};
    input.current_acceleration = {118.616146584562 * scale, 0.0};
    input.target_position = {187.46063869767866 * scale, 3.0};
    input.target_velocity = {294.78329096470617 * scale, 0.0};
    input.target_acceleration = {281.93180232186 * scale, 0.0};
    input.max_velocity = {3455.0549787534787 * scale, 2.0};
    input.max_acceleration = {563.86360464372001 * scale, 2.0};
    input.max_jerk = {110.95566291393718 * scale, 4.0};
    Trajectory<2> trajectory;
    REQUIRE(otg.calculate(input, trajectory) == Result::Working);
    CHECK(trajectory.get_duration() == doctest::Approx(5.7480639142342937));
    const auto independent = trajectory.get_independent_min_durations();
    CHECK(independent[0] == doctest::Approx(1.4719001396438314));
    CHECK(independent[1] == doctest::Approx(3.0));

    // Crossing zero acceleration does not imply a blocked interval after the minimum.
    input.current_velocity = {0.25, 0.0};
    input.current_acceleration = {-0.5, 0.0};
    input.target_position = {1.0 / 6, 0.421875};
    input.target_velocity = {0.25, 0.0};
    input.target_acceleration = {0.5, 0.0};
    input.max_velocity = {4.0, 2.0};
    input.max_acceleration = {2.0, 2.0};
    input.max_jerk = {1.0, 4.0};
    REQUIRE(otg.calculate(input, trajectory) == Result::Working);
    CHECK(trajectory.get_duration() == doctest::Approx(1.5));
}


int main(int argc, char** argv) {
    doctest::Context context;

    if (argc > 1 && std::isdigit(argv[1][0])) {
        number_trajectories = std::stol(argv[1]);
    }
    if (argc > 2 && std::isdigit(argv[2][0])) {
        seed = std::stoi(argv[2]);
    }

    context.applyCommandLine(argc, argv);

    position_second_random_3 = std::min<size_t>(250000, number_trajectories / 25);
    random_direction_3 = std::min<size_t>(250000, number_trajectories / 50);
    random_discrete_3 = std::min<size_t>(250000, number_trajectories / 10);
    velocity_random_3 = number_trajectories / 10;
    velocity_random_discrete_3 = std::min<size_t>(250000, number_trajectories / 10);
    velocity_second_random_3 = std::min<size_t>(250000, number_trajectories / 25);

    const size_t remainder = number_trajectories - (random_direction_3 + velocity_random_3 + random_discrete_3 + position_second_random_3 + velocity_second_random_3); // 1. Normal, 2. High
    position_random_3 = (size_t)(remainder * 95/100);
    random_3_high = (size_t)(remainder * 5/100);

    std::cout << "<number_trajectories>" << std::endl;
    std::cout << "\tPosition (3 DoF): " << position_random_3 << std::endl;
    std::cout << "\tPosition Discrete (3 DoF): " << random_discrete_3 << std::endl;
    std::cout << "\tPosition High Limits (3 DoF): " << random_3_high << std::endl;
    std::cout << "\tPosition Second Order (3 DoF): " << position_second_random_3 << std::endl;
    std::cout << "\tVelocity (3 DoF): " << velocity_random_3 << std::endl;
    std::cout << "\tVelocity Discrete (3 DoF): " << velocity_random_discrete_3 << std::endl;
    std::cout << "\tVelocity Second Order (3 DoF): " << velocity_second_random_3 << std::endl;
    std::cout << "Total: " << number_trajectories << std::endl;

    return context.run();
}

TEST_CASE("two-jerk-short-profile-duplicates") {
    const double cases[][9] = {
        {0.1453753844397165, 0.19828704469061514, -1.5118282382342487,
            0.1533778742000582, -0.09897445831432958, -0.9779203520526656,
            0.041120978405462194, 0.17459794995085792, 0.21571892835632012},
        {1.2664458475998162, 0.028828272126480148, -1.7681621071382487,
            1.266439524707023, -0.029368984644381438, -1.7866465898232171,
            0.018396810435898914, 0.013775689764656818, 0.032172500200555731},
    };
    Ruckig<1> otg;
    Trajectory<1> trajectory;
    for (const auto& values: cases) {
        InputParameter<1> input;
        input.current_position = {values[0]};
        input.current_velocity = {values[1]};
        input.current_acceleration = {values[2]};
        input.target_position = {values[3]};
        input.target_velocity = {values[4]};
        input.target_acceleration = {values[5]};
        input.max_velocity = input.max_acceleration = {2.0};
        input.max_jerk = {4.0};
        CAPTURE(input);
        REQUIRE(input.validate<false>(true, true));
        const auto [p1, v1, a1] = integrate(values[6], values[0], values[1], values[2], -4.0);
        const auto [p2, v2, a2] = integrate(values[7], p1, v1, a1, 4.0);
        CHECK(p2 == doctest::Approx(values[3]).epsilon(1e-12));
        CHECK(v2 == doctest::Approx(values[4]).epsilon(1e-12));
        CHECK(a2 == doctest::Approx(values[5]).epsilon(1e-12));
        REQUIRE(otg.calculate(input, trajectory) == Result::Working);
        CHECK(trajectory.get_duration() == doctest::Approx(values[8]).epsilon(1e-9));
        double p = 0.0, v = 0.0, a = 0.0;
        for (size_t k = 0; k <= 40; ++k) {
            trajectory.at_time(trajectory.get_duration() * k / 40.0, p, v, a);
            CHECK(std::isfinite(p));
            CHECK(std::abs(v) <= 2.0 + 1e-8);
            CHECK(std::abs(a) <= 2.0 + 1e-8);
        }
        CHECK(std::abs(p - values[3]) < 1e-8);
        CHECK(std::abs(v - values[4]) < 1e-8);
        CHECK(std::abs(a - values[5]) < 1e-10);
    }
}

TEST_CASE("two-jerk-short-profile-duplicates-retain-blocked-gap") {
    Ruckig<1> otg;
    InputParameter<1> input;
    input.current_position = {1.2664458475998162};
    input.current_velocity = {0.028828272126480148};
    input.current_acceleration = {-1.7681621071382487};
    input.target_position = {1.266439524707023};
    input.target_velocity = {-0.029368984644381438};
    input.target_acceleration = {-1.7866465898232171};
    input.max_velocity = input.max_acceleration = {2.0};
    input.max_jerk = {4.0};
    Trajectory<1> trajectory;
    input.minimum_duration = 0.0323;
    REQUIRE(otg.calculate(input, trajectory) == Result::Working);
    CHECK(trajectory.get_duration() == doctest::Approx(0.0323).epsilon(1e-10));

    // At 0.1 s even the largest reachable velocity is too low: this is a
    // genuine blocked gap, not an interval removable with the duplicate root.
    input.minimum_duration = 0.1;
    REQUIRE(otg.calculate(input, trajectory) == Result::Working);
    CHECK(trajectory.get_duration() == doctest::Approx(1.7528470225583301).epsilon(1e-9));
    double p = 0.0, v = 0.0, a = 0.0;
    trajectory.at_time(trajectory.get_duration(), p, v, a);
    CHECK(std::abs(p - input.target_position[0]) < 1e-8);
    CHECK(std::abs(v - input.target_velocity[0]) < 1e-8);
    CHECK(std::abs(a - input.target_acceleration[0]) < 1e-10);
}
