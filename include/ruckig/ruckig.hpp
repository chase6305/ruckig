#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <limits>
#include <math.h>
#include <numeric>
#include <optional>
#include <tuple>

#include <ruckig/calculator.hpp>
#include <ruckig/error.hpp>
#include <ruckig/input_parameter.hpp>
#include <ruckig/output_parameter.hpp>
#include <ruckig/trajectory.hpp>


namespace ruckig {

//! Main interface for the Ruckig algorithm
template<size_t DOFs = 0, template<class, size_t> class CustomVector = StandardVector, bool throw_error = false>
class Ruckig {
    //! Current input, only for comparison for recalculation
    InputParameter<DOFs, CustomVector> current_input;

    //! Flag that indicates if the current_input was properly initialized
    bool current_input_initialized {false};

public:
    //! Calculator for new trajectories
    Calculator<DOFs, CustomVector> calculator;

    //! Max number of intermediate waypoints
    const size_t max_number_of_waypoints;

    //! Degrees of freedom
    const size_t degrees_of_freedom;

    //! Time step between updates (cycle time) in [s]
    double delta_time {0.0};

    template<size_t D = DOFs, typename std::enable_if<(D >= 1), int>::type = 0>
    explicit Ruckig():
        max_number_of_waypoints(0),
        degrees_of_freedom(DOFs),
        delta_time(-1.0)
    {
    }

    template<size_t D = DOFs, typename std::enable_if<(D >= 1), int>::type = 0>
    explicit Ruckig(double delta_time):
        max_number_of_waypoints(0),
        degrees_of_freedom(DOFs),
        delta_time(delta_time)
    {
    }

#if defined WITH_CLOUD_CLIENT
    template<size_t D = DOFs, typename std::enable_if<(D >= 1), int>::type = 0>
    explicit Ruckig(double delta_time, size_t max_number_of_waypoints):
        current_input(InputParameter<DOFs, CustomVector>(max_number_of_waypoints)),
        calculator(Calculator<DOFs, CustomVector>(max_number_of_waypoints)),
        max_number_of_waypoints(max_number_of_waypoints),
        degrees_of_freedom(DOFs),
        delta_time(delta_time)
    {
    }
#endif

    template<size_t D = DOFs, typename std::enable_if<(D == 0), int>::type = 0>
    explicit Ruckig(size_t dofs):
        current_input(InputParameter<DOFs, CustomVector>(dofs)),
        calculator(Calculator<DOFs, CustomVector>(dofs)),
        max_number_of_waypoints(0),
        degrees_of_freedom(dofs),
        delta_time(-1.0)
    {
    }

    template<size_t D = DOFs, typename std::enable_if<(D == 0), int>::type = 0>
    explicit Ruckig(size_t dofs, double delta_time):
        current_input(InputParameter<DOFs, CustomVector>(dofs)),
        calculator(Calculator<DOFs, CustomVector>(dofs)),
        max_number_of_waypoints(0),
        degrees_of_freedom(dofs),
        delta_time(delta_time)
    {
    }

#if defined WITH_CLOUD_CLIENT
    template<size_t D = DOFs, typename std::enable_if<(D == 0), int>::type = 0>
    explicit Ruckig(size_t dofs, double delta_time, size_t max_number_of_waypoints):
        current_input(InputParameter<DOFs, CustomVector>(dofs, max_number_of_waypoints)),
        calculator(Calculator<DOFs, CustomVector>(dofs, max_number_of_waypoints)),
        max_number_of_waypoints(max_number_of_waypoints),
        degrees_of_freedom(dofs),
        delta_time(delta_time)
    {
    }
#endif

    //! Reset the instance (e.g. to force a new calculation in the next update)
    void reset() {
        current_input_initialized = false;
    }

    //! Filter intermediate positions based on a threshold distance for each DoF
    template<class T> using Vector = CustomVector<T, DOFs>;
    std::vector<Vector<double>> filter_intermediate_positions(const InputParameter<DOFs, CustomVector>& input, const Vector<double>& threshold_distance) const {
        if (degrees_of_freedom == 0 || input.degrees_of_freedom != degrees_of_freedom
            || input.current_position.size() != degrees_of_freedom || input.target_position.size() != degrees_of_freedom
            || threshold_distance.size() != degrees_of_freedom) {
            throw RuckigError("mismatch in degrees of freedom (waypoint filtering vector size).");
        }
        for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
            if (!std::isfinite(threshold_distance[dof]) || threshold_distance[dof] < 0.0) {
                throw RuckigError("waypoint filtering requires finite non-negative threshold distances.");
            }
        }
        if (input.intermediate_positions.empty()) {
            return input.intermediate_positions;
        }
        for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
            if (!std::isfinite(input.current_position[dof]) || !std::isfinite(input.target_position[dof])) {
                throw RuckigError("waypoint filtering requires finite positions.");
            }
        }
        for (const auto& position: input.intermediate_positions) {
            if (position.size() != degrees_of_freedom) {
                throw RuckigError("mismatch in degrees of freedom (waypoint filtering vector size).");
            }
            for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
                if (!std::isfinite(position[dof])) {
                    throw RuckigError("waypoint filtering requires finite positions.");
                }
            }
        }

        const size_t n_waypoints = input.intermediate_positions.size();
        std::vector<bool> is_active;
        is_active.resize(n_waypoints);
        for (size_t i = 0; i < n_waypoints; ++i) {
            is_active[i] = true;
        }

        size_t start = 0;
        size_t end = start + 2;
        size_t min_current = 1, max_current = 1;
        for (; end < n_waypoints + 2; ++end) {
            const auto& pos_start = (start == 0) ? input.current_position : input.intermediate_positions[start - 1];
            const auto& pos_end = (end == n_waypoints + 1) ? input.target_position : input.intermediate_positions[end - 1];

            // The 1D tests are monotone in position, so the interior extrema suffice.
            // In multiple dimensions retain the full point-wise intersection checks.
            bool are_all_below {true};
            const size_t current_count = degrees_of_freedom == 1 ? 1 + (min_current != max_current) : end - start - 1;
            for (size_t k = 0; k < current_count; ++k) {
                const size_t current = degrees_of_freedom == 1 ? (k == 0 ? min_current : max_current) : start + 1 + k;
                const auto& pos_current = input.intermediate_positions[current - 1];

                // Is there a point t on the line that holds the threshold?
                double t_start_max = 0.0;
                double t_end_min = 1.0;
                for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
                    const double displacement = pos_end[dof] - pos_start[dof];
                    const double offset = pos_current[dof] - pos_start[dof];
                    if (!std::isfinite(displacement) || !std::isfinite(offset)) {
                        are_all_below = false; // Preserve points when subtraction overflows.
                        break;
                    }
                    if (displacement == 0.0) {
                        if (std::abs(offset) > threshold_distance[dof]) {
                            are_all_below = false;
                            break;
                        }
                        continue;
                    }
                    const double h0 = offset / displacement;
                    const double radius = threshold_distance[dof] / std::abs(displacement);
                    const double t_start = h0 - radius;
                    const double t_end = h0 + radius;
                    if (!std::isfinite(t_start) || !std::isfinite(t_end)) {
                        are_all_below = false;
                        break;
                    }

                    t_start_max = std::max(t_start, t_start_max);
                    t_end_min = std::min(t_end, t_end_min);

                    if (t_start_max > t_end_min) {
                        are_all_below = false;
                        break;
                    }
                }
                if (!are_all_below) {
                    break;
                }
            }

            is_active[end - 2] = !are_all_below;
            if (degrees_of_freedom == 1) {
                if (!are_all_below) {
                    min_current = max_current = end;
                } else {
                    if (pos_end[0] < input.intermediate_positions[min_current - 1][0]) {
                        min_current = end;
                    }
                    if (pos_end[0] > input.intermediate_positions[max_current - 1][0]) {
                        max_current = end;
                    }
                }
            }
            if (!are_all_below) {
                start = end - 1;
            }
        }

        std::vector<Vector<double>> filtered_positions;
        filtered_positions.reserve(n_waypoints);
        for (size_t i = 0; i < n_waypoints; ++i) {
            if (is_active[i]) {
                filtered_positions.push_back(input.intermediate_positions[i]);
            }
        }

        return filtered_positions;
    }

    //! Validate the input as well as the Ruckig instance for trajectory calculation
    template<bool throw_validation_error = true>
    bool validate_input(const InputParameter<DOFs, CustomVector>& input, bool check_current_state_within_limits = false, bool check_target_state_within_limits = true) const {
        if (degrees_of_freedom != input.degrees_of_freedom) {
            if constexpr (throw_validation_error) {
                throw RuckigError("mismatch in degrees of freedom (vector size).");
            }
            return false;
        }
        if (!input.template validate<throw_validation_error>(check_current_state_within_limits, check_target_state_within_limits)) {
            return false;
        }

        if (!input.intermediate_positions.empty() && input.control_interface == ControlInterface::Position) {
            if (input.intermediate_positions.size() > max_number_of_waypoints) {
                if constexpr (throw_validation_error) {
                    throw RuckigError("The number of intermediate positions " + std::to_string(input.intermediate_positions.size()) + " exceeds the maximum number of waypoints " + std::to_string(max_number_of_waypoints) + ".");
                }
                return false;
            }
        }

        if ((!std::isfinite(delta_time) || delta_time <= 0.0) && input.duration_discretization != DurationDiscretization::Continuous) {
            if constexpr (throw_validation_error) {
                throw RuckigError("delta time (control rate) parameter " + std::to_string(delta_time) + " should be larger than zero.");
            }
            return false;
        }

        return true;
    }

    //! Calculate a new trajectory for the given input
    Result calculate(const InputParameter<DOFs, CustomVector>& input, Trajectory<DOFs, CustomVector>& trajectory) {
        bool was_interrupted {false};
        return calculate(input, trajectory, was_interrupted);
    }

    //! Calculate a new trajectory for the given input and check for interruption
    Result calculate(const InputParameter<DOFs, CustomVector>& input, Trajectory<DOFs, CustomVector>& trajectory, bool& was_interrupted) {
        current_input_initialized = false;
        trajectory.calculated = false;
        trajectory.profiles_calculated = false;
        trajectory.local_profiles.clear();
        trajectory.local_cumulative_times.clear();
        if (degrees_of_freedom != trajectory.degrees_of_freedom || !trajectory.has_valid_profile_structure()) {
            if constexpr (throw_error) {
                throw RuckigError("mismatch in degrees of freedom (vector size).");
            }
            return Result::ErrorInvalidInput;
        }
        if (!validate_input<throw_error>(input, false, true)) {
            return Result::ErrorInvalidInput;
        }

        const auto result = calculator.template calculate<throw_error>(input, trajectory, delta_time, was_interrupted);
        if (result != Result::Working) {
            return result;
        }
        trajectory.profiles_calculated = true;
        if (!trajectory.check_position_limits(input.min_position, input.max_position)) {
            return Result::ErrorPositionalLimits;
        }
        trajectory.calculated = true;
        return result;
    }

    //! Calculate a local chain. Omitted intermediate velocities/accelerations default to zero.
    //! This offline operation allocates memory; it does not optimize the intermediate states.
    Result calculate_waypoints(const InputParameter<DOFs, CustomVector>& input, Trajectory<DOFs, CustomVector>& trajectory,
        const std::vector<Vector<double>>& intermediate_velocities = {}, const std::vector<Vector<double>>& intermediate_accelerations = {}) {
        current_input_initialized = false;
        trajectory.calculated = false;
        trajectory.profiles_calculated = false;
        const auto invalid = [](const char* message) {
            if constexpr (throw_error) {
                throw RuckigError(message);
            }
            return Result::ErrorInvalidInput;
        };
        if (input.degrees_of_freedom != degrees_of_freedom || trajectory.degrees_of_freedom != degrees_of_freedom
            || !trajectory.has_valid_profile_structure()) {
            return invalid("mismatch in degrees of freedom (vector size).");
        }
        if (!input.template validate<throw_error>()) {
            return Result::ErrorInvalidInput;
        }
        const size_t count = input.intermediate_positions.size();
        if ((!intermediate_velocities.empty() && intermediate_velocities.size() != count)
            || (!intermediate_accelerations.empty() && intermediate_accelerations.size() != count)) {
            return invalid("intermediate state counts must match the number of waypoints.");
        }
        if (input.per_section_max_velocity || input.per_section_max_acceleration || input.per_section_max_jerk
            || input.per_section_min_velocity || input.per_section_min_acceleration
            || input.per_section_max_position || input.per_section_min_position || input.per_section_minimum_duration
            || input.interrupt_calculation_duration) {
            return invalid("local waypoint calculation does not support per-section limits or interruption.");
        }
        if (count == 0) {
            return calculate(input, trajectory);
        }
        if (input.control_interface != ControlInterface::Position || input.per_dof_control_interface || input.per_dof_synchronization
            || (input.synchronization != Synchronization::Time && input.synchronization != Synchronization::Phase)) {
            return invalid("local waypoint calculation requires position control and global time or phase synchronization.");
        }
        for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
            if (!input.enabled[dof]) {
                return invalid("all degrees of freedom must be enabled for local waypoint calculation.");
            }
            if (!std::isfinite(input.current_position[dof]) || !std::isfinite(input.target_position[dof])
                || !std::isfinite(input.current_velocity[dof]) || !std::isfinite(input.target_velocity[dof])
                || !std::isfinite(input.current_acceleration[dof]) || !std::isfinite(input.target_acceleration[dof])) {
                return invalid("local waypoint states must be finite.");
            }
        }
        auto section_input = input;
        section_input.intermediate_positions.clear();
        // Validate both arrival and departure feasibility of the specified intermediate states.
        for (size_t i = 0; i < count; ++i) {
            if ((!intermediate_velocities.empty() && intermediate_velocities[i].size() != degrees_of_freedom)
                || (!intermediate_accelerations.empty() && intermediate_accelerations[i].size() != degrees_of_freedom)) {
                return invalid("mismatch in degrees of freedom (intermediate state vector size).");
            }
            for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
                const double p = input.intermediate_positions[i][dof];
                const double v = intermediate_velocities.empty() ? 0.0 : intermediate_velocities[i][dof];
                const double a = intermediate_accelerations.empty() ? 0.0 : intermediate_accelerations[i][dof];
                if (!std::isfinite(p) || !std::isfinite(v) || !std::isfinite(a)) {
                    return invalid("local waypoint states must be finite.");
                }
                section_input.current_position[dof] = section_input.target_position[dof] = p;
                section_input.current_velocity[dof] = section_input.target_velocity[dof] = v;
                section_input.current_acceleration[dof] = section_input.target_acceleration[dof] = a;
            }
            if (!section_input.template validate<throw_error>(true, true)) {
                return Result::ErrorInvalidInput;
            }
        }
        const auto make_trajectory = [this]() {
            if constexpr (DOFs == 0) {
                return Trajectory<DOFs, CustomVector>(degrees_of_freedom);
            } else {
                return Trajectory<DOFs, CustomVector>();
            }
        };
        auto result = make_trajectory();
        auto section = make_trajectory();
        result.local_profiles.reserve(count + 1);
        result.local_cumulative_times.reserve(count + 1);
        for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
            result.independent_min_durations[dof] = 0.0;
        }
        section_input = input;
        section_input.intermediate_positions.clear();
        for (size_t i = 0; i <= count; ++i) {
            for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
                section_input.target_position[dof] = i == count ? input.target_position[dof] : input.intermediate_positions[i][dof];
                section_input.target_velocity[dof] = i == count ? input.target_velocity[dof] : (intermediate_velocities.empty() ? 0.0 : intermediate_velocities[i][dof]);
                section_input.target_acceleration[dof] = i == count ? input.target_acceleration[dof] : (intermediate_accelerations.empty() ? 0.0 : intermediate_accelerations[i][dof]);
            }
            const auto status = calculate(section_input, section);
            if (status != Result::Working) {
                return status;
            }
            result.duration += section.get_duration();
            if (!std::isfinite(result.duration) || result.duration > 7.6e3) {
                return Result::ErrorTrajectoryDuration;
            }
            result.local_profiles.push_back(section.profiles[0]);
            result.local_cumulative_times.push_back(result.duration);
            for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
                result.independent_min_durations[dof] += section.independent_min_durations[dof];
            }
            // Share the specified boundary state without accumulating endpoint roundoff.
            section_input.current_position = section_input.target_position;
            section_input.current_velocity = section_input.target_velocity;
            section_input.current_acceleration = section_input.target_acceleration;
        }
        result.calculated = true;
        trajectory = std::move(result);
        return Result::Working;
    }

    //! Choose intermediate states by bounded local search over feasible segment pairs.
    //! max_iterations bounds sweeps per seed; zero returns the stopping baseline.
    //! Up to two seeds are refined; this offline operation allocates.
    Result optimize_waypoints(const InputParameter<DOFs, CustomVector>& input, Trajectory<DOFs, CustomVector>& trajectory, size_t max_iterations = 8) {
        current_input_initialized = false;
        trajectory.calculated = false;
        trajectory.profiles_calculated = false;
        if (max_iterations > 64) {
            if constexpr (throw_error) {
                throw RuckigError("waypoint optimization supports at most 64 iterations.");
            }
            return Result::ErrorInvalidInput;
        }
        const size_t count = input.intermediate_positions.size();
        if (count == 0) {
            return calculate_waypoints(input, trajectory);
        }
        if (trajectory.degrees_of_freedom != degrees_of_freedom || !trajectory.has_valid_profile_structure()) {
            if constexpr (throw_error) {
                throw RuckigError("mismatch in degrees of freedom (vector size).");
            }
            return Result::ErrorInvalidInput;
        }
        const auto make_trajectory = [this]() {
            if constexpr (DOFs == 0) {
                return Trajectory<DOFs, CustomVector>(degrees_of_freedom);
            } else {
                return Trajectory<DOFs, CustomVector>();
            }
        };
        // Failed trial states are expected during optimization, even for RuckigThrow.
        auto evaluator = [this]() {
            if constexpr (DOFs == 0) {
                return Ruckig<DOFs, CustomVector, false>(degrees_of_freedom, delta_time);
            } else {
                return Ruckig<DOFs, CustomVector, false>(delta_time);
            }
        }();
        auto best = make_trajectory();
        const auto baseline_status = evaluator.calculate_waypoints(input, best);
        if (baseline_status == Result::ErrorInvalidInput) {
            return calculate_waypoints(input, trajectory); // Preserve public validation diagnostics.
        }
        bool found = baseline_status == Result::Working;
        if (max_iterations == 0 || (found && best.duration == 0.0)) {
            if (found) {
                trajectory = std::move(best);
            }
            return baseline_status;
        }
        // Boundary points can reuse the prescribed endpoint state. Consecutive
        // interior coincident points share one optimized state variable.
        if (input.intermediate_positions.front() == input.current_position
            || input.intermediate_positions.back() == input.target_position
            || std::adjacent_find(input.intermediate_positions.begin(), input.intermediate_positions.end()) != input.intermediate_positions.end()) {
            size_t first = 0, last = count;
            while (first < last && input.intermediate_positions[first] == input.current_position) {
                ++first;
            }
            while (last > first && input.intermediate_positions[last - 1] == input.target_position) {
                --last;
            }
            auto compact_input = input;
            compact_input.intermediate_positions.clear();
            std::vector<size_t> compact_indices;
            compact_indices.reserve(last - first);
            for (size_t i = first; i < last; ++i) {
                const auto& point = input.intermediate_positions[i];
                if (compact_input.intermediate_positions.empty() || !(point == compact_input.intermediate_positions.back())) {
                    compact_input.intermediate_positions.push_back(point);
                }
                compact_indices.push_back(compact_input.intermediate_positions.size() - 1);
            }
            auto compact = make_trajectory();
            if (evaluator.optimize_waypoints(compact_input, compact, max_iterations) == Result::Working) {
                const auto times = compact.get_intermediate_durations();
                std::vector<Vector<double>> v(count, input.current_velocity), a(count, input.current_acceleration);
                auto p = input.current_position;
                for (size_t i = first; i < last; ++i) {
                    compact.at_time(times[compact_indices[i - first]], p, v[i], a[i]);
                }
                for (size_t i = last; i < count; ++i) {
                    v[i] = input.target_velocity;
                    a[i] = input.target_acceleration;
                }
                auto expanded = make_trajectory();
                // Revalidate inherited endpoint states before accepting the expanded chain.
                if (evaluator.calculate_waypoints(input, expanded, v, a) == Result::Working
                    && (!found || expanded.duration <= best.duration)) {
                    trajectory = std::move(expanded);
                    return Result::Working;
                }
            }
        }
        auto zero = input.current_position;
        for (size_t d = 0; d < degrees_of_freedom; ++d) {
            zero[d] = 0.0;
        }
        using States = std::vector<Vector<double>>;
        const States zero_states(count, zero);
        States velocities = zero_states, accelerations = zero_states;
        auto candidate = make_trajectory();
        const auto try_seed = [&](const States& v, const States& a) {
            if (v == zero_states && a == zero_states) {
                return;
            }
            if (evaluator.calculate_waypoints(input, candidate, v, a) == Result::Working
                && (!found || candidate.duration < best.duration)) {
                best = candidate;
                velocities = v;
                accelerations = a;
                found = true;
            }
        };
        // An endpoint trajectory may already visit every waypoint in order. Its
        // states avoid slowing down merely because a straight route is sampled densely.
        auto reference_input = input;
        reference_input.intermediate_positions.clear();
        auto reference = make_trajectory();
        double lower_bound = 0.0;
        size_t crossing_dof = 0;
        double largest_span = 0.0;
        for (size_t d = 0; d < degrees_of_freedom; ++d) {
            reference_input.min_position[d] = -std::numeric_limits<double>::infinity();
            reference_input.max_position[d] = std::numeric_limits<double>::infinity();
            double span = std::abs(input.target_position[d] - input.current_position[d]);
            for (const auto& point: input.intermediate_positions) {
                span = std::max(span, std::abs(point[d] - input.current_position[d]));
            }
            if (span > largest_span) {
                crossing_dof = d;
                largest_span = span;
            }
        }
        States v_seed(count, zero), a_seed(count, zero);
        auto p_sample = zero;
        for (const auto synchronization: {Synchronization::Time, Synchronization::Phase}) {
            reference_input.synchronization = synchronization;
            if (evaluator.calculate(reference_input, reference) != Result::Working) {
                continue;
            }
            if (synchronization == Synchronization::Time) {
                lower_bound = reference.duration;
            }
            double time = 0.0;
            bool visits_all = true;
            for (size_t i = 0; i < count; ++i) {
                const auto crossing = reference.get_first_time_at_position(crossing_dof, input.intermediate_positions[i][crossing_dof], time);
                if (!crossing || *crossing > reference.duration) {
                    visits_all = false;
                    break;
                }
                time = *crossing;
                reference.at_time(time, p_sample, v_seed[i], a_seed[i]);
                for (size_t d = 0; d < degrees_of_freedom; ++d) {
                    if (std::abs(p_sample[d] - input.intermediate_positions[i][d]) > 1e-8) {
                        visits_all = false;
                        break;
                    }
                }
                if (!visits_all) {
                    break;
                }
            }
            if (visits_all) {
                // Re-solve every segment and enforce the original position bounds.
                try_seed(v_seed, a_seed);
                if (found && (best.duration == 0.0 || (lower_bound > 0.0 && best.duration <= lower_bound + 1e-9 * std::max(1.0, lower_bound)))) {
                    trajectory = std::move(best);
                    return Result::Working;
                }
            }
        }
        States velocity_scale(count, zero), acceleration_scale(count, zero);
        States v_lower(count, zero), v_upper(count, zero), a_lower(count, zero), a_upper(count, zero);
        States direction(count, zero), outgoing_direction(count, zero);
        for (size_t i = 0; i < count; ++i) {
            for (size_t d = 0; d < degrees_of_freedom; ++d) {
                const double p = input.intermediate_positions[i][d];
                const double left = p - (i == 0 ? input.current_position[d] : input.intermediate_positions[i - 1][d]);
                const double right = (i + 1 == count ? input.target_position[d] : input.intermediate_positions[i + 1][d]) - p;
                if (left == 0.0 || right == 0.0) {
                    continue; // Keep repeated/stationary coordinates stationary.
                }
                const double min_v = input.min_velocity ? (*input.min_velocity)[d] : -input.max_velocity[d];
                const double min_a = input.min_acceleration ? (*input.min_acceleration)[d] : -input.max_acceleration[d];
                a_lower[i][d] = min_a;
                a_upper[i][d] = input.max_acceleration[d];
                outgoing_direction[i][d] = right > 0.0 ? 1.0 : -1.0;
                if (left > 0.0 && right > 0.0) {
                    direction[i][d] = 1.0;
                    v_upper[i][d] = input.max_velocity[d];
                } else if (left < 0.0 && right < 0.0) {
                    direction[i][d] = -1.0;
                    v_lower[i][d] = min_v;
                } else if (left > 0.0) {
                    a_upper[i][d] = 0.0; // A local maximum requires a reversal.
                } else {
                    a_lower[i][d] = 0.0;
                }
                const double distance = std::min(std::abs(left), std::abs(right));
                const double limit_v = direction[i][d] > 0.0 ? input.max_velocity[d] : -min_v;
                const double scale_v = std::min({limit_v, std::sqrt(std::min(input.max_acceleration[d], -min_a) * distance),
                    std::cbrt(input.max_jerk[d] * distance * distance)});
                velocity_scale[i][d] = direction[i][d] != 0.0 && std::isfinite(scale_v) ? scale_v : 0.0;
                const double scale_a = std::min(std::max(input.max_acceleration[d], -min_a),
                    std::cbrt(input.max_jerk[d] * input.max_jerk[d] * distance));
                acceleration_scale[i][d] = std::isfinite(scale_a) ? scale_a : 0.0;
            }
        }
        // Joint seeds avoid plateaus where moving only one synchronized axis cannot help.
        for (double scale: {0.25, 0.5, 1.0}) {
            States v(count, zero), a(count, zero);
            for (size_t i = 0; i < count; ++i) {
                for (size_t d = 0; d < degrees_of_freedom; ++d) {
                    v[i][d] = direction[i][d] * velocity_scale[i][d] * scale;
                }
            }
            try_seed(v, a);
        }
        // Endpoint motion provides another seed, including some zero-jerk constant-state cases.
        for (size_t i = 0; i < count; ++i) {
            const double fraction = static_cast<double>(i + 1) / (count + 1);
            for (size_t d = 0; d < degrees_of_freedom; ++d) {
                v_seed[i][d] = std::clamp(input.current_velocity[d] + fraction * (input.target_velocity[d] - input.current_velocity[d]), v_lower[i][d], v_upper[i][d]);
                a_seed[i][d] = std::clamp(input.current_acceleration[d] + fraction * (input.target_acceleration[d] - input.current_acceleration[d]), a_lower[i][d], a_upper[i][d]);
            }
        }
        try_seed(v_seed, a_seed);
        // A better seed can enter a worse local basin. Preserve the original
        // seed so the extra geometric proposal cannot discard its refinement.
        const bool previous_found = found;
        auto previous_best = make_trajectory();
        States previous_velocities, previous_accelerations;
        if (previous_found) {
            previous_best = best;
            previous_velocities = velocities;
            previous_accelerations = accelerations;
        }
        // Parameterize the whole polygon by arc length, rather than letting dense
        // sampling reduce every waypoint's velocity scale. All resulting states
        // remain proposals: the section solver must validate the complete chain.
        const auto point = [&](size_t i) -> const Vector<double>& {
            return i == 0 ? input.current_position : (i == count + 1 ? input.target_position : input.intermediate_positions[i - 1]);
        };
        std::vector<double> lengths(count + 1), distances(count + 2, 0.0);
        double path_v = std::numeric_limits<double>::infinity();
        double path_a = path_v, path_j = path_v;
        bool finite_path = true;
        for (size_t i = 0; i <= count; ++i) {
            for (size_t d = 0; d < degrees_of_freedom; ++d) {
                lengths[i] = std::hypot(lengths[i], point(i + 1)[d] - point(i)[d]);
            }
            distances[i + 1] = distances[i] + lengths[i];
            finite_path &= std::isfinite(distances[i + 1]);
            if (lengths[i] == 0.0 || !finite_path) {
                continue;
            }
            for (size_t d = 0; d < degrees_of_freedom; ++d) {
                const double tangent = (point(i + 1)[d] - point(i)[d]) / lengths[i];
                if (tangent == 0.0) {
                    continue;
                }
                const double min_v = input.min_velocity ? (*input.min_velocity)[d] : -input.max_velocity[d];
                const double min_a = input.min_acceleration ? (*input.min_acceleration)[d] : -input.max_acceleration[d];
                path_v = std::min(path_v, (tangent > 0.0 ? input.max_velocity[d] : min_v) / tangent);
                path_a = std::min(path_a, 0.5 * std::min(input.max_acceleration[d], -min_a) / std::abs(tangent));
                path_j = std::min(path_j, input.max_jerk[d] / std::abs(tangent));
            }
        }
        States tangents(count, zero), curvatures(count, zero);
        std::vector<size_t> next_segment(count + 2, count + 1);
        for (size_t i = count + 1; i-- > 0;) {
            next_segment[i] = lengths[i] > 0.0 ? i : next_segment[i + 1];
        }
        size_t left = count + 1;
        for (size_t i = 0; finite_path && i < count; ++i) {
            if (lengths[i] > 0.0) {
                left = i;
            }
            const size_t right = next_segment[i + 1];
            const double h_left = left <= count ? lengths[left] : 0.0;
            const double h_right = right <= count ? lengths[right] : 0.0;
            if (h_left + h_right == 0.0) {
                continue;
            }
            for (size_t d = 0; d < degrees_of_freedom; ++d) {
                const double u_left = h_left > 0.0 ? (point(left + 1)[d] - point(left)[d]) / h_left : 0.0;
                const double u_right = h_right > 0.0 ? (point(right + 1)[d] - point(right)[d]) / h_right : 0.0;
                tangents[i][d] = h_left == 0.0 ? u_right : (h_right == 0.0 ? u_left : (h_right * u_left + h_left * u_right) / (h_left + h_right));
                const double curvature = h_left > 0.0 && h_right > 0.0 ? 2 * (u_right - u_left) / (h_left + h_right) : 0.0;
                curvatures[i][d] = curvature;
                finite_path &= std::isfinite(tangents[i][d]) && std::isfinite(curvature);
                const double min_a = input.min_acceleration ? (*input.min_acceleration)[d] : -input.max_acceleration[d];
                if (curvature != 0.0) {
                    path_v = std::min(path_v, std::sqrt(0.5 * (curvature > 0.0 ? input.max_acceleration[d] : -min_a) / std::abs(curvature)));
                }
            }
        }
        if (finite_path && distances.back() > 0.0 && std::isfinite(path_v) && std::isfinite(path_a) && std::isfinite(path_j)
            && path_v > 0.0 && path_a > 0.0 && path_j > 0.0) {
            Ruckig<1> path_otg;
            InputParameter<1> path_input;
            Trajectory<1> path;
            path_input.current_position = {0.0};
            path_input.target_position = {distances.back()};
            for (double scale: {0.25, 0.5, 0.75}) {
                path_input.max_velocity = {scale * path_v};
                path_input.max_acceleration = {scale * path_a};
                path_input.max_jerk = {scale * path_j};
                if (path_otg.calculate(path_input, path) != Result::Working) {
                    continue;
                }
                bool valid = true;
                for (size_t i = 0; valid && i < count; ++i) {
                    const auto crossing = path.get_first_time_at_position(0, distances[i + 1]);
                    if (!crossing) {
                        valid = false;
                        break;
                    }
                    double p = 0.0, v = 0.0, a = 0.0;
                    path.at_time(*crossing, p, v, a);
                    for (size_t d = 0; d < degrees_of_freedom; ++d) {
                        v_seed[i][d] = tangents[i][d] * v;
                        a_seed[i][d] = tangents[i][d] * a + curvatures[i][d] * v * v;
                        valid &= std::isfinite(v_seed[i][d]) && std::isfinite(a_seed[i][d]);
                    }
                }
                if (valid) {
                    try_seed(v_seed, a_seed);
                }
            }
        }
        const bool new_seed = found && (!previous_found || best.duration < previous_best.duration);
        if (!found) {
            return baseline_status;
        }
        const auto refine = [&](const Trajectory<DOFs, CustomVector>& seed, const States& seed_v, const States& seed_a) {
            velocities = seed_v;
            accelerations = seed_a;
            auto section_input = input;
            section_input.intermediate_positions.clear();
            const auto solve_section = [&](size_t i, Trajectory<DOFs, CustomVector>& section) {
                for (size_t d = 0; d < degrees_of_freedom; ++d) {
                    section_input.current_position[d] = i == 0 ? input.current_position[d] : input.intermediate_positions[i - 1][d];
                    section_input.current_velocity[d] = i == 0 ? input.current_velocity[d] : velocities[i - 1][d];
                    section_input.current_acceleration[d] = i == 0 ? input.current_acceleration[d] : accelerations[i - 1][d];
                    section_input.target_position[d] = i == count ? input.target_position[d] : input.intermediate_positions[i][d];
                    section_input.target_velocity[d] = i == count ? input.target_velocity[d] : velocities[i][d];
                    section_input.target_acceleration[d] = i == count ? input.target_acceleration[d] : accelerations[i][d];
                }
                return evaluator.calculate(section_input, section);
            };
            std::vector<Trajectory<DOFs, CustomVector>> sections;
            sections.reserve(count + 1);
            double total = 0.0;
            for (size_t i = 0; i <= count; ++i) {
                auto section = make_trajectory();
                if (solve_section(i, section) != Result::Working) {
                    return seed;
                }
                total += section.duration;
                sections.push_back(std::move(section));
            }
            auto left_trial = make_trajectory(), right_trial = make_trajectory();
            auto state_input = input;
            state_input.intermediate_positions.clear();
            auto old_v = zero, old_a = zero;
            const auto try_state = [&](size_t i, const Vector<double>& v, const Vector<double>& a) {
                if (v == velocities[i] && a == accelerations[i]) {
                    return;
                }
                old_v = velocities[i];
                old_a = accelerations[i];
                velocities[i] = v;
                accelerations[i] = a;
                for (size_t d = 0; d < degrees_of_freedom; ++d) {
                    state_input.current_position[d] = state_input.target_position[d] = input.intermediate_positions[i][d];
                    state_input.current_velocity[d] = state_input.target_velocity[d] = velocities[i][d];
                    state_input.current_acceleration[d] = state_input.target_acceleration[d] = accelerations[i][d];
                }
                if (state_input.template validate<false>(true, true) && solve_section(i, left_trial) == Result::Working) {
                    // The right section's nonnegative duration cannot lower this bound.
                    const double best_possible = total - sections[i].duration - sections[i + 1].duration + left_trial.duration;
                    if (best_possible < total - 1e-10 * std::max(1.0, total) && solve_section(i + 1, right_trial) == Result::Working) {
                        const double next = total - sections[i].duration - sections[i + 1].duration + left_trial.duration + right_trial.duration;
                        if (std::isfinite(next) && next <= 7.6e3 && next < total - 1e-10 * std::max(1.0, total)) {
                            sections[i] = left_trial;
                            sections[i + 1] = right_trial;
                            total = next;
                            return;
                        }
                    }
                }
                velocities[i] = old_v;
                accelerations[i] = old_a;
            };
            // Neighbors stay fixed during one waypoint visit, and accepted pair cost
            // only decreases. Reuse the four joint trials in the coordinate search.
            std::array<Vector<double>, 4> joint_v, joint_a;
            const auto joint_seen = [&](const Vector<double>& v, const Vector<double>& a) {
                for (size_t trial = 0; trial < 4; ++trial) {
                    if (joint_v[trial] == v && joint_a[trial] == a) {
                        return true;
                    }
                }
                return false;
            };
            auto v = zero, a = zero;
            for (size_t iteration = 0; iteration < max_iterations; ++iteration) {
                const double step = std::ldexp(0.5, -static_cast<int>(iteration));
                for (size_t i = 0; i < count; ++i) {
                    size_t joint_count = 0;
                    for (double sign: {-1.0, 1.0}) {
                        v = velocities[i];
                        a = accelerations[i];
                        for (size_t d = 0; d < degrees_of_freedom; ++d) {
                            v[d] = std::clamp(v[d] + sign * step * direction[i][d] * velocity_scale[i][d], v_lower[i][d], v_upper[i][d]);
                        }
                        joint_v[joint_count] = v;
                        joint_a[joint_count++] = a;
                        try_state(i, v, a);
                        v = velocities[i];
                        a = accelerations[i];
                        for (size_t d = 0; d < degrees_of_freedom; ++d) {
                            a[d] = std::clamp(a[d] + sign * step * outgoing_direction[i][d] * acceleration_scale[i][d], a_lower[i][d], a_upper[i][d]);
                        }
                        joint_v[joint_count] = v;
                        joint_a[joint_count++] = a;
                        try_state(i, v, a);
                    }
                    for (size_t d = 0; d < degrees_of_freedom; ++d) {
                        for (double sign: {-1.0, 1.0}) {
                            v = velocities[i];
                            a = accelerations[i];
                            v[d] = std::clamp(v[d] + sign * step * velocity_scale[i][d], v_lower[i][d], v_upper[i][d]);
                            if (!joint_seen(v, a)) {
                                try_state(i, v, a);
                            }
                            v = velocities[i];
                            a = accelerations[i];
                            a[d] = std::clamp(a[d] + sign * step * acceleration_scale[i][d], a_lower[i][d], a_upper[i][d]);
                            if (!joint_seen(v, a)) {
                                try_state(i, v, a);
                            }
                        }
                    }
                }
            }
            auto result = make_trajectory();
            for (size_t d = 0; d < degrees_of_freedom; ++d) {
                result.independent_min_durations[d] = 0.0;
            }
            for (const auto& section: sections) {
                result.local_profiles.push_back(section.profiles[0]);
                result.duration += section.duration;
                result.local_cumulative_times.push_back(result.duration);
                for (size_t d = 0; d < degrees_of_freedom; ++d) {
                    result.independent_min_durations[d] += section.independent_min_durations[d];
                }
            }
            if (result.duration <= seed.duration && result.check_position_limits(input.min_position, input.max_position)) {
                result.calculated = true;
                return result;
            }
            return seed;
        };
        auto optimized = refine(best, velocities, accelerations);
        if (new_seed && previous_found) {
            auto previous_optimized = refine(previous_best, previous_velocities, previous_accelerations);
            if (previous_optimized.duration < optimized.duration) {
                optimized = std::move(previous_optimized);
            }
        }
        trajectory = std::move(optimized);
        return Result::Working;
    }

    //! Advance a previously calculated trajectory, clamping at its terminal state.
    Result update(const Trajectory<DOFs, CustomVector>& trajectory, OutputParameter<DOFs, CustomVector>& output) {
        const auto start = std::chrono::steady_clock::now();
        output.new_calculation = false;
        if (!trajectory.calculated || !trajectory.has_valid_profile_structure() || degrees_of_freedom != trajectory.degrees_of_freedom
            || degrees_of_freedom != output.degrees_of_freedom
            || output.new_position.size() != degrees_of_freedom || output.new_velocity.size() != degrees_of_freedom
            || output.new_acceleration.size() != degrees_of_freedom || output.new_jerk.size() != degrees_of_freedom
            || !std::isfinite(delta_time) || delta_time <= 0.0
            || !std::isfinite(output.time) || output.time < 0.0) {
            if constexpr (throw_error) {
                throw RuckigError("offline update requires a calculated trajectory, matching vector sizes, a positive finite control cycle, and a non-negative finite time.");
            }
            return Result::ErrorInvalidInput;
        }
        // Preserve the complete trajectory in the output, including when it aliases the argument.
        if (&trajectory != &output.trajectory) {
            output.trajectory = trajectory;
        }
        const size_t old_section = output.new_section;
        output.time = std::min(output.time + delta_time, trajectory.get_duration());
        trajectory.at_time(output.time, output.new_position, output.new_velocity, output.new_acceleration, output.new_jerk, output.new_section);
        output.did_section_change = output.new_section > old_section;
        output.was_calculation_interrupted = false;
        output.calculation_duration = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
        // A later online update must replan from its supplied input.
        current_input_initialized = false;
        return output.time >= trajectory.get_duration() ? Result::Finished : Result::Working;
    }

    //! Get the next output state (with step delta_time) along the calculated trajectory for the given input
    Result update(const InputParameter<DOFs, CustomVector>& input, OutputParameter<DOFs, CustomVector>& output) {
        const auto start = std::chrono::steady_clock::now();
        output.new_calculation = false;
        if (degrees_of_freedom != input.degrees_of_freedom || degrees_of_freedom != output.degrees_of_freedom
            || degrees_of_freedom != output.trajectory.degrees_of_freedom || !output.trajectory.has_valid_profile_structure()) {
            if constexpr (throw_error) {
                throw RuckigError("mismatch in degrees of freedom (vector size).");
            }
            return Result::ErrorInvalidInput;
        }

        if (!std::isfinite(delta_time) || delta_time <= 0.0
            || !std::isfinite(output.time) || output.time < 0.0
            || output.new_position.size() != degrees_of_freedom || output.new_velocity.size() != degrees_of_freedom
            || output.new_acceleration.size() != degrees_of_freedom || output.new_jerk.size() != degrees_of_freedom) {
            if constexpr (throw_error) {
                throw RuckigError("online update requires matching vector sizes, a positive finite control cycle, and a non-negative finite time.");
            }
            return Result::ErrorInvalidInput;
        }

        Result result {Result::Working};
        if (!current_input_initialized || !output.trajectory.calculated || input != current_input) {
            result = calculate(input, output.trajectory, output.was_calculation_interrupted);
            if (result != Result::Working) {
                current_input_initialized = false;
                return result;
            }

            current_input = input;
            current_input_initialized = true;
            output.time = 0.0;
            output.new_calculation = true;
        }

        const size_t old_section = output.new_section;
        output.time = std::min(output.time + delta_time, output.trajectory.get_duration());
        output.trajectory.at_time(output.time, output.new_position, output.new_velocity, output.new_acceleration, output.new_jerk, output.new_section);
        output.did_section_change = (output.new_section > old_section);  // Report only forward section changes

        const auto stop = std::chrono::steady_clock::now();
        output.calculation_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count() / 1000.0;

        output.pass_to_input(current_input);

        if (output.time >= output.trajectory.get_duration()) {
            return Result::Finished;
        }

        return result;
    }
};


template<size_t DOFs, template<class, size_t> class CustomVector = StandardVector>
using RuckigThrow = Ruckig<DOFs, CustomVector, true>;


} // namespace ruckig
