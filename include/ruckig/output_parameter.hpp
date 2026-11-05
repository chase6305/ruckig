#pragma once

#include <array>
#include <cmath>
#include <iomanip>
#include <type_traits>

#include <ruckig/trajectory.hpp>
#include <ruckig/utils.hpp>


namespace ruckig {

//! Output of the Ruckig algorithm
template<size_t DOFs = 0, template<class, size_t> class CustomVector = StandardVector>
class OutputParameter {
    template<class T> using Vector = CustomVector<T, DOFs>;

    void initialize() {
        for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
            new_position[dof] = 0.0;
            new_velocity[dof] = 0.0;
            new_acceleration[dof] = 0.0;
            new_jerk[dof] = 0.0;
        }
    }

    void resize(size_t dofs) {
        new_position.resize(dofs);
        new_velocity.resize(dofs);
        new_acceleration.resize(dofs);
        new_jerk.resize(dofs);
        initialize();
    }

public:
    size_t degrees_of_freedom;

    //! Current trajectory
    Trajectory<DOFs, CustomVector> trajectory;

    // Current kinematic state
    Vector<double> new_position, new_velocity, new_acceleration, new_jerk;

    //! Current time on trajectory
    double time {0.0};

    //! Index of the current section between two (possibly filtered) intermediate positions (only relevant in Ruckig Pro)
    size_t new_section {0};

    //! Was a new section reached in the last cycle? (only relevant in Ruckig Pro)
    bool did_section_change {false};

    //! Was a new trajectory calculation performed in the last cycle?
    bool new_calculation {false};

    //! Was the trajectory calculation interrupted? (only in Ruckig Pro)
    bool was_calculation_interrupted {false};

    //! Computational duration of the last update call
    double calculation_duration {0.0}; // [µs]

    template<size_t D = DOFs, typename std::enable_if<(D >= 1), int>::type = 0>
    OutputParameter(): degrees_of_freedom(DOFs) { initialize(); }

    template<size_t D = DOFs, typename std::enable_if<(D == 0), int>::type = 0>
    OutputParameter(size_t dofs):
        degrees_of_freedom(dofs),
        trajectory(Trajectory<0, CustomVector>(dofs))
    {
        resize(dofs);
    }

#if defined WITH_CLOUD_CLIENT
    template<size_t D = DOFs, typename std::enable_if<(D >= 1), int>::type = 0>
    OutputParameter(size_t max_number_of_waypoints):
        degrees_of_freedom(DOFs),
        trajectory(Trajectory<DOFs, CustomVector>(max_number_of_waypoints))
    { initialize(); }

    template<size_t D = DOFs, typename std::enable_if<(D == 0), int>::type = 0>
    OutputParameter(size_t dofs, size_t max_number_of_waypoints):
        degrees_of_freedom(dofs),
        trajectory(Trajectory<0, CustomVector>(dofs, max_number_of_waypoints))
    {
        resize(dofs);
    }
#endif

    void pass_to_input(InputParameter<DOFs, CustomVector>& input) const {
        input.current_position = new_position;
        input.current_velocity = new_velocity;
        input.current_acceleration = new_acceleration;

        if (input.intermediate_positions.empty()) {
            return;
        }
        // Keep the flag-only behavior for manually populated or invalid outputs.
        size_t passed = did_section_change ? 1 : 0;
        // Explicit local chains preserve one section per original input interval.
        if (!trajectory.local_profiles.empty() && trajectory.calculated
            && std::isfinite(time) && time >= 0.0 && trajectory.has_valid_profile_structure()
            && degrees_of_freedom == trajectory.degrees_of_freedom) {
            const size_t sections = trajectory.profiles_view().size();
            const auto times = trajectory.times_view();
            const bool current_section = new_section == sections ? time >= trajectory.duration
                : (new_section < sections && time < times[new_section]
                    && (new_section == 0 || time >= times[new_section - 1]));
            const size_t waypoints = sections - 1;
            if (current_section && input.intermediate_positions.size() <= waypoints) {
                const size_t remaining = new_section < waypoints ? waypoints - new_section : 0;
                passed = input.intermediate_positions.size() > remaining
                    ? input.intermediate_positions.size() - remaining : 0;
            }
        }
        input.intermediate_positions.erase(input.intermediate_positions.begin(),
            input.intermediate_positions.begin() + passed);
    }

    std::string to_string() const {
        std::stringstream ss;
        ss << "\nout.new_position = [" << join(new_position, true) << "]\n";
        ss << "out.new_velocity = [" << join(new_velocity, true) << "]\n";
        ss << "out.new_acceleration = [" << join(new_acceleration, true) << "]\n";
        ss << "out.new_jerk = [" << join(new_jerk, true) << "]\n";
        ss << "out.time = [" << std::setprecision(16) << time << "]\n";
        ss << "out.calculation_duration = [" << std::setprecision(16) << calculation_duration << "]\n";
        return ss.str();
    }
};

} // namespace ruckig
