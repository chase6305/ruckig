#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <ruckig/ruckig.hpp>

using namespace ruckig;

namespace {
template<class Function>
double median_per_item(Function&& function, size_t count) {
    std::vector<double> measurements;
    for (size_t repetition = 0; repetition < 12; ++repetition) {
        const auto start = std::chrono::steady_clock::now();
        function();
        const auto end = std::chrono::steady_clock::now();
        if (repetition > 0) measurements.push_back(std::chrono::duration<double, std::nano>(end - start).count() / count);
    }
    std::sort(measurements.begin(), measurements.end());
    return measurements[measurements.size() / 2];
}

uint64_t fingerprint(const auto& rows) {
    uint64_t hash = 14695981039346656037ULL;
    for (const auto& row: rows) {
        for (double value: row) {
            hash ^= std::bit_cast<uint64_t>(value);
            hash *= 1099511628211ULL;
        }
    }
    return hash;
}

Trajectory<> constant_speed_chain(size_t dofs, size_t sections) {
    InputParameter<> input(dofs);
    std::vector<double> velocity(dofs), acceleration(dofs, 0.0);
    for (size_t d = 0; d < dofs; ++d) {
        const double scale = 1.0 + 0.125 * d;
        input.current_position[d] = 0.0;
        input.current_velocity[d] = input.target_velocity[d] = velocity[d] = scale;
        input.target_position[d] = sections * 0.25 * scale;
        input.max_velocity[d] = scale;
        input.max_acceleration[d] = 2.0 * scale;
        input.max_jerk[d] = 4.0 * scale;
    }
    for (size_t i = 1; i < sections; ++i) {
        auto point = velocity;
        for (double& coordinate: point) coordinate *= 0.25 * i;
        input.intermediate_positions.push_back(point);
    }
    Ruckig<> solver(dofs);
    Trajectory<> trajectory(dofs);
    if (solver.calculate_waypoints(input, trajectory, std::vector(sections - 1, velocity), std::vector(sections - 1, acceleration)) != Result::Working) {
        throw std::runtime_error("constant-speed waypoint calculation failed");
    }
    if (trajectory.get_duration() != 0.25 * sections) throw std::runtime_error("constant-speed chain has an unexpected duration");
    return trajectory;
}

void sampling(const Trajectory<>& trajectory, size_t sections) {
    constexpr size_t count = 16384;
    const size_t dofs = trajectory.degrees_of_freedom;
    std::vector<double> times(count);
    for (size_t i = 0; i < count; ++i) times[i] = trajectory.get_duration() * i / (count - 1);
    std::vector<std::vector<double>> p(count, std::vector<double>(dofs)), v = p, a = p;
    std::mt19937 generator(42);
    for (const std::string order: {"sorted", "reversed", "random"}) {
        if (order == "reversed") std::reverse(times.begin(), times.end());
        if (order == "random") std::shuffle(times.begin(), times.end(), generator);
        const double timing = median_per_item([&] { trajectory.at_times(times, p, v, a); }, count);
        // Construction, allocation and comparisons are outside the measured loop.
        std::vector<double> ep(dofs), ev(dofs), ea(dofs);
        for (size_t i = 0; i < count; ++i) {
            trajectory.at_time(times[i], ep, ev, ea);
            if (p[i] != ep || v[i] != ev || a[i] != ea) throw std::runtime_error("batch/scalar sampling mismatch");
        }
        const uint64_t hash = fingerprint(p) ^ fingerprint(v) ^ fingerprint(a);
        std::cout << "sampling," << dofs << ',' << sections << ',' << order << ',' << timing << ',' << hash << '\n';
    }
}

void position_queries(const Trajectory<>& trajectory, size_t sections) {
    constexpr size_t count = 128;
    for (const std::string window: {"early", "near-end", "near-end-window"}) {
        std::array<double, count> positions, crossings;
        for (size_t i = 0; i < count; ++i) {
            const double offset = (256.0 + i) / 4096.0;
            positions[i] = window == "early" ? offset : trajectory.get_duration() - offset;
        }
        const double after = window == "near-end-window" ? trajectory.get_duration() - 0.25 : 0.0;
        const double timing = median_per_item([&] {
            for (size_t i = 0; i < count; ++i) {
                const auto crossing = trajectory.get_first_time_at_position(0, positions[i], after);
                if (!crossing) throw std::runtime_error("position query missed a constant-speed crossing");
                crossings[i] = *crossing;
            }
        }, count);
        std::vector<double> p(trajectory.degrees_of_freedom);
        for (size_t i = 0; i < count; ++i) {
            trajectory.at_time(crossings[i], p);
            if (crossings[i] < after || crossings[i] > trajectory.get_duration() || std::abs(p[0] - positions[i]) >= 1e-9) {
                throw std::runtime_error("position query returned outside its window or the wrong position");
            }
        }
        const std::array<std::array<double, count>, 1> rows {crossings};
        std::cout << "queries," << trajectory.degrees_of_freedom << ',' << sections << ',' << window << ',' << timing << ',' << fingerprint(rows) << '\n';
    }
}
} // namespace

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "all";
    if (mode != "all" && mode != "sampling" && mode != "queries") {
        std::cerr << "Usage: benchmark-trajectory [all|sampling|queries]\n";
        return 1;
    }
    std::cout << "workload,dofs,sections,order_or_window,median_ns_per_item,fingerprint\n" << std::fixed << std::setprecision(2);
    for (size_t dofs: {size_t(1), size_t(6)}) {
        for (size_t sections: {size_t(1), size_t(100), size_t(1000), size_t(10000)}) {
            const auto trajectory = constant_speed_chain(dofs, sections);
            if (mode != "queries") sampling(trajectory, sections);
            if (mode != "sampling") position_queries(trajectory, sections);
        }
    }
}
