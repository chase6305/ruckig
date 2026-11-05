#include <doctest/doctest.h>
#include <httplib/httplib.h>
#include <ruckig/ruckig.hpp>
#include <deque>
#include <string>
namespace {
using namespace ruckig;
template <class T, size_t N>
class IndexOnlyMetadataVector {
    std::deque<T> v;

public:
    IndexOnlyMetadataVector() : v(N) {}
    void resize(size_t n) {
        v.resize(n);
    }
    size_t size() const {
        return v.size();
    }
    T& operator[](size_t i) {
        return v[i];
    }
    const T& operator[](size_t i) const {
        return v[i];
    }
    bool operator==(const IndexOnlyMetadataVector& o) const {
        return v == o.v;
    }
};
template <class Tr>
bool rejects_minima(const Tr& tr, const char* message) {
    try {
        (void)tr.get_independent_min_durations();
    } catch (const RuckigError& e) {
        return std::string(e.what()).find(message) != std::string::npos;
    }
    return false;
}
// Build the response using a real local calculation; only HTTP is replaced.
template <class Input>
void response_for(const Input& input, bool single = false, bool ignore_bounds = false, bool error = false) {
    const size_t n = input.degrees_of_freedom;
    InputParameter<> in(n);
    for (size_t d = 0; d < n; ++d) {
        in.current_position[d] = input.current_position[d];
        in.current_velocity[d] = input.current_velocity[d];
        in.current_acceleration[d] = input.current_acceleration[d];
        in.target_position[d] = input.target_position[d];
        in.target_velocity[d] = input.target_velocity[d];
        in.target_acceleration[d] = input.target_acceleration[d];
        in.max_velocity[d] = input.max_velocity[d];
        in.max_acceleration[d] = input.max_acceleration[d];
        in.max_jerk[d] = input.max_jerk[d];
        in.min_position[d] = ignore_bounds ? -1e6 : input.min_position[d];
        in.max_position[d] = ignore_bounds ? 1e6 : input.max_position[d];
    }
    if (!single)
        for (const auto& point : input.intermediate_positions) {
            std::vector<double> p(n);
            for (size_t d = 0; d < n; ++d)
                p[d] = point[d];
            in.intermediate_positions.push_back(p);
        }
    Ruckig<> local(n, .01);
    Trajectory<> tr(n);
    REQUIRE(local.calculate_waypoints(in, tr) == Result::Working);
    auto profiles = tr.get_profiles();
    for (auto& section : profiles)
        for (auto& p : section) {
            if (p.brake.duration == 0.)
                p.brake = BrakeProfile{};
            if (p.accel.duration == 0.)
                p.accel = BrakeProfile{};
        }
    nlohmann::json data{{"duration", tr.get_duration()},
                        {"cumulative_times", tr.get_intermediate_durations()},
                        {"profiles", profiles},
                        {"message", ""},
                        {"result", error ? "Result.Error" : "Result.Working"}};
    httplib::test_response = std::make_shared<httplib::Response>(httplib::Response{200, data.dump()});
}
template <size_t D, template <class, size_t> class V = StandardVector>
void backend_cases() {
    constexpr size_t n = D == 0 ? 2 : D;
    auto trajectory = []() {
        if constexpr (D == 0)
            return Trajectory<D, V>(n);
        else
            return Trajectory<D, V>();
    };
    auto input = [](double target, bool points) {
        auto x = []() {
            if constexpr (D == 0)
                return InputParameter<D, V>(n);
            else
                return InputParameter<D, V>();
        }();
        for (size_t d = 0; d < n; ++d) {
            const double c = d + 1.;
            x.current_position[d] = 0.;
            x.target_position[d] = target * c;
            x.max_velocity[d] = 2. * c;
            x.max_acceleration[d] = 2. * c;
            x.max_jerk[d] = 4. * c;
            x.min_position[d] = -1e6;
            x.max_position[d] = 1e6;
        }
        if (points)
            for (double f : {1., 2.}) {
                V<double, D> p;
                if constexpr (D == 0)
                    p.resize(n);
                for (size_t d = 0; d < n; ++d)
                    p[d] = f * (d + 1.);
                x.intermediate_positions.push_back(p);
            }
        return x;
    };
    auto otg = []() {
        if constexpr (D == 0)
            return Ruckig<D, V>(n, .01, 8);
        else
            return Ruckig<D, V>(.01, 8);
    }();
    auto tr = trajectory();
    CHECK(rejects_minima(tr, "require a calculated"));
    auto scalar = input(3., false);
    auto local = input(3., true);
    auto cloud = input(3., true);
    REQUIRE(otg.calculate(scalar, tr) == Result::Working);
    auto scalar_mins = tr.get_independent_min_durations();
    for (size_t d = 0; d < n; ++d)
        CHECK(scalar_mins[d] > 0.);
    auto scalar_copy = tr;
    CHECK(scalar_copy.get_independent_min_durations()[0] == scalar_mins[0]);
    response_for(cloud);
    REQUIRE(otg.calculate(cloud, tr) == Result::Working);
    CHECK(rejects_minima(tr, "unavailable for cloud"));
    auto cloud_copy = tr;
    CHECK(rejects_minima(cloud_copy, "unavailable for cloud"));
    auto cloud_move = std::move(cloud_copy);
    CHECK(rejects_minima(cloud_move, "unavailable for cloud"));
    scalar_copy = tr;
    CHECK(rejects_minima(scalar_copy, "unavailable for cloud"));
    scalar_copy = std::move(cloud_move);
    CHECK(rejects_minima(scalar_copy, "unavailable for cloud"));
    REQUIRE(otg.calculate(scalar, tr) == Result::Working);
    for (size_t d = 0; d < n; ++d)
        CHECK(tr.get_independent_min_durations()[d] == scalar_mins[d]);
    REQUIRE(otg.calculate_waypoints(local, tr) == Result::Working);
    auto local_mins = tr.get_independent_min_durations();
    for (size_t d = 0; d < n; ++d)
        CHECK(local_mins[d] > 0.);
    auto local_copy = tr;
    auto local_move = std::move(local_copy);
    CHECK(local_move.get_independent_min_durations()[0] == local_mins[0]);
    response_for(cloud);
    REQUIRE(otg.calculate(cloud, tr) == Result::Working);
    CHECK(rejects_minima(tr, "unavailable for cloud"));
    REQUIRE(otg.calculate_waypoints(local, tr) == Result::Working);
    for (size_t d = 0; d < n; ++d)
        CHECK(tr.get_independent_min_durations()[d] == local_mins[d]);
    REQUIRE(otg.optimize_waypoints(local, tr) == Result::Working);
    CHECK(tr.get_independent_min_durations()[0] > 0.);
    // A cloud response can contain one profile: storage cannot imply support.
    response_for(cloud, true);
    REQUIRE(otg.calculate(cloud, tr) == Result::Working);
    REQUIRE(tr.get_profiles().size() == 1);
    CHECK(rejects_minima(tr, "unavailable for cloud"));
    REQUIRE(otg.calculate_waypoints(scalar, tr) == Result::Working);
    CHECK(tr.get_independent_min_durations()[0] == scalar_mins[0]);
    response_for(cloud, true);
    REQUIRE(otg.calculate(cloud, tr) == Result::Working);
    REQUIRE(otg.optimize_waypoints(scalar, tr) == Result::Working);
    CHECK(tr.get_independent_min_durations()[0] == scalar_mins[0]);
    auto identical = input(0., false);
    REQUIRE(otg.calculate(identical, tr) == Result::Working);
    for (size_t d = 0; d < n; ++d)
        CHECK(tr.get_independent_min_durations()[d] == 0.);
    // Preserve diagnostics after scalar bounds rejection, but reject cloud minima.
    auto limited = input(0., false);
    limited.current_velocity[0] = 1.;
    limited.max_position[0] = .001;
    REQUIRE(otg.calculate(limited, tr) == Result::ErrorPositionalLimits);
    CHECK(tr.get_independent_min_durations()[0] > 0.);
    auto rejected_scalar = tr;
    CHECK(rejected_scalar.get_independent_min_durations()[0] > 0.);
    V<double, D> zero;
    if constexpr (D == 0)
        zero.resize(n);
    for (size_t d = 0; d < n; ++d)
        zero[d] = 0.;
    limited.intermediate_positions.push_back(zero);
    response_for(limited, false, true);
    REQUIRE(otg.calculate(limited, tr) == Result::ErrorPositionalLimits);
    CHECK(rejects_minima(tr, "unavailable for cloud"));
    auto rejected_cloud = tr;
    CHECK(rejects_minima(rejected_cloud, "unavailable for cloud"));
    response_for(cloud, false, false, true);
    REQUIRE(otg.calculate(cloud, tr) == Result::Error);
    CHECK(rejects_minima(tr, "require a calculated"));
    REQUIRE(otg.calculate(scalar, tr) == Result::Working);
    httplib::test_response = std::make_shared<httplib::Response>(httplib::Response{200, "{"});
    // Parsing failure must invalidate metadata, regardless of whether the
    // transport reports it as an error result or a JSON parse exception.
    bool failed = false;
    try {
        failed = otg.calculate(cloud, tr) != Result::Working;
    } catch (const nlohmann::json::parse_error&) {
        failed = true;
    }
    CHECK(failed);
    CHECK(rejects_minima(tr, "require a calculated"));
    REQUIRE(otg.calculate_waypoints(local, tr) == Result::Working);
    CHECK(tr.get_independent_min_durations()[0] == local_mins[0]);
}
} // namespace
TEST_CASE("cloud independent minimum lifecycle fixed and dynamic") {
    backend_cases<1>();
    backend_cases<3>();
    backend_cases<0>();
}
TEST_CASE("cloud independent minimum lifecycle index-only custom vector") {
    backend_cases<1, IndexOnlyMetadataVector>();
    backend_cases<3, IndexOnlyMetadataVector>();
    backend_cases<0, IndexOnlyMetadataVector>();
}
