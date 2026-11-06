#include <doctest/doctest.h>

#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <httplib/httplib.h>
#include <ruckig/ruckig.hpp>

using namespace ruckig;

namespace {
using Json = nlohmann::json;

template<size_t DOFs>
auto schema_input() {
    auto input = [] {
        if constexpr (DOFs == 0) return InputParameter<DOFs>(2);
        else return InputParameter<DOFs>();
    }();
    for (size_t dof = 0; dof < input.degrees_of_freedom; ++dof) {
        const double scale = static_cast<double>(dof + 1);
        input.current_position[dof] = 0.0;
        input.target_position[dof] = 4.0 * scale;
        input.max_velocity[dof] = input.max_acceleration[dof] = input.max_jerk[dof] = 2.0 * scale;
    }
    return input;
}

template<size_t DOFs>
auto schema_otg() {
    if constexpr (DOFs == 0) return Ruckig<DOFs>(2, 0.01, 4);
    else return Ruckig<DOFs>(0.01, 4);
}

template<size_t DOFs, bool Throw>
auto schema_otg_with_errors() {
    if constexpr (DOFs == 0) return Ruckig<DOFs, StandardVector, Throw>(2, 0.01, 4);
    else return Ruckig<DOFs, StandardVector, Throw>(0.01, 4);
}

template<size_t DOFs>
auto schema_output() {
    if constexpr (DOFs == 0) return OutputParameter<DOFs>(2, 4);
    else return OutputParameter<DOFs>(4);
}

template<size_t DOFs>
void add_waypoints(InputParameter<DOFs>& input, const std::vector<double>& positions) {
    for (double position: positions) {
        auto point = input.current_position;
        for (size_t dof = 0; dof < input.degrees_of_freedom; ++dof) {
            point[dof] = position * static_cast<double>(dof + 1);
        }
        input.intermediate_positions.push_back(point);
    }
}

template<size_t DOFs>
Json schema_response(const Trajectory<DOFs>& trajectory) {
    auto profiles = trajectory.get_profiles();
    for (auto& section: profiles) {
        for (auto& profile: section) {
            // Inactive brake storage is not initialized by the local solver.
            if (profile.brake.duration == 0.0) profile.brake = BrakeProfile{};
            if (profile.accel.duration == 0.0) profile.accel = BrakeProfile{};
        }
    }
    return {{"duration", trajectory.get_duration()},
        {"cumulative_times", trajectory.get_intermediate_durations()},
        {"profiles", profiles}, {"message", ""}, {"result", "Result.Working"}};
}

template<size_t DOFs>
Json scalar_schema_response_for(InputParameter<DOFs> input) {
    input.intermediate_positions.clear();
    auto otg = schema_otg<DOFs>();
    auto output = schema_output<DOFs>();
    REQUIRE(otg.calculate(input, output.trajectory) == Result::Working);
    return schema_response(output.trajectory);
}

template<size_t DOFs>
Json scalar_schema_response() {
    return scalar_schema_response_for(schema_input<DOFs>());
}

void serve_schema_response(const std::string& body) {
    httplib::test_response = std::make_shared<httplib::Response>(httplib::Response{200, body});
}

template<size_t DOFs, bool Throw, class Exception = RuckigError>
void check_schema_rejection(const std::string& body) {
    auto input = schema_input<DOFs>();
    auto otg = schema_otg_with_errors<DOFs, Throw>();
    auto output = schema_output<DOFs>();
    add_waypoints(input, {1.0, 2.0, 3.0});
    serve_schema_response(scalar_schema_response_for(input).dump());
    REQUIRE(otg.update(input, output) == Result::Working);
    REQUIRE(output.new_calculation);
    for (size_t dof = 0; dof < input.degrees_of_freedom; ++dof) {
        // A changed target forces a new response after a cached cloud success.
        input.target_position[dof] += static_cast<double>(dof + 1);
        output.new_position[dof] = 123.0;
        output.new_velocity[dof] = 234.0;
        output.new_acceleration[dof] = 345.0;
    }
    serve_schema_response(body);
    const auto posts_before = httplib::test_posts;
    for (size_t attempt = 0; attempt < 2; ++attempt) {
        if constexpr (Throw) {
            CHECK_THROWS_AS(otg.update(input, output), Exception);
        } else {
            Result result = Result::Working;
            CHECK_NOTHROW(result = otg.update(input, output));
            CHECK(result == Result::Error);
        }
        CHECK_FALSE(output.new_calculation);
        CHECK_FALSE(output.was_calculation_interrupted);
        for (size_t dof = 0; dof < input.degrees_of_freedom; ++dof) {
            CHECK(output.new_position[dof] == 123.0);
            CHECK(output.new_velocity[dof] == 234.0);
            CHECK(output.new_acceleration[dof] == 345.0);
        }
        CHECK_THROWS_AS(output.trajectory.get_profiles(), RuckigError);
        CHECK_THROWS_AS(output.trajectory.at_time(0.0, output.new_position), RuckigError);
    }
    CHECK(httplib::test_posts == posts_before + 2);

    // A rejected response must not poison retry or mark the input as cached.
    serve_schema_response(scalar_schema_response_for(input).dump());
    REQUIRE(otg.update(input, output) == Result::Working);
    CHECK(output.new_calculation);
    CHECK(httplib::test_posts == posts_before + 3);
    CHECK(output.trajectory.get_profiles().size() == 1);
    CHECK(output.new_position[0] > 0.0);
}

template<size_t DOFs>
void malformed_schema_cases() {
    using Mutation = std::pair<const char*, std::function<void(Json&)>>;
    const auto response = scalar_schema_response<DOFs>();
    const std::vector<Mutation> mutations {
        {"missing result", [](Json& j) { j.erase("result"); }},
        {"result type", [](Json& j) { j["result"] = 7; }},
        {"unknown result", [](Json& j) { j["result"] = "Result.Unrecognized"; }},
        {"finished result", [](Json& j) { j["result"] = "Result.Finished"; }},
        {"message type", [](Json& j) { j["message"] = Json::object(); }},
        {"missing profiles", [](Json& j) { j.erase("profiles"); }},
        {"profiles type", [](Json& j) { j["profiles"] = Json::object(); }},
        {"empty profiles", [](Json& j) { j["profiles"] = Json::array(); }},
        {"section type", [](Json& j) { j["profiles"][0] = nullptr; }},
        {"too few DOFs", [](Json& j) { j["profiles"][0].erase(j["profiles"][0].end() - 1); }},
        {"too many DOFs", [](Json& j) { j["profiles"][0].push_back(j["profiles"][0][0]); }},
        {"missing duration", [](Json& j) { j.erase("duration"); }},
        {"duration type", [](Json& j) { j["duration"] = "4.0"; }},
        {"duration boolean", [](Json& j) { j["duration"] = false; }},
        {"negative duration", [](Json& j) { j["duration"] = -0.1; }},
        {"nonfinite duration encoded as null", [](Json& j) { j["duration"] = std::numeric_limits<double>::infinity(); }},
        {"missing cumulative times", [](Json& j) { j.erase("cumulative_times"); }},
        {"cumulative times type", [](Json& j) { j["cumulative_times"] = Json::object(); }},
        {"too few cumulative times", [](Json& j) { j["cumulative_times"] = Json::array(); }},
        {"too many cumulative times", [](Json& j) { j["cumulative_times"].push_back(j["duration"]); }},
        {"negative cumulative time", [](Json& j) { j["cumulative_times"][0] = -0.1; }},
        {"cumulative time type", [](Json& j) { j["cumulative_times"][0] = "4.0"; }},
        {"nonfinite cumulative time encoded as null", [](Json& j) { j["cumulative_times"][0] = std::numeric_limits<double>::quiet_NaN(); }},
        {"duration beyond final section", [](Json& j) { j["duration"] = j["duration"].get<double>() + 1.0; }},
        {"duration before final section", [](Json& j) { j["duration"] = j["duration"].get<double>() - 1.0; }},
        {"duration one ULP after final section", [](Json& j) {
            j["duration"] = std::nextafter(j["duration"].get<double>(), std::numeric_limits<double>::infinity());
        }},
        {"duration one ULP before final section", [](Json& j) {
            j["duration"] = std::nextafter(j["duration"].get<double>(), 0.0);
        }},
        {"unsorted cumulative times", [](Json& j) {
            j["profiles"].push_back(j["profiles"][0]);
            j["cumulative_times"] = {j["duration"].get<double>() + 1.0, j["duration"]};
        }},
        {"profile type", [](Json& j) { j["profiles"][0][0] = Json::array(); }},
        {"missing profile field", [](Json& j) { j["profiles"][0][0].erase("pf"); }},
        {"profile field type", [](Json& j) { j["profiles"][0][0]["pf"] = Json::array(); }},
        {"negative segment duration", [](Json& j) { j["profiles"][0][0]["t"][0] = -0.1; }},
        {"negative segment sum", [](Json& j) { j["profiles"][0][0]["t_sum"][0] = -0.1; }},
        {"unsorted segment sums", [](Json& j) { j["profiles"][0][0]["t_sum"][0] = 100.0; }},
        {"nonfinite segment duration encoded as null", [](Json& j) { j["profiles"][0][0]["t"][0] = std::numeric_limits<double>::infinity(); }},
        {"negative brake duration", [](Json& j) { j["profiles"][0][0]["brake"]["duration"] = -0.1; }},
        {"negative active brake segment", [](Json& j) {
            j["profiles"][0][0]["brake"]["duration"] = 1.0;
            j["profiles"][0][0]["brake"]["t"][0] = -0.1;
        }},
        {"negative accel duration", [](Json& j) { j["profiles"][0][0]["accel"]["duration"] = -0.1; }},
    };
    for (const auto& [name, mutate]: mutations) {
        INFO("mutation: " << name);
        auto invalid = response;
        mutate(invalid);
        check_schema_rejection<DOFs, false>(invalid.dump());
        check_schema_rejection<DOFs, true>(invalid.dump());
    }
    for (const Json& value: {Json(), Json::array(), Json(false), Json(1), Json("response")}) {
        INFO("top-level response: " << value.dump());
        check_schema_rejection<DOFs, false>(value.dump());
        check_schema_rejection<DOFs, true>(value.dump());
    }
    for (const auto& field: {"t", "t_sum", "j", "a", "v", "p"}) {
        for (bool extra: {false, true}) {
            INFO("profile array " << field << ", extra: " << extra);
            auto invalid = response;
            auto& array = invalid["profiles"][0][0][field];
            if (extra) array.push_back(0.0);
            else array.erase(array.end() - 1);
            check_schema_rejection<DOFs, false>(invalid.dump());
            check_schema_rejection<DOFs, true>(invalid.dump());
        }
    }
    for (const auto& part: {"brake", "accel"}) {
        for (const auto& field: {"t", "j", "a", "v", "p"}) {
            for (bool extra: {false, true}) {
                INFO("subprofile array " << part << "." << field << ", extra: " << extra);
                auto invalid = response;
                auto& array = invalid["profiles"][0][0][part][field];
                if (extra) array.push_back(0.0);
                else array.erase(array.end() - 1);
                check_schema_rejection<DOFs, false>(invalid.dump());
                check_schema_rejection<DOFs, true>(invalid.dump());
            }
        }
    }

    // JSON has no nonfinite number literal. Overflow is rejected by the real
    // parser, before the schema decoder, retaining its throwing-mode exception.
    auto overflow = response.dump();
    const auto number_start = overflow.find("\"cumulative_times\":[") + std::string("\"cumulative_times\":[").size();
    const auto array_end = overflow.find(']', number_start);
    overflow.replace(number_start, array_end - number_start, "1e309");
    check_schema_rejection<DOFs, false>(overflow);
    check_schema_rejection<DOFs, true, Json::out_of_range>(overflow);
}

template<size_t DOFs>
auto check_schema_matches_local(InputParameter<DOFs> source, const std::vector<double>& requested_waypoints,
    const std::function<void(Json&)>& transform = {}) {
    auto local = schema_otg<DOFs>();
    auto expected = schema_output<DOFs>();
    REQUIRE(local.calculate_waypoints(source, expected.trajectory) == Result::Working);
    auto response = schema_response(expected.trajectory);
    response.erase("message"); // Working messages are optional.
    if (transform) transform(response);
    serve_schema_response(response.dump());

    auto request = source;
    request.intermediate_positions.clear();
    request.minimum_duration.reset();
    add_waypoints(request, requested_waypoints);
    auto cloud = schema_otg<DOFs>();
    auto actual = schema_output<DOFs>();
    REQUIRE(cloud.calculate(request, actual.trajectory) == Result::Working);
    CHECK(actual.trajectory.get_profiles().size() == expected.trajectory.get_profiles().size());
    CHECK(actual.trajectory.get_duration() == expected.trajectory.get_duration());
    const auto params = Json::parse(httplib::test_request_body);
    CHECK(params.at("intermediate_positions").size() == requested_waypoints.size());
    const double duration = expected.trajectory.get_duration();
    for (double time: {0.0, 0.01, duration / 2.0, duration, duration + 1.0}) {
        expected.trajectory.at_time(time, expected.new_position, expected.new_velocity, expected.new_acceleration);
        actual.trajectory.at_time(time, actual.new_position, actual.new_velocity, actual.new_acceleration);
        for (size_t dof = 0; dof < source.degrees_of_freedom; ++dof) {
            CHECK(actual.new_position[dof] == doctest::Approx(expected.new_position[dof]));
            CHECK(actual.new_velocity[dof] == doctest::Approx(expected.new_velocity[dof]));
            CHECK(actual.new_acceleration[dof] == doctest::Approx(expected.new_acceleration[dof]));
        }
    }
    return expected.trajectory;
}

template<size_t DOFs>
void valid_schema_cases() {
    auto input = schema_input<DOFs>();
    auto single = check_schema_matches_local(input, {1.0, 2.0, 3.0}, [](Json& response) {
        // Unused segments do not need to describe a running brake trajectory.
        response["profiles"][0][0]["brake"]["t"][0] = -1.0;
        response["profiles"][0][0]["accel"]["t"][1] = -1.0;
    });
    CHECK(single.get_profiles().size() == 1);
    add_waypoints(input, {2.0});
    auto merged = check_schema_matches_local(input, {1.0, 2.0, 3.0});
    CHECK(merged.get_profiles().size() == 2);

    input = schema_input<DOFs>();
    add_waypoints(input, {0.0, 2.0, 2.0});
    auto repeated = check_schema_matches_local(input, {0.0, 2.0, 2.0});
    const auto times = repeated.get_intermediate_durations();
    REQUIRE(times.size() == 4);
    CHECK(times[0] == 0.0);
    CHECK(times[1] == times[2]);

    input = schema_input<DOFs>();
    for (size_t dof = 0; dof < input.degrees_of_freedom; ++dof) input.target_position[dof] = 0.0;
    auto zero = check_schema_matches_local(input, {0.0, 0.0, 0.0}, [](Json& response) {
        response["duration"] = -0.0;
    });
    CHECK(zero.get_duration() == 0.0);

    input = schema_input<DOFs>();
    input.current_velocity[0] = 3.0;
    auto braked = check_schema_matches_local(input, {1.0, 2.0, 3.0});
    CHECK(braked.get_profiles()[0][0].brake.duration > 0.0);

    input = schema_input<DOFs>();
    input.synchronization = Synchronization::None;
    input.minimum_duration = 15.0;
    auto tail = check_schema_matches_local(input, {1.0, 2.0, 3.0});
    CHECK(tail.get_profiles()[0][0].t_sum.back() < tail.get_duration());

    if constexpr (DOFs == 0 || DOFs > 1) {
        input = schema_input<DOFs>();
        input.enabled[1] = false;
        input.current_velocity[1] = 0.5;
        check_schema_matches_local(input, {1.0, 2.0, 3.0});
    }
}

template<size_t DOFs, bool Throw>
void explicit_error_schema_cases() {
    auto input = schema_input<DOFs>();
    add_waypoints(input, {1.0, 2.0, 3.0});
    auto otg = schema_otg_with_errors<DOFs, Throw>();
    auto output = schema_output<DOFs>();
    for (const auto& [name, result]: {std::pair{"Result.Error", Result::Error},
        std::pair{"Result.ErrorInvalidInput", Result::ErrorInvalidInput}}) {
        for (bool with_message: {false, true}) {
            Json response {{"result", name}};
            if (with_message) response["message"] = "fixture rejected";
            serve_schema_response(response.dump());
            Result actual = Result::Working;
            CHECK_NOTHROW(actual = otg.update(input, output));
            CHECK(actual == result);
            CHECK_FALSE(output.new_calculation);
            CHECK_THROWS_AS(output.trajectory.get_profiles(), RuckigError);
        }
    }
}
} // namespace

TEST_CASE("cloud schema rejects malformed successful responses and retries") {
    malformed_schema_cases<1>();
    malformed_schema_cases<0>();
}

TEST_CASE("cloud schema preserves explicit minimal error responses") {
    explicit_error_schema_cases<1, false>();
    explicit_error_schema_cases<1, true>();
    explicit_error_schema_cases<0, false>();
    explicit_error_schema_cases<0, true>();
}

TEST_CASE("cloud schema preserves filtered sections and local profile boundaries") {
    valid_schema_cases<1>();
    valid_schema_cases<0>();
}
