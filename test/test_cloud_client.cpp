#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <httplib/httplib.h>
#include <ruckig/ruckig.hpp>

using namespace ruckig;

namespace {
template<size_t DOFs>
InputParameter<DOFs> make_input() {
    auto input = [] {
        if constexpr (DOFs == 0) return InputParameter<DOFs>(1);
        else return InputParameter<DOFs>();
    }();
    input.current_position = {0.0};
    input.target_position = {4.0};
    input.max_velocity = input.max_acceleration = input.max_jerk = {2.0};
    return input;
}

template<size_t DOFs, bool Throw = false>
auto make_otg() {
    if constexpr (DOFs == 0) return Ruckig<DOFs, StandardVector, Throw>(1, 0.01, 4);
    else return Ruckig<DOFs, StandardVector, Throw>(0.01, 4);
}

template<size_t DOFs>
auto make_output() {
    if constexpr (DOFs == 0) return OutputParameter<DOFs>(1, 4);
    else return OutputParameter<DOFs>(4);
}

template<size_t DOFs>
void check_transport_failure(const std::shared_ptr<httplib::Response>& response) {
    auto input = make_input<DOFs>();
    auto otg = make_otg<DOFs>();
    auto output = make_output<DOFs>();
    REQUIRE(otg.calculate(input, output.trajectory) == Result::Working);
    input.intermediate_positions = {{1.0}, {2.0}, {3.0}};
    output.new_position = {123.0};
    output.new_velocity = {234.0};
    output.new_acceleration = {345.0};
    httplib::test_response = response;
    const auto posts_before = httplib::test_posts;
    for (size_t attempt = 0; attempt < 2; ++attempt) {
        Result result = Result::Working;
        CHECK_NOTHROW(result = otg.update(input, output));
        CHECK(result == Result::Error);
        CHECK_FALSE(output.new_calculation);
        CHECK_FALSE(output.was_calculation_interrupted);
        CHECK(output.new_position[0] == 123.0);
        CHECK(output.new_velocity[0] == 234.0);
        CHECK(output.new_acceleration[0] == 345.0);
        CHECK_THROWS_AS(output.trajectory.get_profiles(), RuckigError);
    }
    CHECK(httplib::test_posts == posts_before + 2);
    const auto params = nlohmann::json::parse(httplib::test_request_body);
    CHECK(params["intermediate_positions"].size() == 3);
}

nlohmann::json make_valid_response() {
    auto input = make_input<1>();
    Ruckig<1> otg;
    Trajectory<1> trajectory;
    REQUIRE(otg.calculate(input, trajectory) == Result::Working);
    auto profiles = trajectory.get_profiles();
    for (auto& section: profiles) {
        for (auto& profile: section) {
            if (profile.brake.duration == 0.0) profile.brake = BrakeProfile{};
            if (profile.accel.duration == 0.0) profile.accel = BrakeProfile{};
        }
    }
    return {{"duration", trajectory.get_duration()}, {"cumulative_times", trajectory.get_intermediate_durations()},
        {"profiles", profiles}, {"message", ""}, {"result", "Result.Working"}};
}
} // namespace

TEST_CASE("cloud-transport-failures-return-result") {
    std::shared_ptr<httplib::Response> response;
    SUBCASE("network unavailable") { }
    SUBCASE("http 400") { response = std::make_shared<httplib::Response>(httplib::Response{400, "request rejected"}); }
    SUBCASE("http 503") { response = std::make_shared<httplib::Response>(httplib::Response{503, "unavailable"}); }
    SUBCASE("http 200 malformed JSON") { response = std::make_shared<httplib::Response>(httplib::Response{200, "{"}); }
    check_transport_failure<1>(response);
    check_transport_failure<0>(response);
}

TEST_CASE("cloud-transport-failures-preserve-throwing-mode") {
    auto input = make_input<1>();
    input.intermediate_positions = {{1.0}, {2.0}, {3.0}};
    auto otg = make_otg<1, true>();
    auto output = make_output<1>();
    httplib::test_response.reset();
    SUBCASE("network unavailable") { }
    SUBCASE("http 400") { httplib::test_response = std::make_shared<httplib::Response>(httplib::Response{400, "rejected"}); }
    CHECK_THROWS_AS(otg.update(input, output), RuckigError);
}


TEST_CASE("cloud-malformed-json-preserves-throwing-mode") {
    auto input = make_input<1>();
    input.intermediate_positions = {{1.0}, {2.0}, {3.0}};
    auto otg = make_otg<1, true>();
    auto output = make_output<1>();
    httplib::test_response = std::make_shared<httplib::Response>(httplib::Response{200, "{"});
    CHECK_THROWS_AS(otg.update(input, output), nlohmann::json::parse_error);
}

TEST_CASE("cloud-valid-and-error-responses-remain-unchanged") {
    auto input = make_input<1>();
    input.intermediate_positions = {{1.0}, {2.0}, {3.0}};
    auto otg = make_otg<1>();
    auto output = make_output<1>();
    auto response = make_valid_response();
    SUBCASE("valid single-profile response") {
        httplib::test_response = std::make_shared<httplib::Response>(httplib::Response{200, response.dump()});
        REQUIRE(otg.update(input, output) == Result::Working);
        CHECK(output.new_calculation);
        CHECK(output.trajectory.get_profiles().size() == 1);
        CHECK(output.new_position[0] > 0.0);
    }
    SUBCASE("explicit error with empty profiles") {
        response["profiles"] = nlohmann::json::array();
        response["cumulative_times"] = nlohmann::json::array();
        response["duration"] = 0.0;
        response["result"] = "Result.ErrorInvalidInput";
        httplib::test_response = std::make_shared<httplib::Response>(httplib::Response{200, response.dump()});
        CHECK(otg.update(input, output) == Result::ErrorInvalidInput);
    }
}
