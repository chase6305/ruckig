#pragma once

#include <cstddef>
#include <memory>
#include <string>

// Used only by test-cloud-client: production CloudClient::post and its JSON parser
// run unchanged; this adapter supplies an HTTP result without network access.
namespace httplib {
struct Response {
    int status;
    std::string body;
};

inline std::shared_ptr<Response> test_response;
inline std::size_t test_posts {0};
inline std::string test_request_body;

class Client {
public:
    explicit Client(const std::string&) { }

    std::shared_ptr<Response> Post(const std::string&, const std::string& body, const std::string&) {
        ++test_posts;
        test_request_body = body;
        return test_response;
    }
};
} // namespace httplib
