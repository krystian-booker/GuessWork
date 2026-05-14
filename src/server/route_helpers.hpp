#pragma once

#include <string>
#include <utility>

#include <crow.h>

namespace gw::server {

inline crow::response with_no_store(crow::response res) {
    res.add_header("Cache-Control", "no-store");
    return res;
}

inline crow::response json_response(int status, crow::json::wvalue body) {
    crow::response res(status, body);
    return with_no_store(std::move(res));
}

inline crow::response error_response(int status, const std::string& message) {
    crow::json::wvalue body;
    body["error"] = message;
    return json_response(status, std::move(body));
}

}  // namespace gw::server
