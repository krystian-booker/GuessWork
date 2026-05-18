#pragma once

#include <optional>
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

template <typename T>
inline void put_opt(crow::json::wvalue& j, const char* key, const std::optional<T>& v) {
    if (v) j[key] = *v;
    else   j[key] = nullptr;
}

}  // namespace gw::server
