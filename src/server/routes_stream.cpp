#include "server/routes_stream.hpp"

#include <exception>
#include <memory>
#include <string>

#include "server/stream_consumer.hpp"
#include "server/webrtc_peer.hpp"

namespace gw::server {

void register_stream_routes(crow::SimpleApp& app, StreamConsumer& stream) {
    CROW_ROUTE(app, "/api/stream/offer").methods("POST"_method)
    ([&stream](const crow::request& req) {
        if (req.body.empty()) {
            return crow::response(400, "Empty SDP offer");
        }

        try {
            auto peer = std::make_shared<WebRtcPeer>();
            const std::string answer = peer->create_answer(req.body);
            stream.add_peer(std::move(peer));

            crow::response res(200, answer);
            res.add_header("Content-Type", "application/sdp");
            res.add_header("Cache-Control", "no-store");
            return res;
        } catch (const std::exception& e) {
            return crow::response(500, std::string("Signaling failed: ") + e.what());
        }
    });
}

}  // namespace gw::server
