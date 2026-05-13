#include "server/routes_stream.hpp"

#include <exception>
#include <memory>
#include <string>

#include "server/camera_supervisor.hpp"
#include "server/stream_consumer.hpp"
#include "server/webrtc_peer.hpp"

namespace gw::server {

void register_stream_routes(crow::SimpleApp& app, CameraSupervisor& supervisor) {
    CROW_ROUTE(app, "/api/cameras/<int>/stream/offer").methods("POST"_method)
    ([&supervisor](const crow::request& req, int64_t camera_id) {
        if (req.body.empty()) {
            return crow::response(400, "Empty SDP offer");
        }

        auto stream = supervisor.stream_consumer_for(camera_id);
        if (!stream) {
            return crow::response(404, "Camera not online");
        }

        try {
            auto peer = std::make_shared<WebRtcPeer>();
            const std::string answer = peer->create_answer(req.body);
            stream->add_peer(std::move(peer));

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
