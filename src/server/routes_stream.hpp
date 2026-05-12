#pragma once

#include <crow.h>

namespace gw::server {

class StreamConsumer;

void register_stream_routes(crow::SimpleApp& app, StreamConsumer& stream);

}  // namespace gw::server
