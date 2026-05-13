#include "producer/spinnaker_video_modes.hpp"

#include <SpinnakerPlatform.h>
#undef  SPINNAKER_DEPRECATED_CLASS
#define SPINNAKER_DEPRECATED_CLASS(msg) class SPINNAKER_API
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

namespace gw {

namespace {

// Try to read an integer node, swallowing any GenICam exception.
std::optional<int> try_read_int(Spinnaker::GenApi::INodeMap& nm, const char* name) {
    try {
        Spinnaker::GenApi::CIntegerPtr p = nm.GetNode(name);
        if (Spinnaker::GenApi::IsReadable(p)) {
            return static_cast<int>(p->GetValue());
        }
    } catch (...) {}
    return std::nullopt;
}

// Try to read the max of a float node (e.g. AcquisitionFrameRate's upper bound
// at the current resolution).
std::optional<double> try_read_float_max(Spinnaker::GenApi::INodeMap& nm,
                                         const char*                  name) {
    try {
        Spinnaker::GenApi::CFloatPtr p = nm.GetNode(name);
        if (Spinnaker::GenApi::IsReadable(p)) {
            return p->GetMax();
        }
    } catch (...) {}
    return std::nullopt;
}

// Reads the VideoMode CEnumeration from an initialized camera and populates the
// returned list, including a brief probe of each mode's resolution and max
// frame rate (achieved by switching into the mode and reading Width/Height/
// AcquisitionFrameRate). The original mode is restored at the end. Never
// throws — Spinnaker exceptions are absorbed into {supported=false}. The
// caller must have already called Init() and must not have started
// acquisition.
VideoModeList read_video_modes(Spinnaker::CameraPtr cam) {
    VideoModeList out;
    try {
        Spinnaker::GenApi::INodeMap& dev_nm = cam->GetNodeMap();
        Spinnaker::GenApi::CEnumerationPtr node = dev_nm.GetNode("VideoMode");
        if (!Spinnaker::GenApi::IsReadable(node)) {
            return out;  // supported=false
        }
        out.supported = true;

        // Capture the current selection so we can restore it after probing.
        std::optional<int64_t> original_value;
        try {
            Spinnaker::GenApi::CEnumEntryPtr current = node->GetCurrentEntry();
            if (Spinnaker::GenApi::IsReadable(current)) {
                out.current   = std::string(current->GetSymbolic().c_str());
                original_value = current->GetValue();
            }
        } catch (...) { /* leave current as nullopt */ }

        const bool node_writable = Spinnaker::GenApi::IsWritable(node);

        Spinnaker::GenApi::NodeList_t entries;
        node->GetEntries(entries);
        out.options.reserve(entries.size());
        for (auto* raw : entries) {
            Spinnaker::GenApi::CEnumEntryPtr entry = raw;
            if (!Spinnaker::GenApi::IsAvailable(entry)) continue;
            VideoModeOption opt;
            opt.name = std::string(entry->GetSymbolic().c_str());
            // Mode5 is broken on our hardware — never expose it.
            if (opt.name == "Mode5") continue;
            try {
                opt.display_name = std::string(entry->GetDisplayName().c_str());
            } catch (...) { opt.display_name = opt.name; }
            if (opt.display_name.empty()) opt.display_name = opt.name;
            try {
                opt.description = std::string(entry->GetDescription().c_str());
            } catch (...) { /* leave description empty */ }

            // Probe by switching into this mode. Skip if VideoMode isn't
            // writable (would change nothing, and reads would just report the
            // current mode's geometry for every entry).
            if (node_writable) {
                try {
                    node->SetIntValue(entry->GetValue());
                    opt.width   = try_read_int(dev_nm, "Width");
                    opt.height  = try_read_int(dev_nm, "Height");
                    opt.max_fps = try_read_float_max(dev_nm, "AcquisitionFrameRate");
                } catch (...) { /* leave geometry nullopt */ }
            }

            out.options.push_back(std::move(opt));
        }

        // Restore original mode so we don't leave the camera in a probed-into
        // state. start() will re-set this to the user's chosen mode shortly
        // anyway, but for the standalone variant the camera is about to be
        // DeInited and we should leave its node in the state we found it.
        if (original_value && node_writable) {
            try { node->SetIntValue(*original_value); } catch (...) {}
        }
    } catch (...) {
        // Any Spinnaker/GenICam error → not supported.
        return VideoModeList{};
    }
    return out;
}

}  // namespace

VideoModeList enumerate_video_modes_initialized(Spinnaker::CameraPtr cam) {
    return read_video_modes(cam);
}

VideoModeList enumerate_video_modes_standalone(Spinnaker::CameraPtr cam) {
    cam->Init();  // Spinnaker exceptions propagate (route layer maps to 503).
    VideoModeList out;
    try {
        out = read_video_modes(cam);
    } catch (...) {
        try { cam->DeInit(); } catch (...) {}
        throw;
    }
    try { cam->DeInit(); } catch (...) {}
    return out;
}

}  // namespace gw
