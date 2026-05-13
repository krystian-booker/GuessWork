# Camera Mode Setting — Design

Date: 2026-05-12
Branch: `project-hail-mary`

## Problem

The camera management UI today supports only naming a camera at registration time (plus inline name edits). The Spinnaker producer hardcodes `PixelFormat=Mono8` and runs whatever the camera's default `VideoMode` is. Users have no way to choose a sensor mode (e.g. Mode0 = full-res, Mode1 = 2×2 binning, Mode5 = faster binned, Mode7 = ROI on Chameleon3-class cameras), and no way to see what modes a given camera offers.

## Goal

Let users pick the camera's sensor `VideoMode` from a dropdown — populated live from the camera and showing each mode's description — both when first registering a camera and when editing an existing camera row. Changing the mode on an online camera applies immediately by restarting that camera's producer.

## Non-goals

- No other camera settings (binning, ROI, exposure, gain, trigger). Only `VideoMode`.
- No per-camera streaming overrides (`StreamParams` remains process-global).
- No migration tooling for existing dev databases — follow the project's existing "bump schema in place, `--reset-db` to start over" convention from `database.cpp`.
- No HTTP-level route tests; existing pattern is repository-direct testing.

## Design decisions

| Question | Decision |
|---|---|
| Which "Mode" | GenICam `VideoMode` enum on the device nodemap |
| Where in UI | Both: existing Add modal AND a new Edit modal (replacing inline name edit) |
| Apply timing | Immediately — supervisor stops + restarts producer on save |
| Offline cameras | Dropdown disabled; saved value shown read-only |
| Mode-list delivery | Lazy per-camera HTTP endpoints; hit hardware only on explicit user action |

## Architecture

### Backend layers

**Database (`src/server/database.cpp`):**
- `cameras` table schema gains a column: `mode TEXT NULL`. Schema string in `kSchemaCameras` is updated in place — existing dev DBs need `--reset-db`.

**Repository (`src/server/camera_repository.{hpp,cpp}`):**
- `struct Camera` grows `std::optional<std::string> mode`.
- `create(name, serial, mode)` — `mode` optional.
- `update(id, name, mode)` — both `optional<string>`; partial update supported. A no-op call (both nullopt) still returns the row.
- All `SELECT`/`INSERT`/`UPDATE` statements updated to include the column.

**Mode enumeration helper (new — `src/producer/spinnaker_video_modes.{hpp,cpp}`):**
- `struct VideoModeOption { std::string name; std::string display_name; std::string description; }`
- `struct VideoModeList { bool supported; std::optional<std::string> current; std::vector<VideoModeOption> options; }`
- Two functions, both operating only on a camera handle (no `SystemPtr` dependency):
  - `VideoModeList enumerate_video_modes_standalone(Spinnaker::CameraPtr cam)` — does `Init` → enumerate → `DeInit`. Used by the supervisor for the available-serial path.
  - `VideoModeList enumerate_video_modes_initialized(Spinnaker::CameraPtr cam)` — assumes `Init` has already been called. Used by `SpinnakerProducer::start()` so we don't double-Init.
- Enumeration reads `VideoMode` as a `CEnumerationPtr`, walks entries collecting `GetSymbolic()`/`GetDisplayName()`/`GetDescription()`, and reads the current symbolic value. Returns `{supported=false, current=nullopt, options=[]}` if the node is not present or not readable.

**Producer (`src/producer/spinnaker_producer.{hpp,cpp}`):**
- Constructor signature becomes `SpinnakerProducer(std::string name, std::string serial, std::optional<std::string> mode)`. Stored on `Impl`.
- `start()` order:
  1. `cam->Init()`
  2. `set_enum_node(tl_stream_nm, "StreamBufferHandlingMode", "NewestOnly")`
  3. **NEW:** If `impl_->mode` has a value, `set_enum_node(dev_nm, "VideoMode", *impl_->mode)`. Done before PixelFormat because mode affects available formats.
  4. `set_enum_node(dev_nm, "PixelFormat", "Mono8")`
  5. (continue as today: read Width/Height, build pool, `BeginAcquisition`)
- Between steps 1 and 2, call `enumerate_video_modes_initialized(cam)` and cache the result on `Impl`.
- New public method `const VideoModeList& cached_video_modes() const`. Returns an empty `VideoModeList{supported=false,...}` if `start()` hasn't run or failed before the cache was filled.

**Supervisor (`src/server/camera_supervisor.{hpp,cpp}`):**
- `CameraSlot` already has `name`, `serial`. Add `std::optional<std::string> mode` mirroring the row.
- `try_start_slot_locked` constructs `SpinnakerProducer` with the slot's mode.
- `on_camera_updated(id)` extended:
  - Re-read the row.
  - If `row.name` differs, update slot name (existing behavior).
  - If `row.mode` differs from `slot.mode`:
    - Update `slot.mode`.
    - If `slot.producer` is running: `stop_slot_locked(slot)`, then re-fetch the matching `CameraPtr` from the Spinnaker system (because `binding` was dropped during stop), then `try_start_slot_locked(slot, cam)`.
- Two new public methods:
  - `std::optional<VideoModeList> list_video_modes_for_id(int64_t id)` — returns the cached list from the running producer; `nullopt` if the camera id has no running producer (offline or unknown). The route layer separately checks `repo.get(id)` to distinguish 404 from 409.
  - `std::optional<VideoModeList> list_video_modes_for_serial(std::string serial)` — walks `system->GetCameras()`, finds the matching serial, calls `enumerate_video_modes_standalone`. `nullopt` if serial isn't currently connected. Spinnaker exceptions during Init propagate to the caller for 503 mapping.

**Routes (`src/server/routes_camera.cpp`):**
- `GET /api/cameras/<id>/modes` →
  - 404 if `repo.get(id)` is nullopt.
  - 409 `{error:"camera is offline"}` if supervisor returns nullopt for online lookup.
  - 200 with `{ supported, current, options: [{name, display_name, description}] }`.
- `GET /api/cameras/available/<serial>/modes` →
  - 404 if supervisor's serial lookup returns nullopt.
  - 503 with Spinnaker message if `Init` throws.
  - 200 with the same shape.
- `POST /api/cameras` body schema: `{ name, serial, mode? }`. The route reads `mode` if present (string, non-empty) and passes it to `repo.create`. Otherwise repo stores NULL and the producer uses the camera's reported current mode at start.
- `PUT /api/cameras/<id>` body schema: `{ name?, mode? }`. Partial update; at least one of the two must be present (400 if both absent). After `repo.update` returns the new row, call `supervisor.on_camera_updated(id)`; only then send response. JSON body includes the new `mode` value.
- `GET /api/cameras` and `GET /api/cameras/<id>` responses gain a `mode` field (nullable string).

Routes file follows the existing pattern: small parser helpers (`parse_create_body`, `parse_update_body`) extended to handle the optional mode. `with_no_store` + `error_response` reused as-is.

### Frontend

**API client (`web/src/api/cameras.ts`):**
- `Camera` gains `mode: string | null`.
- New types:
  ```ts
  interface CameraMode { name: string; display_name: string; description: string }
  interface CameraModesResponse { supported: boolean; current: string | null; options: CameraMode[] }
  ```
- `createCamera(input: { name: string; serial: string; mode?: string }): Promise<Camera>` — signature change.
- `updateCamera(id: number, patch: { name?: string; mode?: string }): Promise<Camera>` — signature change.
- New: `getCameraModes(id: number): Promise<CameraModesResponse>` — throws structured error on 409 with a discriminable type (e.g. `OfflineError`) so the UI can branch without parsing error strings.
- New: `getAvailableCameraModes(serial: string): Promise<CameraModesResponse>`.

**Shared component (new — `web/src/components/ModeSelect.tsx`):**
- Props: `{ value, options, loading, error, disabled, onChange, hint? }`.
- Renders a `<select>` plus description text for the currently-selected option below the select.
- Handles "not supported" state internally: if `options.length === 0 && !loading && !error`, render the hint message instead of the select.

**Cameras page (`web/src/pages/CamerasPage.tsx`):**
- Remove inline name editing UI (`editingId`/`editingName`/`saveEdit`/`cancelEdit`). The "Edit" button now opens a new Edit modal.
- Add a column or row indicator showing the saved `mode` (compact, fixed width).
- **Add modal:** existing fields plus `<ModeSelect>`. When the user selects a serial (or the initial selection auto-fires when the modal opens), call `getAvailableCameraModes(serial)`, populate options, default selection to `response.current`. Switching the serial resets the mode dropdown state and refetches. On submit, include `mode` in the POST body iff `supported` was true and a value is selected. While the modes endpoint is loading or the serial-modes call failed, the Add button is disabled.
- **Edit modal (new):** Pre-filled with `camera.name`, displays read-only `camera.serial`, and `<ModeSelect>`. On open, call `getCameraModes(id)`:
  - 200: populate options, set selection to `camera.mode ?? response.current`. Enable the dropdown.
  - 409 / offline: disable the dropdown, show "Camera must be online to change Mode." The Name field stays editable.
  - 200 with `supported: false`: show "This camera doesn't expose a Mode setting." Hide the dropdown.
- On submit, build a partial body containing only changed fields (`name` if different, `mode` if different) and PUT. If neither changed, close without calling the API.

## Data flow

### Add camera (online)
1. User opens Add modal → `GET /api/cameras/available`.
2. User picks serial → `GET /api/cameras/available/<serial>/modes`. Backend opens that `CameraPtr`, Init → enumerate `VideoMode` → DeInit, returns list + current.
3. Dropdown populates with current as default. User picks a mode (or accepts default).
4. Submit → `POST /api/cameras` with `{name, serial, mode}`.
5. Repo inserts row. Route calls `supervisor.on_camera_added(id)`. Supervisor opens slot, locates `CameraPtr` from the system, calls `try_start_slot_locked` which constructs `SpinnakerProducer` with the saved mode. Producer Init → set VideoMode → set PixelFormat → start streaming.
6. Response includes the new row including `mode`.

### Edit mode (online camera)
1. User clicks Edit → modal opens with `camera.name`, `camera.serial`, `camera.mode`.
2. `GET /api/cameras/<id>/modes` → supervisor returns the producer's cached `VideoModeList`.
3. User changes selection, clicks Save → `PUT /api/cameras/<id>` body `{mode: "Mode1"}`.
4. Route writes mode, calls `supervisor.on_camera_updated(id)`. Supervisor (mutex held) detects mode delta, calls `stop_slot_locked` (worker thread joined, producer Deinit, binding dropped), re-fetches `CameraPtr` from `system->GetCameras()` by serial, calls `try_start_slot_locked` with new mode.
5. Route returns 200 with updated row.
6. Frontend re-fetches cameras list; UI shows online + new mode value.

### Edit mode (offline camera)
1. User clicks Edit → modal opens with `camera.name`, `camera.serial`, `camera.mode` shown read-only.
2. `GET /api/cameras/<id>/modes` returns 409.
3. ModeSelect renders disabled with hint. Name field stays editable. PUT issues only `{name}` on submit.

### Edit on camera with no `VideoMode` node
1. `GET /api/cameras/<id>/modes` returns 200 `{supported: false, current: null, options: []}`.
2. UI hides dropdown, shows "doesn't expose a Mode setting." Name stays editable.

## Error handling

| Condition | HTTP | Response | UI behavior |
|---|---|---|---|
| Unknown camera id (modes) | 404 | `{error: "camera not found"}` | Close modal with toast (existing pattern) |
| Offline camera (modes) | 409 | `{error: "camera is offline"}` | Disabled dropdown + hint |
| Serial not currently connected (available modes) | 404 | `{error: "serial not connected"}` | Disabled dropdown + hint |
| Spinnaker Init failure | 503 | `{error: "<spinnaker message>"}` | Show error inline in modal |
| Camera has no VideoMode node | 200 | `{supported: false, current: null, options: []}` | Hide dropdown, show hint (not an error) |
| Invalid mode value in PUT | not pre-validated | n/a | Producer fails to start on next run; camera shows offline |
| Duplicate name on PUT/POST | 409 | existing `{error: "camera name already exists: ..."}` | Existing handling unchanged |

The route layer pre-validates only that fields are strings of non-zero length when present. Mode-value-against-camera validation is delegated to the producer at `start()` time — we already enumerated the modes when the user picked one, so this is a recovery-from-firmware-change concern, not a typical-path concern.

## Concurrency

All slot transitions (start, stop, restart-on-mode-change, arrival, removal) serialize on `CameraSupervisor::Impl::mu`. The restart path holds the mutex across `Init`/`BeginAcquisition` (~hundreds of ms). This blocks other slot operations and `is_online`/`snapshot_all` calls, but not the frame data path. Acceptable: the only callers blocked are HTTP route handlers and Spinnaker arrival events.

`stop_slot_locked` drops the `binding` (releases `CameraPtr`/`SystemPtr` refcounts) before the restart logic re-fetches a fresh `CameraPtr` from `system->GetCameras()`. This avoids re-using a handle whose state may be partial after `DeInit`.

## Stream interruption

Restart on mode change causes a brief WebRTC stream gap (~0.5–1s on Chameleon3). `StreamConsumer::detach`/`attach` handle the producer disappearing and reappearing; the consumer triggers a keyframe on next attach. Browser video element re-keys without a manual reconnect. No confirmation dialog is added — the user is in a settings modal clicking Save, the context implies intent.

## Testing

**Unit (`tests/test_camera_repository.cpp`):**
- Extend fixture and assertions for `mode` column.
- `create` with and without `mode`.
- `update`: name only, mode only, both, neither (no-op).
- `get`/`list_all` return mode field correctly (nullopt and string cases).
- Duplicate-name behavior unchanged.

**E2E (`web/e2e/cameras-mode.spec.ts` — new):**
- Use Playwright route interception to mock all `/api/cameras*` endpoints.
- Add modal: pick a serial, assert mode dropdown populates, assert description text appears under the selected option, submit, assert POST body has `mode`.
- Edit modal (online): assert dropdown populates with `camera.mode` pre-selected, change selection, submit, assert PUT body has only `mode`.
- Edit modal (offline): mock 409 on modes endpoint, assert dropdown disabled + hint visible, change name only, assert PUT body has only `name`.
- Edit modal (not supported): mock 200 with `supported:false`, assert dropdown hidden and hint visible.

The existing `stream.spec.ts` continues to run against `build-fresh/guesswork` and is not modified.

**Manual smoke checklist:**
1. Fresh DB (or `--reset-db`) → Add modal → connected Chameleon3 → mode dropdown shows entries with descriptions (Mode0, Mode1, etc.) → save → row appears online with selected mode.
2. Online camera → Edit → change mode → save → stream blinks ~1s then resumes → resolution change visible in WebRTC stats / browser DevTools.
3. Unplug camera → Edit → dropdown disabled, hint visible; name still editable; submit name-only PUT succeeds.
4. Online camera with no VideoMode node (if hardware available) → Edit shows "doesn't expose a Mode setting" hint; name still editable.

No automated coverage for the actual Spinnaker enumeration or the supervisor restart path — these are hardware-dependent and `gw_tests` never opens a camera.

## Files

**New:**
- `src/producer/spinnaker_video_modes.hpp`
- `src/producer/spinnaker_video_modes.cpp`
- `web/src/components/ModeSelect.tsx`
- `web/e2e/cameras-mode.spec.ts`

**Modified:**
- `src/server/database.cpp` (schema)
- `src/server/camera_repository.hpp` (Camera struct, signatures)
- `src/server/camera_repository.cpp` (SQL updates)
- `src/server/camera_supervisor.hpp` (new public methods)
- `src/server/camera_supervisor.cpp` (slot mode field, restart-on-update, new list methods)
- `src/server/routes_camera.cpp` (new routes, body parsing, response shape)
- `src/producer/spinnaker_producer.hpp` (constructor signature, new accessor)
- `src/producer/spinnaker_producer.cpp` (mode application order in `start()`, cached modes)
- `web/src/api/cameras.ts` (types, new client calls, signature changes)
- `web/src/pages/CamerasPage.tsx` (Edit modal, Add modal mode integration, remove inline edit)
- `tests/test_camera_repository.cpp` (extended assertions)
- `CMakeLists.txt` for `gw_producer` (new source file)
