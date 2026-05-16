import { useEffect, useState } from 'react'
import { Link } from 'react-router-dom'
import {
  CameraOfflineError,
  createCamera,
  deleteCamera,
  getAvailableCameraModes,
  getCameraModes,
  listAvailableCameras,
  listCameras,
  updateCamera,
  type AvailableCamera,
  type Camera,
  type CameraMode,
} from '../api/cameras'

// Sensor-side constants used for the inline preview of HFOV + auto-selected
// Kalibr model. Must stay in sync with the server-side build_suggested_command
// in calibration_supervisor.cpp (the source of truth at calibration time).
const PIXEL_PITCH_MM = 0.00345
const SENSOR_WIDTH_PX = 2048
const FISHEYE_HFOV_THRESHOLD_DEG = 95

function previewLensSpec(focalMm: number) {
  if (!Number.isFinite(focalMm) || focalMm <= 0) return null
  const focalPx = focalMm / PIXEL_PITCH_MM
  const sensorWmm = PIXEL_PITCH_MM * SENSOR_WIDTH_PX
  const hfovDeg = (2 * Math.atan((sensorWmm / 2) / focalMm) * 180) / Math.PI
  const model = hfovDeg >= FISHEYE_HFOV_THRESHOLD_DEG ? 'pinhole-equi' : 'pinhole-radtan'
  return { focalPx, hfovDeg, model }
}

function LensPreview({ focalMm }: { focalMm: number }) {
  const preview = previewLensSpec(focalMm)
  if (!preview) {
    return (
      <small style={{ display: 'block', marginTop: 4, color: '#9ca3af' }}>
        Enter a positive focal length to preview the calibration model.
      </small>
    )
  }
  return (
    <small style={{ display: 'block', marginTop: 4, color: '#4b5563' }}>
      ≈ {preview.hfovDeg.toFixed(0)}° HFOV → <code>{preview.model}</code>
      {' '}(~{preview.focalPx.toFixed(0)} px focal hint)
    </small>
  )
}
import ModeSelect from '../components/ModeSelect'
import { dangerButtonStyle, neutralButtonStyle, primaryButtonStyle } from '../components/buttonStyles'

const inputStyle: React.CSSProperties = {
  padding: '6px 10px',
  fontSize: 14,
  border: '1px solid #ccc',
  borderRadius: 6,
  minWidth: 0,
}

const modalOverlay: React.CSSProperties = {
  position: 'fixed',
  inset: 0,
  background: 'rgba(0,0,0,0.4)',
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'center',
  zIndex: 100,
}

const modalCard: React.CSSProperties = {
  background: '#fff',
  borderRadius: 8,
  padding: 24,
  minWidth: 360,
  maxWidth: 480,
  boxShadow: '0 10px 25px rgba(0,0,0,0.15)',
}

// Renders "[1280x960 @ 30fps]" suffix when the camera reports geometry.
function modeGeometrySuffix(c: Camera): string {
  const parts: string[] = []
  if (c.mode_width != null && c.mode_height != null) {
    parts.push(`${c.mode_width}x${c.mode_height}`)
  }
  if (c.mode_max_fps != null) {
    const fps = c.mode_max_fps >= 1
      ? Math.round(c.mode_max_fps)
      : Math.round(c.mode_max_fps * 10) / 10
    parts.push(`${fps}fps`)
  }
  return parts.length === 0 ? '' : ` [${parts.join(' @ ')}]`
}

function OnlineDot({ online }: { online: boolean }) {
  return (
    <span
      title={online ? 'online' : 'offline'}
      style={{
        display: 'inline-block',
        width: 10,
        height: 10,
        borderRadius: '50%',
        background: online ? '#16a34a' : '#9ca3af',
        marginRight: 8,
      }}
    />
  )
}

// Tri-state load handle for an async modes fetch.
interface ModesState {
  options: CameraMode[]
  current: string | null
  supported: boolean
  loading: boolean
  error: string | null
  offline: boolean
}

const emptyModes: ModesState = {
  options: [],
  current: null,
  supported: true,
  loading: false,
  error: null,
  offline: false,
}

export default function CamerasPage() {
  const [cameras, setCameras] = useState<Camera[] | null>(null)
  const [listError, setListError] = useState<string | null>(null)
  const [busy, setBusy] = useState(false)

  // --- Add modal state ---
  const [addOpen, setAddOpen] = useState(false)
  const [addName, setAddName] = useState('')
  const [addSerial, setAddSerial] = useState('')
  const [available, setAvailable] = useState<AvailableCamera[] | null>(null)
  const [addError, setAddError] = useState<string | null>(null)
  const [addLoading, setAddLoading] = useState(false)
  const [addModes, setAddModes] = useState<ModesState>(emptyModes)
  const [addSelectedMode, setAddSelectedMode] = useState<string | null>(null)
  // String state so the user can type partial decimals like "6." without the
  // controlled <input type=number> snapping the value mid-edit.
  const [addFocalMm, setAddFocalMm] = useState<string>('')

  // --- Edit modal state ---
  const [editCamera, setEditCamera] = useState<Camera | null>(null)
  const [editName, setEditName] = useState('')
  const [editError, setEditError] = useState<string | null>(null)
  const [editModes, setEditModes] = useState<ModesState>(emptyModes)
  const [editSelectedMode, setEditSelectedMode] = useState<string | null>(null)
  const [editFocalMm, setEditFocalMm] = useState<string>('')

  const refresh = async () => {
    try {
      const rows = await listCameras()
      setCameras(rows)
      setListError(null)
    } catch (e) {
      setListError(e instanceof Error ? e.message : String(e))
    }
  }

  useEffect(() => {
    refresh()
  }, [])

  // --- Add modal handlers ---

  const openAddModal = async () => {
    setAddOpen(true)
    setAddName('')
    setAddSerial('')
    setAddError(null)
    setAddLoading(true)
    setAvailable(null)
    setAddModes(emptyModes)
    setAddSelectedMode(null)
    setAddFocalMm('')
    try {
      const list = await listAvailableCameras()
      setAvailable(list)
      if (list.length > 0) setAddSerial(list[0].serial)
    } catch (e) {
      setAddError(e instanceof Error ? e.message : String(e))
      setAvailable([])
    } finally {
      setAddLoading(false)
    }
  }

  // Refetch modes whenever the chosen serial changes.
  useEffect(() => {
    if (!addOpen || !addSerial) return
    let cancelled = false
    setAddModes({ ...emptyModes, loading: true })
    setAddSelectedMode(null)
    getAvailableCameraModes(addSerial)
      .then((r) => {
        if (cancelled) return
        setAddModes({
          options: r.options,
          current: r.current,
          supported: r.supported,
          loading: false,
          error: null,
          offline: false,
        })
        setAddSelectedMode(r.current ?? (r.options[0]?.name ?? null))
      })
      .catch((e) => {
        if (cancelled) return
        setAddModes({
          ...emptyModes,
          error: e instanceof Error ? e.message : String(e),
        })
      })
    return () => {
      cancelled = true
    }
  }, [addOpen, addSerial])

  const closeAddModal = () => {
    if (busy) return
    setAddOpen(false)
  }

  const onSubmitAdd = async (e: React.FormEvent) => {
    e.preventDefault()
    const focalNum = parseFloat(addFocalMm)
    if (!addName.trim() || !addSerial || !Number.isFinite(focalNum) || focalNum <= 0 || busy) return
    setBusy(true)
    setAddError(null)
    try {
      await createCamera({
        name: addName.trim(),
        serial: addSerial,
        focal_length_mm: focalNum,
        mode:
          addModes.supported && addSelectedMode ? addSelectedMode : undefined,
      })
      setAddOpen(false)
      await refresh()
    } catch (err) {
      setAddError(err instanceof Error ? err.message : String(err))
    } finally {
      setBusy(false)
    }
  }

  // --- Edit modal handlers ---

  const openEditModal = async (c: Camera) => {
    setEditCamera(c)
    setEditName(c.name)
    setEditError(null)
    setEditModes({ ...emptyModes, loading: true })
    setEditSelectedMode(c.mode)
    setEditFocalMm(c.focal_length_mm.toString())
    try {
      const r = await getCameraModes(c.id)
      setEditModes({
        options: r.options,
        current: r.current,
        supported: r.supported,
        loading: false,
        error: null,
        offline: false,
      })
      setEditSelectedMode(c.mode ?? r.current ?? (r.options[0]?.name ?? null))
    } catch (err) {
      if (err instanceof CameraOfflineError) {
        setEditModes({ ...emptyModes, offline: true })
      } else {
        setEditModes({
          ...emptyModes,
          error: err instanceof Error ? err.message : String(err),
        })
      }
    }
  }

  const closeEditModal = () => {
    if (busy) return
    setEditCamera(null)
  }

  const onSubmitEdit = async (e: React.FormEvent) => {
    e.preventDefault()
    if (!editCamera || busy) return
    const trimmedName = editName.trim()
    if (!trimmedName) return

    // Build a partial body containing only changed fields.
    const patch: { name?: string; mode?: string; focal_length_mm?: number } = {}
    if (trimmedName !== editCamera.name) patch.name = trimmedName
    if (
      editModes.supported &&
      !editModes.offline &&
      editSelectedMode &&
      editSelectedMode !== editCamera.mode
    ) {
      patch.mode = editSelectedMode
    }
    const editFocalNum = parseFloat(editFocalMm)
    if (Number.isFinite(editFocalNum) && editFocalNum > 0 &&
        editFocalNum !== editCamera.focal_length_mm) {
      patch.focal_length_mm = editFocalNum
    }
    if (!patch.name && !patch.mode && patch.focal_length_mm === undefined) {
      setEditCamera(null)
      return
    }

    setBusy(true)
    setEditError(null)
    try {
      await updateCamera(editCamera.id, patch)
      setEditCamera(null)
      await refresh()
    } catch (err) {
      setEditError(err instanceof Error ? err.message : String(err))
    } finally {
      setBusy(false)
    }
  }

  const onDelete = async (id: number) => {
    if (busy) return
    setBusy(true)
    try {
      await deleteCamera(id)
      await refresh()
    } catch (err) {
      setListError(err instanceof Error ? err.message : String(err))
    } finally {
      setBusy(false)
    }
  }

  return (
    <div>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center',
                     marginBottom: 16 }}>
        <h2 style={{ margin: 0 }}>Cameras</h2>
        <button type="button" style={primaryButtonStyle} onClick={openAddModal} disabled={busy}>
          Add camera
        </button>
      </div>

      {listError && <p style={{ color: 'crimson' }}>{listError}</p>}

      {cameras === null && !listError && <p>Loading…</p>}

      {cameras && cameras.length === 0 && (
        <p style={{ color: '#666' }}>No cameras yet. Click “Add camera” above.</p>
      )}

      {cameras && cameras.length > 0 && (
        <table style={{ width: '100%', borderCollapse: 'collapse' }}>
          <thead>
            <tr style={{ borderBottom: '1px solid #ddd', textAlign: 'left' }}>
              <th style={{ padding: 8, width: 60 }}>ID</th>
              <th style={{ padding: 8, width: 120 }}>Status</th>
              <th style={{ padding: 8 }}>Name</th>
              <th style={{ padding: 8, width: 180 }}>Serial</th>
              <th style={{ padding: 8, width: 100 }}>Lens</th>
              <th style={{ padding: 8, width: 220 }}>Mode</th>
              <th style={{ padding: 8, width: 280 }}>Actions</th>
            </tr>
          </thead>
          <tbody>
            {cameras.map((c) => (
              <tr key={c.id} style={{ borderBottom: '1px solid #eee' }}>
                <td style={{ padding: 8, color: '#666' }}>{c.id}</td>
                <td style={{ padding: 8, color: '#374151' }}>
                  <OnlineDot online={c.online} />
                  {c.online ? 'online' : 'offline'}
                </td>
                <td style={{ padding: 8 }}>{c.name}</td>
                <td style={{ padding: 8, fontFamily: 'monospace', color: '#374151' }}>
                  {c.serial}
                </td>
                <td style={{ padding: 8, color: '#4b5563' }}>
                  {c.focal_length_mm.toFixed(1)} mm
                </td>
                <td style={{ padding: 8, color: '#4b5563' }}>
                  {c.mode ? (
                    <>
                      {c.mode}
                      {modeGeometrySuffix(c) && (
                        <span style={{ color: '#6b7280' }}>{modeGeometrySuffix(c)}</span>
                      )}
                    </>
                  ) : (
                    <span style={{ color: '#9ca3af' }}>—</span>
                  )}
                </td>
                <td style={{ padding: 8 }}>
                  <div style={{ display: 'flex', gap: 6 }}>
                    <button
                      type="button"
                      style={neutralButtonStyle}
                      onClick={() => openEditModal(c)}
                      disabled={busy}
                    >
                      Edit
                    </button>
                    <Link
                      to={`/cameras/${c.id}/calibrate`}
                      style={{
                        ...neutralButtonStyle,
                        textDecoration: 'none',
                        display: 'inline-block',
                      }}
                    >
                      Calibrate
                    </Link>
                    <button
                      type="button"
                      style={dangerButtonStyle}
                      onClick={() => onDelete(c.id)}
                      disabled={busy}
                    >
                      Delete
                    </button>
                  </div>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      )}

      {addOpen && (
        <div role="dialog" aria-modal="true" style={modalOverlay} onClick={closeAddModal}>
          <div style={modalCard} onClick={(e) => e.stopPropagation()}>
            <h3 style={{ marginTop: 0 }}>Add camera</h3>
            <form onSubmit={onSubmitAdd}>
              <label style={{ display: 'block', marginBottom: 12 }}>
                <span style={{ display: 'block', fontSize: 13, color: '#374151', marginBottom: 4 }}>
                  Name
                </span>
                <input
                  aria-label="New camera name"
                  placeholder="e.g. front-left"
                  value={addName}
                  onChange={(e) => setAddName(e.target.value)}
                  style={{ ...inputStyle, width: '100%' }}
                  autoFocus
                />
              </label>
              <label style={{ display: 'block', marginBottom: 12 }}>
                <span style={{ display: 'block', fontSize: 13, color: '#374151', marginBottom: 4 }}>
                  Spinnaker camera
                </span>
                {addLoading ? (
                  <p style={{ color: '#666', margin: 0 }}>Detecting connected cameras…</p>
                ) : available && available.length === 0 ? (
                  <p style={{ color: '#666', margin: 0 }}>
                    No unmapped Spinnaker cameras detected. Plug one in, then reopen this dialog.
                  </p>
                ) : (
                  <select
                    aria-label="Spinnaker camera"
                    value={addSerial}
                    onChange={(e) => setAddSerial(e.target.value)}
                    style={{ ...inputStyle, width: '100%', background: '#fff' }}
                  >
                    {available?.map((a) => (
                      <option key={a.serial} value={a.serial}>
                        {a.model || a.serial} — {a.serial}
                        {a.vendor ? ` (${a.vendor})` : ''}
                      </option>
                    ))}
                  </select>
                )}
              </label>
              {addSerial && available && available.length > 0 && (
                <label style={{ display: 'block', marginBottom: 12 }}>
                  <span style={{ display: 'block', fontSize: 13, color: '#374151', marginBottom: 4 }}>
                    Mode
                  </span>
                  <ModeSelect
                    value={addSelectedMode}
                    options={addModes.options}
                    loading={addModes.loading}
                    error={addModes.error}
                    onChange={setAddSelectedMode}
                    hint="This camera doesn't expose a Mode setting."
                  />
                </label>
              )}
              <label style={{ display: 'block', marginBottom: 12 }}>
                <span style={{ display: 'block', fontSize: 13, color: '#374151', marginBottom: 4 }}>
                  Lens focal length (mm)
                </span>
                <input
                  type="number"
                  step="0.1"
                  min="0.1"
                  max="200"
                  aria-label="Lens focal length (mm)"
                  placeholder="e.g. 6.0"
                  value={addFocalMm}
                  onChange={(e) => setAddFocalMm(e.target.value)}
                  style={{ ...inputStyle, width: '100%' }}
                />
                <LensPreview focalMm={parseFloat(addFocalMm)} />
              </label>
              {addError && (
                <p style={{ color: 'crimson', marginTop: 8, marginBottom: 8 }}>{addError}</p>
              )}
              <div style={{ display: 'flex', justifyContent: 'flex-end', gap: 8, marginTop: 16 }}>
                <button type="button" style={neutralButtonStyle} onClick={closeAddModal} disabled={busy}>
                  Cancel
                </button>
                <button
                  type="submit"
                  style={primaryButtonStyle}
                  disabled={
                    busy ||
                    addLoading ||
                    addModes.loading ||
                    !addName.trim() ||
                    !addSerial ||
                    !(parseFloat(addFocalMm) > 0)
                  }
                >
                  Add
                </button>
              </div>
            </form>
          </div>
        </div>
      )}

      {editCamera && (
        <div role="dialog" aria-modal="true" style={modalOverlay} onClick={closeEditModal}>
          <div style={modalCard} onClick={(e) => e.stopPropagation()}>
            <h3 style={{ marginTop: 0 }}>Edit camera</h3>
            <form onSubmit={onSubmitEdit}>
              <label style={{ display: 'block', marginBottom: 12 }}>
                <span style={{ display: 'block', fontSize: 13, color: '#374151', marginBottom: 4 }}>
                  Name
                </span>
                <input
                  aria-label={`Edit name for camera ${editCamera.id}`}
                  value={editName}
                  onChange={(e) => setEditName(e.target.value)}
                  style={{ ...inputStyle, width: '100%' }}
                  autoFocus
                />
              </label>
              <div style={{ marginBottom: 12 }}>
                <span style={{ display: 'block', fontSize: 13, color: '#374151', marginBottom: 4 }}>
                  Serial
                </span>
                <code style={{ fontSize: 13, color: '#4b5563' }}>{editCamera.serial}</code>
              </div>
              <label style={{ display: 'block', marginBottom: 12 }}>
                <span style={{ display: 'block', fontSize: 13, color: '#374151', marginBottom: 4 }}>
                  Mode
                </span>
                <ModeSelect
                  value={editSelectedMode}
                  options={editModes.options}
                  loading={editModes.loading}
                  error={editModes.error}
                  disabled={editModes.offline}
                  onChange={setEditSelectedMode}
                  hint={
                    editModes.offline
                      ? 'Camera must be online to change Mode.'
                      : "This camera doesn't expose a Mode setting."
                  }
                  disabledHint="Camera must be online to change Mode."
                />
              </label>
              <label style={{ display: 'block', marginBottom: 12 }}>
                <span style={{ display: 'block', fontSize: 13, color: '#374151', marginBottom: 4 }}>
                  Lens focal length (mm)
                </span>
                <input
                  type="number"
                  step="0.1"
                  min="0.1"
                  max="200"
                  aria-label="Lens focal length (mm)"
                  value={editFocalMm}
                  onChange={(e) => setEditFocalMm(e.target.value)}
                  style={{ ...inputStyle, width: '100%' }}
                />
                <LensPreview focalMm={parseFloat(editFocalMm)} />
              </label>
              {editError && (
                <p style={{ color: 'crimson', marginTop: 8, marginBottom: 8 }}>{editError}</p>
              )}
              <div style={{ display: 'flex', justifyContent: 'flex-end', gap: 8, marginTop: 16 }}>
                <button type="button" style={neutralButtonStyle} onClick={closeEditModal} disabled={busy}>
                  Cancel
                </button>
                <button
                  type="submit"
                  style={primaryButtonStyle}
                  disabled={busy || editModes.loading || !editName.trim()}
                >
                  Save
                </button>
              </div>
            </form>
          </div>
        </div>
      )}
    </div>
  )
}
