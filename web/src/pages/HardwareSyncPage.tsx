import { useEffect, useState } from 'react'
import {
  arm as armApi,
  createGroup,
  deleteGroup,
  getStatus,
  listGroups,
  stopOutputs,
  updateGroup,
  type HardwareSyncStatus,
  type TriggerGroup,
} from '../api/hardwareSync'
import { listCameras, type Camera } from '../api/cameras'
import {
  dangerButtonStyle,
  neutralButtonStyle,
  primaryButtonStyle,
} from '../components/buttonStyles'

const MAX_OUTPUTS = 6

const cardStyle: React.CSSProperties = {
  border: '1px solid #e5e7eb',
  borderRadius: 8,
  padding: 16,
  marginBottom: 16,
  background: '#fff',
}

const inputStyle: React.CSSProperties = {
  padding: '6px 10px',
  fontSize: 14,
  border: '1px solid #ccc',
  borderRadius: 6,
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

// Build a Set of pins claimed by `groups` excluding `excludeGroupId` (so the
// editor doesn't disable its own pins).
function pinsClaimedByOtherGroups(
  groups: TriggerGroup[], excludeGroupId: number | null,
): Set<number> {
  const out = new Set<number>()
  for (const g of groups) {
    if (g.id === excludeGroupId) continue
    for (const p of g.output_pins) out.add(p)
  }
  return out
}

function camerasOnPin(cameras: Camera[] | null, pin: number): Camera[] {
  if (!cameras) return []
  return cameras.filter((c) => c.trigger_output_pin === pin && c.hardware_sync_enabled)
}

interface GroupFormState {
  id: number | null
  name: string
  fps: string
  pins: Set<number>
}

const emptyForm: GroupFormState = {
  id: null,
  name: '',
  fps: '30',
  pins: new Set(),
}

function StatusBlock({ status }: { status: HardwareSyncStatus | null }) {
  if (!status) return <p style={{ color: '#666' }}>Loading status…</p>
  const dotColor = status.connected ? '#16a34a' : '#9ca3af'
  return (
    <div style={{ display: 'grid', gridTemplateColumns: '140px 1fr', rowGap: 6, fontSize: 14 }}>
      <span style={{ color: '#374151' }}>Connection</span>
      <span>
        <span style={{
          display: 'inline-block', width: 10, height: 10, borderRadius: '50%',
          background: dotColor, marginRight: 8,
        }} />
        {status.connected
          ? <>connected · <code>{status.port}</code></>
          : 'waiting for Teensy…'}
      </span>
      <span style={{ color: '#374151' }}>State</span>
      <span style={{ color: status.armed ? '#15803d' : '#6b7280' }}>
        {status.armed ? 'armed (pulsing)' : 'idle'}
      </span>
      <span style={{ color: '#374151' }}>Pulses</span>
      <span>{status.total_pulses.toLocaleString()}</span>
      <span style={{ color: '#374151' }}>Last pulse</span>
      <span>
        {status.last_pulse_age_ms == null
          ? <span style={{ color: '#9ca3af' }}>—</span>
          : `${status.last_pulse_age_ms} ms ago`}
      </span>
      {status.last_error && (
        <>
          <span style={{ color: '#374151' }}>Last error</span>
          <span style={{ color: '#b91c1c' }}>{status.last_error}</span>
        </>
      )}
    </div>
  )
}

export default function HardwareSyncPage() {
  const [status, setStatus] = useState<HardwareSyncStatus | null>(null)
  const [groups, setGroups] = useState<TriggerGroup[] | null>(null)
  const [cameras, setCameras] = useState<Camera[] | null>(null)
  const [loadErr, setLoadErr] = useState<string | null>(null)
  const [actionErr, setActionErr] = useState<string | null>(null)
  const [busy, setBusy] = useState(false)

  const [form, setForm] = useState<GroupFormState>(emptyForm)
  const [formOpen, setFormOpen] = useState(false)
  const [formErr, setFormErr] = useState<string | null>(null)

  const refresh = async () => {
    try {
      const [s, g, c] = await Promise.all([getStatus(), listGroups(), listCameras()])
      setStatus(s)
      setGroups(g)
      setCameras(c)
      setLoadErr(null)
    } catch (e) {
      setLoadErr(e instanceof Error ? e.message : String(e))
    }
  }

  useEffect(() => {
    refresh()
    const t = setInterval(() => {
      getStatus()
        .then(setStatus)
        .catch(() => {/* transient */})
    }, 1000)
    return () => clearInterval(t)
  }, [])

  const openCreate = () => {
    setForm({ ...emptyForm, pins: new Set() })
    setFormErr(null)
    setFormOpen(true)
  }
  const openEdit = (g: TriggerGroup) => {
    setForm({
      id: g.id,
      name: g.name,
      fps: g.fps.toString(),
      pins: new Set(g.output_pins),
    })
    setFormErr(null)
    setFormOpen(true)
  }
  const closeForm = () => { if (!busy) setFormOpen(false) }

  const togglePinInForm = (pin: number) => {
    setForm((f) => {
      const next = new Set(f.pins)
      if (next.has(pin)) next.delete(pin)
      else                next.add(pin)
      return { ...f, pins: next }
    })
  }

  const onSubmitForm = async (e: React.FormEvent) => {
    e.preventDefault()
    if (busy) return
    const name = form.name.trim()
    const fpsNum = parseFloat(form.fps)
    if (!name) { setFormErr('Name is required.');                     return }
    if (!Number.isFinite(fpsNum) || fpsNum <= 0) {
      setFormErr('FPS must be a positive number.');                   return
    }
    if (form.pins.size === 0) {
      setFormErr('Pick at least one output pin.');                    return
    }
    setBusy(true)
    setFormErr(null)
    try {
      const pins = Array.from(form.pins).sort((a, b) => a - b)
      if (form.id == null) await createGroup({ name, fps: fpsNum, output_pins: pins })
      else                  await updateGroup(form.id, { name, fps: fpsNum, output_pins: pins })
      setFormOpen(false)
      await refresh()
    } catch (err) {
      setFormErr(err instanceof Error ? err.message : String(err))
    } finally {
      setBusy(false)
    }
  }

  const onDelete = async (id: number) => {
    if (busy) return
    setBusy(true)
    setActionErr(null)
    try {
      await deleteGroup(id)
      await refresh()
    } catch (err) {
      setActionErr(err instanceof Error ? err.message : String(err))
    } finally {
      setBusy(false)
    }
  }

  const onArm = async () => {
    if (busy) return
    setBusy(true)
    setActionErr(null)
    try {
      await armApi()
      await refresh()
    } catch (err) {
      setActionErr(err instanceof Error ? err.message : String(err))
    } finally {
      setBusy(false)
    }
  }

  const onStop = async () => {
    if (busy) return
    setBusy(true)
    setActionErr(null)
    try {
      await stopOutputs()
      await refresh()
    } catch (err) {
      setActionErr(err instanceof Error ? err.message : String(err))
    } finally {
      setBusy(false)
    }
  }

  const groupForPin = (pin: number): TriggerGroup | null => {
    if (!groups) return null
    for (const g of groups) if (g.output_pins.includes(pin)) return g
    return null
  }

  return (
    <div>
      <h2 style={{ marginTop: 0 }}>Hardware Sync</h2>
      {loadErr && <p style={{ color: 'crimson' }}>{loadErr}</p>}

      <div style={cardStyle}>
        <h3 style={{ marginTop: 0, marginBottom: 12 }}>Teensy</h3>
        <StatusBlock status={status} />
        <div style={{ display: 'flex', gap: 8, marginTop: 16 }}>
          <button
            type="button"
            style={primaryButtonStyle}
            onClick={onArm}
            disabled={busy || !status?.connected || (groups?.length ?? 0) === 0}
            title={!status?.connected ? 'Plug in the Teensy 4.1' : ''}
          >
            Arm
          </button>
          <button
            type="button"
            style={neutralButtonStyle}
            onClick={onStop}
            disabled={busy || !status?.connected}
          >
            Stop
          </button>
        </div>
        {actionErr && <p style={{ color: 'crimson', marginTop: 12 }}>{actionErr}</p>}
      </div>

      <div style={cardStyle}>
        <div style={{ display: 'flex', justifyContent: 'space-between',
                       alignItems: 'center', marginBottom: 12 }}>
          <h3 style={{ margin: 0 }}>Outputs</h3>
        </div>
        <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: 14 }}>
          <thead>
            <tr style={{ borderBottom: '1px solid #ddd', textAlign: 'left' }}>
              <th style={{ padding: 8, width: 60 }}>#</th>
              <th style={{ padding: 8, width: 200 }}>Group</th>
              <th style={{ padding: 8, width: 80 }}>FPS</th>
              <th style={{ padding: 8 }}>Camera wired to this pin</th>
            </tr>
          </thead>
          <tbody>
            {Array.from({ length: MAX_OUTPUTS }, (_, i) => i + 1).map((pin) => {
              const g = groupForPin(pin)
              const cams = camerasOnPin(cameras, pin)
              return (
                <tr key={pin} style={{ borderBottom: '1px solid #f1f5f9' }}>
                  <td style={{ padding: 8, color: '#374151' }}>Output {pin}</td>
                  <td style={{ padding: 8 }}>
                    {g
                      ? <span style={{ color: '#0f172a' }}>{g.name}</span>
                      : <span style={{ color: '#9ca3af' }}>—</span>}
                  </td>
                  <td style={{ padding: 8, color: '#4b5563' }}>
                    {g ? g.fps : <span style={{ color: '#9ca3af' }}>—</span>}
                  </td>
                  <td style={{ padding: 8, color: '#4b5563' }}>
                    {cams.length === 0
                      ? <span style={{ color: '#9ca3af' }}>—</span>
                      : cams.map((c) => c.name).join(', ')}
                  </td>
                </tr>
              )
            })}
          </tbody>
        </table>
      </div>

      <div style={cardStyle}>
        <div style={{ display: 'flex', justifyContent: 'space-between',
                       alignItems: 'center', marginBottom: 12 }}>
          <h3 style={{ margin: 0 }}>Trigger groups</h3>
          <button type="button" style={primaryButtonStyle} onClick={openCreate} disabled={busy}>
            + New group
          </button>
        </div>
        {!groups
          ? <p style={{ color: '#666' }}>Loading…</p>
          : groups.length === 0
            ? <p style={{ color: '#6b7280', margin: 0 }}>
                No groups yet. Create one and assign outputs to it.
              </p>
            : (
              <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: 14 }}>
                <thead>
                  <tr style={{ borderBottom: '1px solid #ddd', textAlign: 'left' }}>
                    <th style={{ padding: 8 }}>Name</th>
                    <th style={{ padding: 8, width: 80 }}>FPS</th>
                    <th style={{ padding: 8, width: 180 }}>Pins</th>
                    <th style={{ padding: 8, width: 160 }}>Actions</th>
                  </tr>
                </thead>
                <tbody>
                  {groups.map((g) => (
                    <tr key={g.id} style={{ borderBottom: '1px solid #f1f5f9' }}>
                      <td style={{ padding: 8 }}>{g.name}</td>
                      <td style={{ padding: 8, color: '#4b5563' }}>{g.fps}</td>
                      <td style={{ padding: 8, color: '#4b5563' }}>
                        {g.output_pins.join(', ')}
                      </td>
                      <td style={{ padding: 8 }}>
                        <div style={{ display: 'flex', gap: 6 }}>
                          <button
                            type="button"
                            style={neutralButtonStyle}
                            onClick={() => openEdit(g)}
                            disabled={busy}
                          >
                            Edit
                          </button>
                          <button
                            type="button"
                            style={dangerButtonStyle}
                            onClick={() => onDelete(g.id)}
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
      </div>

      {formOpen && (
        <div role="dialog" aria-modal="true" style={modalOverlay} onClick={closeForm}>
          <div style={modalCard} onClick={(e) => e.stopPropagation()}>
            <h3 style={{ marginTop: 0 }}>
              {form.id == null ? 'New trigger group' : 'Edit trigger group'}
            </h3>
            <form onSubmit={onSubmitForm}>
              <label style={{ display: 'block', marginBottom: 12 }}>
                <span style={{ display: 'block', fontSize: 13, color: '#374151', marginBottom: 4 }}>
                  Name
                </span>
                <input
                  value={form.name}
                  onChange={(e) => setForm({ ...form, name: e.target.value })}
                  style={{ ...inputStyle, width: '100%' }}
                  placeholder="e.g. apriltag_trigger"
                  autoFocus
                />
              </label>
              <label style={{ display: 'block', marginBottom: 12 }}>
                <span style={{ display: 'block', fontSize: 13, color: '#374151', marginBottom: 4 }}>
                  FPS
                </span>
                <input
                  type="number"
                  min="0.1"
                  step="0.1"
                  value={form.fps}
                  onChange={(e) => setForm({ ...form, fps: e.target.value })}
                  style={{ ...inputStyle, width: '100%' }}
                />
              </label>
              <div style={{ marginBottom: 12 }}>
                <span style={{ display: 'block', fontSize: 13, color: '#374151', marginBottom: 4 }}>
                  Outputs
                </span>
                <div style={{ display: 'flex', flexWrap: 'wrap', gap: 8 }}>
                  {Array.from({ length: MAX_OUTPUTS }, (_, i) => i + 1).map((pin) => {
                    const claimed = pinsClaimedByOtherGroups(groups ?? [], form.id)
                    const disabled = claimed.has(pin)
                    return (
                      <label key={pin} style={{
                        display: 'flex', alignItems: 'center', gap: 4, fontSize: 13,
                        opacity: disabled ? 0.5 : 1,
                      }}>
                        <input
                          type="checkbox"
                          checked={form.pins.has(pin)}
                          disabled={disabled}
                          onChange={() => togglePinInForm(pin)}
                        />
                        Output {pin}{disabled ? ' (in use)' : ''}
                      </label>
                    )
                  })}
                </div>
              </div>
              {formErr && <p style={{ color: 'crimson' }}>{formErr}</p>}
              <div style={{ display: 'flex', justifyContent: 'flex-end', gap: 8 }}>
                <button type="button" style={neutralButtonStyle} onClick={closeForm} disabled={busy}>
                  Cancel
                </button>
                <button type="submit" style={primaryButtonStyle} disabled={busy}>
                  {form.id == null ? 'Create' : 'Save'}
                </button>
              </div>
            </form>
          </div>
        </div>
      )}
    </div>
  )
}
