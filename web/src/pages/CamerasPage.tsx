import { useEffect, useState } from 'react'
import {
  createCamera,
  deleteCamera,
  listAvailableCameras,
  listCameras,
  updateCamera,
  type AvailableCamera,
  type Camera,
} from '../api/cameras'

const inputStyle: React.CSSProperties = {
  padding: '6px 10px',
  fontSize: 14,
  border: '1px solid #ccc',
  borderRadius: 6,
  minWidth: 0,
}

const buttonStyle = (color: string, fill: string, text: string): React.CSSProperties => ({
  padding: '6px 12px',
  fontSize: 14,
  borderWidth: 1,
  borderStyle: 'solid',
  borderColor: color,
  borderRadius: 6,
  background: fill,
  color: text,
  cursor: 'pointer',
})

const neutralButtonStyle = buttonStyle('#ccc', '#fafafa', '#222')
const primaryButtonStyle = buttonStyle('#2563eb', '#2563eb', '#fff')
const dangerButtonStyle = buttonStyle('#dc2626', '#fff', '#dc2626')

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

export default function CamerasPage() {
  const [cameras, setCameras] = useState<Camera[] | null>(null)
  const [listError, setListError] = useState<string | null>(null)
  const [busy, setBusy] = useState(false)
  const [editingId, setEditingId] = useState<number | null>(null)
  const [editingName, setEditingName] = useState('')
  const [editError, setEditError] = useState<string | null>(null)

  const [modalOpen, setModalOpen] = useState(false)
  const [modalName, setModalName] = useState('')
  const [modalSerial, setModalSerial] = useState('')
  const [available, setAvailable] = useState<AvailableCamera[] | null>(null)
  const [modalError, setModalError] = useState<string | null>(null)
  const [modalLoading, setModalLoading] = useState(false)

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

  const openAddModal = async () => {
    setModalOpen(true)
    setModalName('')
    setModalSerial('')
    setModalError(null)
    setModalLoading(true)
    setAvailable(null)
    try {
      const list = await listAvailableCameras()
      setAvailable(list)
      if (list.length > 0) setModalSerial(list[0].serial)
    } catch (e) {
      setModalError(e instanceof Error ? e.message : String(e))
      setAvailable([])
    } finally {
      setModalLoading(false)
    }
  }

  const closeModal = () => {
    if (busy) return
    setModalOpen(false)
  }

  const onSubmitModal = async (e: React.FormEvent) => {
    e.preventDefault()
    if (!modalName.trim() || !modalSerial || busy) return
    setBusy(true)
    setModalError(null)
    try {
      await createCamera(modalName.trim(), modalSerial)
      setModalOpen(false)
      await refresh()
    } catch (err) {
      setModalError(err instanceof Error ? err.message : String(err))
    } finally {
      setBusy(false)
    }
  }

  const startEdit = (c: Camera) => {
    setEditingId(c.id)
    setEditingName(c.name)
    setEditError(null)
  }

  const cancelEdit = () => {
    setEditingId(null)
    setEditingName('')
    setEditError(null)
  }

  const saveEdit = async (id: number) => {
    if (!editingName.trim() || busy) return
    setBusy(true)
    setEditError(null)
    try {
      await updateCamera(id, editingName.trim())
      cancelEdit()
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
              <th style={{ padding: 8, width: 220 }}>Actions</th>
            </tr>
          </thead>
          <tbody>
            {cameras.map((c) => {
              const isEditing = editingId === c.id
              return (
                <tr key={c.id} style={{ borderBottom: '1px solid #eee' }}>
                  <td style={{ padding: 8, color: '#666' }}>{c.id}</td>
                  <td style={{ padding: 8, color: '#374151' }}>
                    <OnlineDot online={c.online} />
                    {c.online ? 'online' : 'offline'}
                  </td>
                  <td style={{ padding: 8 }}>
                    {isEditing ? (
                      <input
                        aria-label={`Edit name for camera ${c.id}`}
                        value={editingName}
                        onChange={(e) => setEditingName(e.target.value)}
                        style={{ ...inputStyle, width: '100%' }}
                      />
                    ) : (
                      c.name
                    )}
                  </td>
                  <td style={{ padding: 8, fontFamily: 'monospace', color: '#374151' }}>
                    {c.serial}
                  </td>
                  <td style={{ padding: 8 }}>
                    {isEditing ? (
                      <div style={{ display: 'flex', gap: 6 }}>
                        <button
                          type="button"
                          style={primaryButtonStyle}
                          onClick={() => saveEdit(c.id)}
                          disabled={busy || !editingName.trim()}
                        >
                          Save
                        </button>
                        <button type="button" style={neutralButtonStyle} onClick={cancelEdit}>
                          Cancel
                        </button>
                      </div>
                    ) : (
                      <div style={{ display: 'flex', gap: 6 }}>
                        <button type="button" style={neutralButtonStyle} onClick={() => startEdit(c)}>
                          Edit
                        </button>
                        <button
                          type="button"
                          style={dangerButtonStyle}
                          onClick={() => onDelete(c.id)}
                          disabled={busy}
                        >
                          Delete
                        </button>
                      </div>
                    )}
                  </td>
                </tr>
              )
            })}
          </tbody>
        </table>
      )}
      {editError && <p style={{ color: 'crimson', marginTop: 8 }}>{editError}</p>}

      {modalOpen && (
        <div role="dialog" aria-modal="true" style={modalOverlay} onClick={closeModal}>
          <div style={modalCard} onClick={(e) => e.stopPropagation()}>
            <h3 style={{ marginTop: 0 }}>Add camera</h3>
            <form onSubmit={onSubmitModal}>
              <label style={{ display: 'block', marginBottom: 12 }}>
                <span style={{ display: 'block', fontSize: 13, color: '#374151', marginBottom: 4 }}>
                  Name
                </span>
                <input
                  aria-label="New camera name"
                  placeholder="e.g. front-left"
                  value={modalName}
                  onChange={(e) => setModalName(e.target.value)}
                  style={{ ...inputStyle, width: '100%' }}
                  autoFocus
                />
              </label>
              <label style={{ display: 'block', marginBottom: 12 }}>
                <span style={{ display: 'block', fontSize: 13, color: '#374151', marginBottom: 4 }}>
                  Spinnaker camera
                </span>
                {modalLoading ? (
                  <p style={{ color: '#666', margin: 0 }}>Detecting connected cameras…</p>
                ) : available && available.length === 0 ? (
                  <p style={{ color: '#666', margin: 0 }}>
                    No unmapped Spinnaker cameras detected. Plug one in, then reopen this dialog.
                  </p>
                ) : (
                  <select
                    aria-label="Spinnaker camera"
                    value={modalSerial}
                    onChange={(e) => setModalSerial(e.target.value)}
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
              {modalError && (
                <p style={{ color: 'crimson', marginTop: 8, marginBottom: 8 }}>{modalError}</p>
              )}
              <div style={{ display: 'flex', justifyContent: 'flex-end', gap: 8, marginTop: 16 }}>
                <button type="button" style={neutralButtonStyle} onClick={closeModal} disabled={busy}>
                  Cancel
                </button>
                <button
                  type="submit"
                  style={primaryButtonStyle}
                  disabled={busy || modalLoading || !modalName.trim() || !modalSerial}
                >
                  Add
                </button>
              </div>
            </form>
          </div>
        </div>
      )}
    </div>
  )
}
