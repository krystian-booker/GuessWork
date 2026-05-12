import { useEffect, useState } from 'react'
import {
  createCamera,
  deleteCamera,
  listCameras,
  updateCamera,
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

export default function CamerasPage() {
  const [cameras, setCameras] = useState<Camera[] | null>(null)
  const [listError, setListError] = useState<string | null>(null)
  const [newName, setNewName] = useState('')
  const [createError, setCreateError] = useState<string | null>(null)
  const [busy, setBusy] = useState(false)
  const [editingId, setEditingId] = useState<number | null>(null)
  const [editingName, setEditingName] = useState('')
  const [editError, setEditError] = useState<string | null>(null)

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

  const onCreate = async (e: React.FormEvent) => {
    e.preventDefault()
    if (!newName.trim() || busy) return
    setBusy(true)
    setCreateError(null)
    try {
      await createCamera(newName.trim())
      setNewName('')
      await refresh()
    } catch (err) {
      setCreateError(err instanceof Error ? err.message : String(err))
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
      <h2 style={{ marginBottom: 8 }}>Cameras</h2>

      <form onSubmit={onCreate} style={{ display: 'flex', gap: 8, marginBottom: 16 }}>
        <input
          aria-label="New camera name"
          placeholder="Camera name"
          value={newName}
          onChange={(e) => setNewName(e.target.value)}
          style={{ ...inputStyle, flex: 1 }}
        />
        <button type="submit" style={primaryButtonStyle} disabled={busy || !newName.trim()}>
          Add
        </button>
      </form>
      {createError && (
        <p style={{ color: 'crimson', marginTop: -8, marginBottom: 16 }}>{createError}</p>
      )}

      {listError && <p style={{ color: 'crimson' }}>{listError}</p>}

      {cameras === null && !listError && <p>Loading…</p>}

      {cameras && cameras.length === 0 && (
        <p style={{ color: '#666' }}>No cameras yet. Add one above.</p>
      )}

      {cameras && cameras.length > 0 && (
        <table style={{ width: '100%', borderCollapse: 'collapse' }}>
          <thead>
            <tr style={{ borderBottom: '1px solid #ddd', textAlign: 'left' }}>
              <th style={{ padding: 8, width: 60 }}>ID</th>
              <th style={{ padding: 8 }}>Name</th>
              <th style={{ padding: 8, width: 220 }}>Actions</th>
            </tr>
          </thead>
          <tbody>
            {cameras.map((c) => {
              const isEditing = editingId === c.id
              return (
                <tr key={c.id} style={{ borderBottom: '1px solid #eee' }}>
                  <td style={{ padding: 8, color: '#666' }}>{c.id}</td>
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
    </div>
  )
}
