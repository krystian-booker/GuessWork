import type { CameraMode } from '../api/cameras'

// Builds the bracketed "[1280x960 @ 30fps]" suffix shown after the mode name.
// Returns an empty string if neither geometry nor frame rate is available.
function modeSuffix(m: CameraMode): string {
  const parts: string[] = []
  if (m.width != null && m.height != null) {
    parts.push(`${m.width}x${m.height}`)
  }
  if (m.max_fps != null) {
    // Round to nearest integer; if very slow (<1fps) keep one decimal.
    const fps = m.max_fps >= 1 ? Math.round(m.max_fps) : Math.round(m.max_fps * 10) / 10
    parts.push(`${fps}fps`)
  }
  if (parts.length === 0) return ''
  return ` [${parts.join(' @ ')}]`
}

function modeLabel(m: CameraMode): string {
  const base = m.display_name || m.name
  return `${base}${modeSuffix(m)}`
}

const selectStyle: React.CSSProperties = {
  padding: '6px 10px',
  fontSize: 14,
  border: '1px solid #ccc',
  borderRadius: 6,
  width: '100%',
  background: '#fff',
}

interface Props {
  value: string | null
  options: CameraMode[]
  loading: boolean
  error: string | null
  disabled?: boolean
  onChange: (mode: string) => void
  // Shown in place of the dropdown when the camera doesn't expose a Mode node
  // (options is empty + not loading + no error).
  hint?: string
  // Shown when disabled and the camera has options (e.g. offline edit).
  disabledHint?: string
}

export default function ModeSelect({
  value,
  options,
  loading,
  error,
  disabled,
  onChange,
  hint,
  disabledHint,
}: Props) {
  if (loading) {
    return <p style={{ color: '#666', margin: 0 }}>Loading modes…</p>
  }
  if (error) {
    return <p style={{ color: 'crimson', margin: 0 }}>{error}</p>
  }
  if (options.length === 0) {
    return <p style={{ color: '#666', margin: 0 }}>{hint ?? 'No modes available.'}</p>
  }

  const selected = options.find((o) => o.name === value) ?? options[0]

  return (
    <div>
      <select
        aria-label="Camera mode"
        value={selected.name}
        onChange={(e) => onChange(e.target.value)}
        disabled={disabled}
        style={{ ...selectStyle, opacity: disabled ? 0.6 : 1 }}
      >
        {options.map((o) => (
          <option key={o.name} value={o.name}>
            {modeLabel(o)}
          </option>
        ))}
      </select>
      {selected.description && (
        <p style={{ marginTop: 6, marginBottom: 0, fontSize: 13, color: '#4b5563' }}>
          {selected.description}
        </p>
      )}
      {disabled && disabledHint && (
        <p style={{ marginTop: 6, marginBottom: 0, fontSize: 13, color: '#a16207' }}>
          {disabledHint}
        </p>
      )}
    </div>
  )
}
