import type { CameraMode } from '@/api/cameras'
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select'

// Builds the bracketed "[1280x960 @ 30fps]" suffix shown after the mode name.
function modeSuffix(m: CameraMode): string {
  const parts: string[] = []
  if (m.width != null && m.height != null) parts.push(`${m.width}x${m.height}`)
  if (m.max_fps != null) {
    const fps = m.max_fps >= 1 ? Math.round(m.max_fps) : Math.round(m.max_fps * 10) / 10
    parts.push(`${fps}fps`)
  }
  return parts.length ? ` [${parts.join(' @ ')}]` : ''
}

export function modeLabel(m: CameraMode): string {
  return `${m.display_name || m.name}${modeSuffix(m)}`
}

export function ModeSelect({
  value,
  options,
  loading,
  error,
  disabled,
  onChange,
  hint,
  disabledHint,
}: {
  value: string | null
  options: CameraMode[]
  loading: boolean
  error: string | null
  disabled?: boolean
  onChange: (mode: string) => void
  // Shown in place of the dropdown when the camera doesn't expose a Mode node.
  hint?: string
  // Shown when disabled and the camera has options (e.g. offline edit).
  disabledHint?: string
}) {
  if (loading) return <p className="text-sm text-muted-foreground">Loading modes…</p>
  if (error) return <p className="text-sm text-destructive">{error}</p>
  if (options.length === 0)
    return <p className="text-sm text-muted-foreground">{hint ?? 'No modes available.'}</p>

  const selected = options.find((o) => o.name === value) ?? options[0]

  return (
    <div className="space-y-1.5">
      <Select value={selected.name} onValueChange={onChange} disabled={disabled}>
        <SelectTrigger className="w-full" aria-label="Camera mode">
          <SelectValue />
        </SelectTrigger>
        <SelectContent>
          {options.map((o) => (
            <SelectItem key={o.name} value={o.name}>
              {modeLabel(o)}
            </SelectItem>
          ))}
        </SelectContent>
      </Select>
      {selected.description && (
        <p className="text-xs text-muted-foreground">{selected.description}</p>
      )}
      {disabled && disabledHint && <p className="text-xs text-warning">{disabledHint}</p>}
    </div>
  )
}
