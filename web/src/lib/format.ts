export function formatHz(hz: number | null | undefined, digits = 1): string {
  if (hz == null || !Number.isFinite(hz)) return '—'
  return `${hz.toFixed(digits)} Hz`
}

export function formatMs(ms: number | null | undefined, digits = 1): string {
  if (ms == null || !Number.isFinite(ms)) return '—'
  return `${ms.toFixed(digits)} ms`
}

export function formatMeters(m: number | null | undefined, digits = 2): string {
  if (m == null || !Number.isFinite(m)) return '—'
  return `${m.toFixed(digits)} m`
}

export function formatDeg(rad: number | null | undefined, digits = 1): string {
  if (rad == null || !Number.isFinite(rad)) return '—'
  return `${((rad * 180) / Math.PI).toFixed(digits)}°`
}

export function formatUptime(seconds: number | null | undefined): string {
  if (seconds == null || !Number.isFinite(seconds)) return '—'
  const s = Math.floor(seconds)
  const d = Math.floor(s / 86400)
  const h = Math.floor((s % 86400) / 3600)
  const m = Math.floor((s % 3600) / 60)
  if (d > 0) return `${d}d ${h}h ${m}m`
  if (h > 0) return `${h}h ${m}m`
  return `${m}m ${s % 60}s`
}

export function formatElapsedMs(ms: number | null | undefined): string {
  if (ms == null || !Number.isFinite(ms)) return '—'
  const total = Math.floor(ms / 1000)
  const m = Math.floor(total / 60)
  const s = total % 60
  return m > 0 ? `${m}m ${s.toString().padStart(2, '0')}s` : `${s}s`
}

// Unix seconds → local date-time. The backend emits created_at /
// calibrated_at as unix seconds.
export function formatUnixSeconds(t: number | null | undefined): string {
  if (t == null || t <= 0) return '—'
  return new Date(t * 1000).toLocaleString()
}

// Coarse "updated X ago" for values whose age is tracked client-side
// (use-change-age). Sub-1.5 s reads as live.
export function formatAgeMs(ms: number | null | undefined): string {
  if (ms == null || !Number.isFinite(ms)) return '—'
  if (ms < 1500) return 'just now'
  const s = Math.round(ms / 1000)
  if (s < 90) return `${s} s ago`
  const m = Math.round(s / 60)
  return `${m} min ago`
}

export function formatCount(n: number | null | undefined): string {
  if (n == null || !Number.isFinite(n)) return '—'
  return n.toLocaleString()
}
