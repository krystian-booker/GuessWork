import { useRef } from 'react'

// Milliseconds since `stamp` last changed, measured on the browser clock.
// The backend publishes pose timestamps on the sync controller clock domain, which the
// browser can't map — "time since the value last changed" is the honest local
// proxy. Relies on the caller re-rendering (the status polls tick at 1–2 s)
// to refresh the reading; no timer of its own.
export function useChangeAge(stamp: number | null | undefined): number | null {
  const ref = useRef<{ stamp: number; at: number } | null>(null)
  if (stamp == null) {
    ref.current = null
    return null
  }
  if (ref.current == null || ref.current.stamp !== stamp) {
    ref.current = { stamp, at: Date.now() }
  }
  return Date.now() - ref.current.at
}
