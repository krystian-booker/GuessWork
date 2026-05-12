import { useEffect, useState } from 'react'
import { fetchStatus, type Status } from '../api/status'
import Stream from '../Stream'

export default function StreamPage() {
  const [status, setStatus] = useState<Status | null>(null)
  const [error, setError] = useState<string | null>(null)

  useEffect(() => {
    let cancelled = false
    const tick = async () => {
      try {
        const s = await fetchStatus()
        if (!cancelled) {
          setStatus(s)
          setError(null)
        }
      } catch (e) {
        if (!cancelled) setError(e instanceof Error ? e.message : String(e))
      }
    }
    tick()
    const id = setInterval(tick, 1000)
    return () => {
      cancelled = true
      clearInterval(id)
    }
  }, [])

  return (
    <>
      <Stream />
      <h2 style={{ marginBottom: 8 }}>Status</h2>
      {error && (
        <p style={{ color: 'crimson' }}>Failed to fetch /api/status: {error}</p>
      )}
      {status ? (
        <pre style={{ background: '#f4f4f4', padding: 16, borderRadius: 8 }}>
          {JSON.stringify(status, null, 2)}
        </pre>
      ) : (
        !error && <p>Loading…</p>
      )}
    </>
  )
}
