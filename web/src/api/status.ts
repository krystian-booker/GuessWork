export interface Status {
  ok: boolean
  uptime_s: number
  pipeline: {
    camera_connected: boolean
    frames_produced: number
    frames_dropped: number
    frames_incomplete: number
    fps_1s: number
  }
}

export async function fetchStatus(): Promise<Status> {
  const res = await fetch('/api/status')
  if (!res.ok) throw new Error(`HTTP ${res.status}`)
  return (await res.json()) as Status
}
