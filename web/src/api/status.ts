import { asJson } from './http'

export interface CameraStatus {
  id: number
  name: string
  serial: string
  online: boolean
  frames_produced: number
  frames_dropped: number
  frames_incomplete: number
  fps_1s: number
}

export interface Status {
  ok: boolean
  uptime_s: number
  cameras: CameraStatus[]
}

export async function fetchStatus(): Promise<Status> {
  return asJson<Status>(await fetch('/api/status'))
}
