import { asJson, readError } from './http'

// One named, FPS-configured trigger group on the Teensy. `output_pins` are
// the physical Teensy outputs (1..6) that fire in lockstep for this group.
export interface TriggerGroup {
  id: number
  name: string
  fps: number
  output_pins: number[]
  created_at: number
}

// Snapshot of the Teensy connection + arm state. The host re-pushes the
// most-recent armed config on every reconnect, so `armed` reflects intent
// as well as current device state.
export interface HardwareSyncStatus {
  connected: boolean
  port: string | null
  armed: boolean
  last_pulse_age_ms: number | null
  total_pulses: number
  last_error: string | null
}

export async function getStatus(): Promise<HardwareSyncStatus> {
  return asJson<HardwareSyncStatus>(await fetch('/api/hardware-sync/status'))
}

export async function listGroups(): Promise<TriggerGroup[]> {
  return asJson<TriggerGroup[]>(await fetch('/api/hardware-sync/groups'))
}

export async function createGroup(input: {
  name: string
  fps: number
  output_pins: number[]
}): Promise<TriggerGroup> {
  const res = await fetch('/api/hardware-sync/groups', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(input),
  })
  return asJson<TriggerGroup>(res)
}

export async function updateGroup(
  id: number,
  patch: { name?: string; fps?: number; output_pins?: number[] },
): Promise<TriggerGroup> {
  const res = await fetch(`/api/hardware-sync/groups/${id}`, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(patch),
  })
  return asJson<TriggerGroup>(res)
}

export async function deleteGroup(id: number): Promise<void> {
  const res = await fetch(`/api/hardware-sync/groups/${id}`, { method: 'DELETE' })
  if (!res.ok) throw new Error(await readError(res))
}

export async function arm(): Promise<void> {
  const res = await fetch('/api/hardware-sync/arm', { method: 'POST' })
  if (!res.ok) throw new Error(await readError(res))
}

export async function stopOutputs(): Promise<void> {
  const res = await fetch('/api/hardware-sync/stop', { method: 'POST' })
  if (!res.ok) throw new Error(await readError(res))
}
