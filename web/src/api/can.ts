import { getJson, sendJson } from './http'

export type CanMode = 'off' | 'roborio' | 'systemcore'

export interface CanConfig {
  mode: CanMode
  updated_at: number
}

// PUT result: the DB update always succeeds; `pushed` reports whether the
// live CAN_MODE push to the Teensy worked (it re-syncs on reconnect anyway).
export interface CanConfigPutResult extends CanConfig {
  pushed: boolean
  push_error?: string | null
}

export interface CanStatus {
  mode: CanMode | null
  fw_mode: string | null // 'off' | 'classic' | 'fd' as reported by firmware
  teensy_connected: boolean
  telemetry_connected: boolean
  fw_version?: number | string | null
  can_ok: boolean
  odom: {
    rate_hz: number
    packets: number
    last_age_ms?: number | null
    last: {
      t_ns: number
      t_arrival_ns: number
      rio_time_us: number
      vx_mps: number
      vy_mps: number
      omega_radps: number
      status_flags: number
      counter: number
    } | null
  }
  counters: {
    can_rx: number
    can_rx_drops: number
    odom_tx_fw_drops: number
    odom_crc_errors: number
    pose_tx_fw: number
    pose_sent: number
    pose_send_errors: number
  }
  clock_sync: {
    healthy: boolean
    offset_us: number
    drift_ppm: number
    samples: number
    resets: number
  }
}

export async function fetchCanConfig(): Promise<CanConfig> {
  return getJson<CanConfig>('/api/can/config')
}

export async function updateCanConfig(mode: CanMode): Promise<CanConfigPutResult> {
  return sendJson<CanConfigPutResult>('/api/can/config', 'PUT', { mode })
}

export async function fetchCanStatus(): Promise<CanStatus> {
  return getJson<CanStatus>('/api/can/status')
}

// Bench/debug downlink: hand-craft a pose frame to the controller.
export async function postBenchPose(input: {
  x: number
  y: number
  theta: number
  quality?: number
}): Promise<{ sent: boolean }> {
  return sendJson('/api/can/pose', 'POST', input)
}
