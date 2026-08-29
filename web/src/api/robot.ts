import { getJson, sendJson } from './http'

export interface RobotConfig {
  enabled: boolean
  bind_port: number
  robot_port: number
  // "" = learn the robot's address from inbound packets.
  robot_ip: string
  updated_at: number
}

export type RobotConfigPatch = Partial<Omit<RobotConfig, 'updated_at'>>

// PUT result: the DB update is the source of truth; `restarted` reports
// whether the live UDP link rebind worked (restart_error carries the reason
// when it didn't).
export interface RobotConfigPutResult extends RobotConfig {
  restarted: boolean
  restart_error: string | null
}

// One hop of the timestamp chain (rio↔host over UDP, host↔controller over USB).
export interface ClockSyncHop {
  healthy: boolean
  offset_us: number
  drift_ppm: number
  samples: number
  resets: number
}

export interface RobotStatus {
  running: boolean
  bind_port: number
  // Learned (or configured) robot address; null until a packet arrives.
  robot_addr: string | null
  odom: {
    rate_hz: number
    packets: number
    rejected: number
    counter_gaps: number
    last_age_ms: number | null
    last: {
      vx_mps: number
      vy_mps: number
      omega_radps: number
      rio_time_us: number
      status_flags: number
      t_ns: number
    } | null
  }
  pose: {
    sent: number
    send_errors: number
    no_dest: number
  }
  clock_sync: {
    // Chain healthy = both hops healthy.
    healthy: boolean
    rio_host: ClockSyncHop
    host_sync_controller: ClockSyncHop
  }
}

export async function fetchRobotConfig(): Promise<RobotConfig> {
  return getJson<RobotConfig>('/api/robot/config')
}

export async function updateRobotConfig(
  patch: RobotConfigPatch,
): Promise<RobotConfigPutResult> {
  return sendJson<RobotConfigPutResult>('/api/robot/config', 'PUT', patch)
}

export async function fetchRobotStatus(): Promise<RobotStatus> {
  return getJson<RobotStatus>('/api/robot/status')
}

// Bench/debug downlink: hand-craft a pose packet to the controller. 503 when
// the link is down or no robot address has been learned yet.
export async function postBenchPose(input: {
  x: number
  y: number
  theta: number
  quality?: number
}): Promise<{ sent: boolean; mode: string }> {
  return sendJson('/api/robot/pose', 'POST', input)
}
