import { getJson, sendJson } from './http'
import type { Mat4 } from '@/lib/matrices'

export interface ImuStatus {
  teensy_connected: boolean
  telemetry_connected: boolean
  imu_ok: boolean
  rate_hz: number
  samples: number
  fw_drops: number
  crc_errors: number
  last_sample_age_ms?: number | null
  fw_version?: number | string | null
}

// t_imu_robot schema: {"T_robot_imu": [[4x4 row-major]]} (parsed/validated
// server-side by gw::calib::parse_t_robot_imu). null = unset, which disables
// AprilTag publishing and fusion VIO ingestion.
export interface TImuRobot {
  T_robot_imu: Mat4
}

export interface ImuConfig {
  rate_hz: number
  accel_noise_density: number
  accel_random_walk: number
  gyro_noise_density: number
  gyro_random_walk: number
  t_imu_robot: TImuRobot | null
  updated_at: number
}

export type ImuConfigPatch = Partial<Omit<ImuConfig, 'updated_at'>>

export interface AllanAxisResult {
  noise_density: number
  random_walk: number
  noise_density_ok: boolean
  random_walk_ok: boolean
  fit_quality: string
}

export interface AllanAnalysis {
  file: string
  samples: number
  duration_s: number
  rate_hz: number
  suggested: {
    accel_noise_density: number
    accel_random_walk: number
    gyro_noise_density: number
    gyro_random_walk: number
  }
  axes: Record<string, AllanAxisResult> // accel_x..gyro_z
  warnings: string[]
  analyzed_at: number
}

export interface AllanRecordingStatus {
  recording: boolean
  file: string | null
  samples: number
  bytes: number
  rate_hz: number
  remaining_s: number
}

export interface AllanStatus {
  recording: AllanRecordingStatus
  last_analysis: AllanAnalysis | null
}

export interface Vec3Fields {
  x: number
  y: number
  z: number
}

// Live attitude from the on-host complementary filter. `q` rotates BODY-frame
// vectors into the WORLD frame (Hamilton, world Z-up) and arrives
// hemisphere-canonicalized (w >= 0). Euler is ZYX. Yaw is gyro-only and
// drifts — zero-yaw re-references it.
export interface ImuAttitude {
  initialized: boolean
  rate_hz: number
  last_age_ms: number | null
  q: { w: number; x: number; y: number; z: number }
  euler: { roll_deg: number; pitch_deg: number; yaw_deg: number }
  accel_mps2: Vec3Fields
  gyro_radps: Vec3Fields
}

export async function fetchImuStatus(): Promise<ImuStatus> {
  return getJson<ImuStatus>('/api/imu/status')
}

export async function fetchImuAttitude(): Promise<ImuAttitude> {
  return getJson<ImuAttitude>('/api/imu/attitude')
}

export async function postZeroYaw(): Promise<{ ok: boolean }> {
  return sendJson('/api/imu/attitude/zero-yaw', 'POST')
}

export async function fetchImuConfig(): Promise<ImuConfig> {
  return getJson<ImuConfig>('/api/imu/config')
}

export async function updateImuConfig(
  patch: ImuConfigPatch,
): Promise<ImuConfig & { restarted?: boolean }> {
  return sendJson('/api/imu/config', 'PUT', patch)
}

export async function startAllanRecording(
  durationS: number,
): Promise<{ recording: boolean; file: string | null; duration_s: number }> {
  return sendJson('/api/imu/allan/recording', 'POST', { duration_s: durationS })
}

export async function stopAllanRecording(): Promise<{
  stopped: boolean
  samples: number
  file: string | null
}> {
  return sendJson('/api/imu/allan/recording', 'DELETE')
}

export async function fetchAllanStatus(): Promise<AllanStatus> {
  return getJson<AllanStatus>('/api/imu/allan/status')
}

// Defaults to the most recent log when `file` is omitted. Long-running for
// big logs — keep the button in a pending state.
export async function analyzeAllan(file?: string): Promise<AllanAnalysis> {
  return sendJson('/api/imu/allan/analyze', 'POST', file ? { file } : {})
}

export async function applyAllan(): Promise<ImuConfig & { restarted?: boolean }> {
  return sendJson('/api/imu/allan/apply', 'POST')
}
