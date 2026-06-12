import { getJson, sendJson } from './http'
import type { Mat4 } from '@/lib/matrices'

export interface VioStatus {
  enabled: boolean
  // Why VIO is disabled (missing roles, unparseable extrinsics, ...).
  reason: string
  running: boolean
  initialized: boolean
  phase: string
  epoch: number
  reinits: number
  freq_hz: number
  tracked_features: number
  cov_pos_std_m: number
  imu_rate_hz: number
  counters: {
    paired: number
    dropped_zero_ts: number
    dropped_unmatched: number
    dropped_pair_queue: number
    frames_fed: number
    imu_fed: number
    imu_bus_dropped: number
  }
  last_pose: {
    t_ns: number
    epoch: number
    T_odom_imu: Mat4
  } | null
  cameras: Array<{
    camera_id: number
    name: string
    role: string
    feeder_running: boolean
    reproj_std_px?: number | null
  }>
}

export interface VioConfig {
  enabled: boolean
  num_pts: number
  fast_threshold: number
  downsample: boolean
  max_reproj_std_px: number
  auto_reinit: boolean
  reinit_min_features: number
  reinit_window_frames: number
  reinit_max_pos_std_m: number
  updated_at: number
}

export type VioConfigPatch = Partial<Omit<VioConfig, 'updated_at'>>

export async function fetchVioStatus(): Promise<VioStatus> {
  return getJson<VioStatus>('/api/vio/status')
}

export async function fetchVioConfig(): Promise<VioConfig> {
  return getJson<VioConfig>('/api/vio/config')
}

export async function updateVioConfig(
  patch: VioConfigPatch,
): Promise<VioConfig & { restarted: boolean }> {
  return sendJson('/api/vio/config', 'PUT', patch)
}

export async function restartVio(): Promise<{ epoch: number; running: boolean }> {
  return sendJson('/api/vio/restart', 'POST')
}
