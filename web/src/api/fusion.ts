import { getJson, sendJson } from './http'
import type { Mat4 } from '@/lib/matrices'

export interface LatencyStat {
  last_ms: number
  p95_ms: number
  count: number
}

export interface FusionStatus {
  enabled: boolean
  reason: string
  // Degraded-modes matrix string from derive_fusion_mode (docs/pose_pipeline.md §6).
  mode: string
  initialized: boolean
  collision_mode: boolean
  reinits: number
  quality: number
  pose: {
    x_m: number
    y_m: number
    theta_rad: number
    t_ns: number
  } | null
  T_field_robot: Mat4 | null
  sources: {
    tag: {
      rate_hz: number
      last_age_ms?: number | null
      accepted: number
      rejected_gate: number
      rejected_clock: number
      rejected_stale: number
      bus_dropped: number
    }
    vio: {
      enabled: boolean
      reason?: string | null
      rate_hz: number
      last_age_ms?: number | null
      fused_intervals: number
      skipped_epoch: number
      skipped_unhealthy: number
      bus_dropped: number
    }
    odom: {
      rate_hz: number
      last_age_ms?: number | null
      fused_intervals: number
      stale: number
      slip: number
      bus_dropped: number
    }
  }
  solve_ms: { last: number; p95: number }
  latency: {
    tag_pulse_to_fusion: LatencyStat
    queue_wait: LatencyStat
    solve: LatencyStat
    // Trigger-pulse → pose-on-wire headline; p95 target < 50 ms.
    pose_staleness: LatencyStat
  }
  lag: { states: number; lag_s: number; oldest_age_s: number }
  sync_clock_now: { healthy: boolean; offset_ms: number }
  output: {
    sent: number
    send_errors: number
    queue_dropped: number
    bridge_factors: number
    gate_reopens: number
    update_exceptions: number
  }
}

export interface FusionConfig {
  enabled: boolean
  lag_s: number
  min_state_dt_ms: number
  output_hz: number
  max_extrapolation_ms: number
  tag_gate_chi2: number
  tag_huber_k: number
  vio_huber_k: number
  odom_cauchy_k: number
  odom_sigma_vx: number
  odom_sigma_vy: number
  odom_sigma_omega: number
  vio_sigma_rot: number
  vio_sigma_trans: number
  collision_inflation: number
  collision_window: number
  reinit_pos_std_m: number
  updated_at: number
}

export type FusionConfigPatch = Partial<Omit<FusionConfig, 'updated_at'>>

export async function fetchFusionStatus(): Promise<FusionStatus> {
  return getJson<FusionStatus>('/api/fusion/status')
}

export async function fetchFusionConfig(): Promise<FusionConfig> {
  return getJson<FusionConfig>('/api/fusion/config')
}

export async function updateFusionConfig(
  patch: FusionConfigPatch,
): Promise<FusionConfig & { restarted: boolean }> {
  return sendJson('/api/fusion/config', 'PUT', patch)
}

export async function resetFusion(): Promise<{ reinits: number }> {
  return sendJson('/api/fusion/reset', 'POST')
}
