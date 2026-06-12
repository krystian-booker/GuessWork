import { getJson } from './http'
import type { Mat4 } from '@/lib/matrices'

export interface TagObservation {
  id: number
  decision_margin: number
  range_m: number | null
}

export interface AprilTagCameraStatus {
  camera_id: number
  name: string
  running: boolean
  // Why the consumer isn't publishing (e.g. "offline", "no_extrinsics_chain").
  reason?: string
  det_per_s: number
  frames_seen: number
  detections_total: number
  published: number
  skipped_no_tags: number
  skipped_ambiguous: number
  skipped_high_reproj: number
  skipped_no_extrinsics: number
  skipped_solve_failed: number
  last_latency_ms: number
  latency_ewma_ms: number
  mean_reproj_err_px: number
  last_tags: TagObservation[]
  last_pose: Mat4 | null
  last_pose_t_ns: number | null
}

export interface AprilTagStatus {
  active_layout_id?: number
  active_layout_name?: string
  t_robot_imu_set: boolean
  cameras: AprilTagCameraStatus[]
}

export async function fetchAprilTagStatus(): Promise<AprilTagStatus> {
  return getJson<AprilTagStatus>('/api/apriltag/status')
}
