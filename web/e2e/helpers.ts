import type { Page } from '@playwright/test'

// Default healthy payloads for the status endpoints the AppShell header and
// Dashboard poll. Mocked specs install these so unrelated pages don't spam
// proxy errors; individual tests override whichever endpoint they exercise
// (later page.route registrations win).

export const defaultStatus = {
  ok: true,
  uptime_s: 4242,
  cameras: [
    {
      id: 1,
      name: 'front',
      serial: '11111111',
      online: true,
      frames_produced: 100000,
      frames_dropped: 0,
      frames_incomplete: 0,
      fps_1s: 30.1,
    },
  ],
}

export const defaultFusionStatus = {
  enabled: true,
  reason: '',
  mode: 'tags+vio+odom',
  initialized: true,
  collision_mode: false,
  reinits: 0,
  quality: 255,
  pose: { x_m: 4.2, y_m: 2.1, theta_rad: 0.5, t_ns: 123456789 },
  T_field_robot: [
    [1, 0, 0, 4.2],
    [0, 1, 0, 2.1],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ],
  sources: {
    tag: { rate_hz: 25, last_age_ms: 12, accepted: 1000, rejected_gate: 3, rejected_clock: 0, rejected_stale: 0, bus_dropped: 0 },
    vio: { enabled: true, reason: null, rate_hz: 28, last_age_ms: 40, fused_intervals: 900, skipped_epoch: 1, skipped_unhealthy: 0, bus_dropped: 0 },
    odom: { rate_hz: 75, last_age_ms: 8, fused_intervals: 4000, stale: 0, slip: 2, bus_dropped: 0 },
  },
  solve_ms: { last: 3.2, p95: 6.1 },
  latency: {
    tag_pulse_to_fusion: { last_ms: 18, p95_ms: 25, count: 1000 },
    queue_wait: { last_ms: 0.4, p95_ms: 1.1, count: 1000 },
    solve: { last_ms: 3.2, p95_ms: 6.1, count: 1000 },
    pose_staleness: { last_ms: 28, p95_ms: 41, count: 1000 },
  },
  lag: { states: 60, lag_s: 2.0, oldest_age_s: 1.9 },
  teensy_now: { healthy: true, offset_ms: 0.3 },
  output: { sent: 5000, send_errors: 0, queue_dropped: 0, bridge_factors: 12, gate_reopens: 0, update_exceptions: 0 },
}

export const defaultVioStatus = {
  enabled: true,
  reason: '',
  running: true,
  initialized: true,
  phase: 'tracking',
  epoch: 2,
  reinits: 1,
  freq_hz: 28.5,
  tracked_features: 142,
  cov_pos_std_m: 0.04,
  imu_rate_hz: 400,
  counters: { paired: 9000, dropped_zero_ts: 0, dropped_unmatched: 4, dropped_pair_queue: 0, frames_fed: 9000, imu_fed: 120000, imu_bus_dropped: 0 },
  last_pose: null,
  cameras: [
    { camera_id: 2, name: 'vio-left', role: 'vio_left', feeder_running: true, reproj_std_px: 0.31 },
    { camera_id: 3, name: 'vio-right', role: 'vio_right', feeder_running: true, reproj_std_px: 0.29 },
  ],
}

export const defaultAprilTagStatus = {
  active_layout_id: 1,
  active_layout_name: 'test-field',
  t_robot_imu_set: true,
  cameras: [
    {
      camera_id: 1,
      name: 'front',
      running: true,
      det_per_s: 22.4,
      frames_seen: 50000,
      detections_total: 30000,
      published: 29000,
      skipped_no_tags: 900,
      skipped_ambiguous: 50,
      skipped_high_reproj: 30,
      skipped_no_extrinsics: 0,
      skipped_solve_failed: 20,
      last_latency_ms: 12.5,
      latency_ewma_ms: 12.1,
      mean_reproj_err_px: 0.42,
      last_tags: [
        { id: 7, decision_margin: 60.2, range_m: 2.4 },
        { id: 8, decision_margin: 55.0, range_m: 3.1 },
      ],
      last_pose: null,
      last_pose_t_ns: null,
    },
  ],
}

export const defaultCanStatus = {
  mode: 'roborio',
  fw_mode: 'classic',
  teensy_connected: true,
  telemetry_connected: true,
  fw_version: 3,
  can_ok: true,
  odom: {
    rate_hz: 75.2,
    packets: 40000,
    last_age_ms: 9,
    last: { t_ns: 1, t_arrival_ns: 2, rio_time_us: 3, vx_mps: 1.2, vy_mps: 0.1, omega_radps: 0.4, status_flags: 0, counter: 41 },
  },
  counters: { can_rx: 40000, can_rx_drops: 0, odom_tx_fw_drops: 0, odom_crc_errors: 0, pose_tx_fw: 5000, pose_sent: 5000, pose_send_errors: 0 },
  clock_sync: { healthy: true, offset_us: 120, drift_ppm: 4.2, samples: 24, resets: 0 },
}

export const defaultImuStatus = {
  teensy_connected: true,
  telemetry_connected: true,
  imu_ok: true,
  rate_hz: 400.3,
  samples: 1000000,
  fw_drops: 0,
  crc_errors: 0,
  last_sample_age_ms: 2,
  fw_version: 3,
}

export const defaultHwSyncStatus = {
  connected: true,
  port: '/dev/cu.usbmodem1234',
  armed: true,
  last_pulse_age_ms: 12,
  total_pulses: 123456,
  last_error: null,
}

// A tiny two-tag WPILib layout for field-view tests.
export const testLayoutJson = {
  tags: [
    {
      ID: 7,
      pose: {
        translation: { x: 1.0, y: 2.0, z: 1.4 },
        rotation: { quaternion: { W: 1, X: 0, Y: 0, Z: 0 } },
      },
    },
    {
      ID: 8,
      pose: {
        translation: { x: 15.0, y: 6.0, z: 1.4 },
        rotation: { quaternion: { W: 0, X: 0, Y: 0, Z: 1 } },
      },
    },
  ],
  field: { length: 16.54, width: 8.07 },
}

export const defaultLayouts = [
  {
    id: 1,
    name: 'test-field',
    active: true,
    created_at: 1700000000,
    tag_count: 2,
    field_length_m: 16.54,
    field_width_m: 8.07,
  },
]

export async function json(page: Page, url: string, body: unknown, status = 200) {
  await page.route(url, (route) =>
    route.fulfill({ status, contentType: 'application/json', body: JSON.stringify(body) }),
  )
}

// Install default mocks for every endpoint the shell + dashboard poll, plus
// an empty-ish default for the rest. Register BEFORE page.goto. Tests
// override specific endpoints afterwards (Playwright matches newest-first).
export async function mockAllStatus(page: Page) {
  await json(page, '**/api/status', defaultStatus)
  await json(page, '**/api/fusion/status', defaultFusionStatus)
  await json(page, '**/api/vio/status', defaultVioStatus)
  await json(page, '**/api/apriltag/status', defaultAprilTagStatus)
  await json(page, '**/api/can/status', defaultCanStatus)
  await json(page, '**/api/imu/status', defaultImuStatus)
  await json(page, '**/api/hardware-sync/status', defaultHwSyncStatus)
  await json(page, '**/api/field-layouts', defaultLayouts)
  await json(page, '**/api/field-layouts/1', {
    ...defaultLayouts[0],
    json: JSON.stringify(testLayoutJson),
  })
}
