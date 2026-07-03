import { test, expect, type Page } from '@playwright/test'
import { defaultHwSyncStatus, defaultImuStatus, json, mockAllStatus } from './helpers'

const vioCam = (id: number, name: string, role: string, over: Record<string, unknown> = {}) => ({
  id,
  name,
  serial: `${id}${id}${id}${id}${id}${id}${id}${id}`,
  focal_length_mm: 6,
  mode: null,
  mode_width: null,
  mode_height: null,
  mode_max_fps: null,
  gain_auto: null,
  gain: null,
  exposure_auto: null,
  exposure: null,
  hardware_sync_enabled: true,
  trigger_output_pin: id,
  role,
  online: true,
  created_at: 1700000000,
  calibrated_at: 1700000100,
  reprojection_error_px: 0.31,
  extrinsics_calibrated_at: null,
  orientation: 0,
  ...over,
})

async function mockCamerasAndExtrinsics(page: Page, cams: Record<string, unknown>[]) {
  await json(page, '**/api/cameras', cams)
  for (const c of cams) {
    await page.route(`**/api/cameras/${c.id}/extrinsics`, (route) =>
      route.fulfill({ status: 404, contentType: 'application/json', body: '{"error":"no extrinsics"}' }),
    )
  }
  await json(page, '**/api/calibration/extrinsics/job', { error: 'no job' }, 404)
}

// StatusDot tone assertion: the row's dot is bg-success (good) or
// bg-destructive (bad).
function preflightDot(page: Page, label: string) {
  return page.getByText(label, { exact: true }).locator('..')
}

test.describe('Camera-IMU extrinsics (mocked)', () => {
  test('preflight gates reflect IMU, hw-sync arm and per-camera sync state', async ({ page }) => {
    await mockAllStatus(page)
    await mockCamerasAndExtrinsics(page, [
      vioCam(2, 'left', 'vio_left'),
      vioCam(3, 'right', 'vio_right'),
    ])
    await json(page, '**/api/calibration/extrinsics/recording', { error: 'none' }, 404)

    await page.goto('/calibration/extrinsics')
    await expect(preflightDot(page, 'IMU healthy').locator('span.bg-success')).toHaveCount(1)
    await expect(preflightDot(page, 'Hardware sync armed').locator('span.bg-success')).toHaveCount(1)
    await expect(
      preflightDot(page, 'Selected cameras have hw-sync enabled').locator('span.bg-success'),
    ).toHaveCount(1)
  })

  test('preflight gates go red when IMU/arm/sync are missing', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/imu/status', { ...defaultImuStatus, imu_ok: false })
    await json(page, '**/api/hardware-sync/status', { ...defaultHwSyncStatus, armed: false })
    await mockCamerasAndExtrinsics(page, [
      vioCam(2, 'left', 'vio_left', { hardware_sync_enabled: false }),
      vioCam(3, 'right', 'vio_right'),
    ])
    await json(page, '**/api/calibration/extrinsics/recording', { error: 'none' }, 404)

    await page.goto('/calibration/extrinsics')
    await expect(preflightDot(page, 'IMU healthy').locator('span.bg-destructive')).toHaveCount(1)
    await expect(preflightDot(page, 'Hardware sync armed').locator('span.bg-destructive')).toHaveCount(1)
    await expect(
      preflightDot(page, 'Selected cameras have hw-sync enabled').locator('span.bg-destructive'),
    ).toHaveCount(1)
  })

  test('start records the VIO pair; 409 stop surfaces job_error and keeps the dataset UI', async ({ page }) => {
    await mockAllStatus(page)
    await mockCamerasAndExtrinsics(page, [
      vioCam(2, 'left', 'vio_left'),
      vioCam(3, 'right', 'vio_right'),
    ])

    const state = {
      recording: null as Record<string, unknown> | null,
      startBody: null as Record<string, unknown> | null,
    }
    const recordingStatus = () => ({
      session_id: 'ext-1',
      path: '/tmp/ext-1',
      cameras: [
        { camera_id: 2, topic: '/cam0/image_raw', frames_written: 90, frames_dropped: 0 },
        { camera_id: 3, topic: '/cam1/image_raw', frames_written: 90, frames_dropped: 0 },
      ],
      imu_written: 1200,
      imu_dropped: 0,
      elapsed_ms: 3000,
    })
    await page.route('**/api/calibration/extrinsics/recording', (route) => {
      const method = route.request().method()
      if (method === 'POST') {
        state.startBody = route.request().postDataJSON()
        state.recording = recordingStatus()
        return route.fulfill({ status: 201, contentType: 'application/json', body: JSON.stringify(state.recording) })
      }
      if (method === 'DELETE') {
        // Recording stopped but the Kalibr launch was rejected: 409 carries
        // the kept dataset + job_error (parsed by asJsonSoft, not thrown).
        state.recording = null
        return route.fulfill({
          status: 409,
          contentType: 'application/json',
          body: JSON.stringify({
            recording_result: { ...recordingStatus(), model: 'pinhole-radtan', suggested_command: 'docker run …' },
            job: null,
            job_error: 'another Kalibr job is already running',
          }),
        })
      }
      // GET
      if (!state.recording) {
        return route.fulfill({ status: 404, contentType: 'application/json', body: '{"error":"no recording"}' })
      }
      return route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify(state.recording) })
    })

    await page.goto('/calibration/extrinsics')

    // The VIO pair is pre-checked — start records cam ids in cam-index order.
    await page.getByRole('button', { name: 'Start recording' }).click()
    await expect.poll(() => state.startBody).not.toBeNull()
    expect(state.startBody).toEqual({ camera_ids: [2, 3] })
    await expect(page.getByText('/cam0/image_raw: 90 frames')).toBeVisible()
    await expect(page.getByText('1,200 IMU samples')).toBeVisible()

    await page.getByRole('button', { name: 'Stop & calibrate' }).click()

    // job_error toast, and the page returns to a recordable state — no
    // generic error path, dataset kept server-side.
    await expect(page.getByText('another Kalibr job is already running')).toBeVisible()
    await expect(page.getByRole('button', { name: 'Start recording' })).toBeEnabled()
  })
})
