import { test, expect, type Page } from '@playwright/test'
import { json, mockAllStatus } from './helpers'

// Offline camera: the wizard skips the live preview, so no WebRTC mocking.
const camera = {
  id: 9,
  name: 'cal-cam',
  serial: '99999999',
  focal_length_mm: 6,
  mode: null,
  mode_width: null,
  mode_height: null,
  mode_max_fps: null,
  gain_auto: null,
  gain: null,
  exposure_auto: null,
  exposure: null,
  hardware_sync_enabled: false,
  trigger_output_pin: null,
  role: null,
  online: true,
  created_at: 1700000000,
  calibrated_at: null,
  reprojection_error_px: null,
  extrinsics_calibrated_at: null,
  orientation: 0,
}

const CAMCHAIN = [
  'cam0:',
  '  camera_model: pinhole',
  '  distortion_model: radtan',
  '  intrinsics: [1100.5, 1101.2, 1024.0, 768.0]',
  '  distortion_coeffs: [-0.17, 0.05, 0.0001, 0.0002]',
  '  resolution: [2048, 1536]',
  'guesswork_meta:',
  '  reprojection_error_px: 0.21',
].join('\n')

// Scripted server: recording + job state advance as the test drives the UI.
async function installWizardMocks(page: Page, opts: { jobSucceeds: boolean }) {
  await mockAllStatus(page)
  await json(page, '**/api/cameras', [camera])
  await json(page, '**/api/cameras/9', camera)
  await json(page, '**/api/cameras/9/modes', { supported: false, current: null, options: [] })
  await json(page, '**/api/cameras/9/settings/limits', { gain: null, exposure: null })

  const state = {
    recording: null as Record<string, unknown> | null,
    job: null as Record<string, unknown> | null,
    calibration: null as Record<string, unknown> | null,
    // The job must stay 'running' until the client has actually processed the
    // SSE done event — flipping it the moment the SSE body is served lets the
    // job poll close the EventSource before Chromium dispatches the buffered
    // events (a race the real server can't hit: its stream stays open until
    // the job is terminal). The done handler invalidates the calibration
    // query, so its refetch is our "done processed" signal; a few job polls
    // after the SSE serve act as the fallback (mirrors a dropped stream —
    // the wizard handles that via its poll-transition invalidation).
    sseServed: false,
    jobPollsSinceSse: 0,
  }
  const finishJob = () => {
    if (!state.sseServed || !state.job || state.job.state !== 'running') return
    state.job = {
      ...state.job,
      state: opts.jobSucceeds ? 'succeeded' : 'failed',
      exit_code: opts.jobSucceeds ? 0 : 1,
      calibration_stored: opts.jobSucceeds,
    }
    if (opts.jobSucceeds) {
      state.calibration = { camera_id: 9, calibrated_at: 1700001000, calibration: CAMCHAIN }
    }
  }

  await page.route('**/api/cameras/9/calibration/recording', (route) => {
    const method = route.request().method()
    if (method === 'POST') {
      state.recording = {
        session_id: 'sess-1',
        path: '/tmp/sess-1',
        frames_written: 0,
        frames_dropped: 0,
        elapsed_ms: 0,
      }
      return route.fulfill({ status: 201, contentType: 'application/json', body: JSON.stringify(state.recording) })
    }
    if (method === 'DELETE') {
      state.recording = null
      state.job = {
        state: 'running',
        model: 'pinhole-radtan',
        exit_code: 0,
        started_at_ms: 1,
        ended_at_ms: 0,
        log_bytes: 0,
        calibration_stored: false,
        upload_error: null,
      }
      return route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({
          recording_result: { session_id: 'sess-1', path: '/tmp/sess-1', frames_written: 420, frames_dropped: 0, elapsed_ms: 30000, suggested_command: 'docker run …' },
          job: state.job,
        }),
      })
    }
    // GET
    if (!state.recording) return route.fulfill({ status: 404, contentType: 'application/json', body: '{"error":"no recording"}' })
    state.recording.frames_written = (state.recording.frames_written as number) + 30
    state.recording.elapsed_ms = (state.recording.elapsed_ms as number) + 750
    return route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify(state.recording) })
  })

  await page.route('**/api/cameras/9/calibration/job', (route) => {
    if (state.sseServed && ++state.jobPollsSinceSse >= 3) finishJob()
    if (!state.job) return route.fulfill({ status: 404, contentType: 'application/json', body: '{"error":"no job"}' })
    return route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify(state.job) })
  })

  await page.route('**/api/cameras/9/calibration/job/log', (route) => {
    const done = opts.jobSucceeds
      ? '{"state":"succeeded","exit_code":0,"calibration_stored":true,"upload_error":null}'
      : '{"state":"failed","exit_code":1,"calibration_stored":false,"upload_error":null}'
    state.sseServed = true
    // route.fulfill closes the connection after the body; EventSource then
    // auto-reconnects and re-receives the same events. retry: 100 keeps that
    // loop fast in case Chromium drops the first instantly-closed delivery.
    return route.fulfill({
      status: 200,
      contentType: 'text/event-stream',
      body: [
        'retry: 100',
        '',
        'data: Extracting calibration target corners',
        '',
        'data: Optimizing...',
        '',
        `event: done`,
        `data: ${done}`,
        '',
        '',
      ].join('\n'),
    })
  })

  await page.route('**/api/cameras/9/calibration', (route) => {
    finishJob() // refetch after the SSE = the client processed `done`
    if (!state.calibration)
      return route.fulfill({ status: 404, contentType: 'application/json', body: '{"error":"camera is not calibrated"}' })
    return route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify(state.calibration) })
  })
}

test.describe('Intrinsics wizard (mocked)', () => {
  test('record → Kalibr log → stored calibration', async ({ page }) => {
    await installWizardMocks(page, { jobSucceeds: true })
    await page.goto('/calibration/intrinsics/9')

    await expect(page.getByText('No calibration uploaded yet.')).toBeVisible()

    await page.getByTestId('start-recording').click()
    await expect(page.getByTestId('stop-recording')).toBeVisible()
    await expect(page.getByText(/frames/)).toBeVisible()

    await page.getByTestId('stop-recording').click()

    // SSE done (or the poll fallback) → calibration card shows the parsed
    // camchain. The log content itself is timing-dependent with a mocked
    // (instantly-closed) SSE stream, so the workflow outcome is what's
    // asserted; live log streaming is covered by real Kalibr runs.
    await expect(page.getByText('Good — 0.210 px reprojection σ')).toBeVisible()
    await expect(page.getByText('pinhole / radtan')).toBeVisible()
    await expect(page.getByText('2048 × 1536')).toBeVisible()
  })

  test('failed job keeps the dataset note and allows retry', async ({ page }) => {
    await installWizardMocks(page, { jobSucceeds: false })
    await page.goto('/calibration/intrinsics/9')

    await page.getByTestId('start-recording').click()
    await page.getByTestId('stop-recording').click()

    await expect(page.getByText(/Exit code 1/)).toBeVisible()
    await expect(page.getByText(/dataset is kept/)).toBeVisible()
    // Wizard returns to step 1 — recording can be restarted.
    await expect(page.getByTestId('start-recording')).toBeEnabled()
  })
})
