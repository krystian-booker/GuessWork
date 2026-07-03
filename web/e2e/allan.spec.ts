import { test, expect } from '@playwright/test'
import { mockAllStatus } from './helpers'

const analysis = {
  file: 'imu-log-1.bin',
  samples: 11_520_000,
  duration_s: 8 * 3600,
  rate_hz: 400,
  suggested: {
    accel_noise_density: 0.0028,
    accel_random_walk: 0.00086,
    gyro_noise_density: 0.00016,
    gyro_random_walk: 0.000022,
  },
  axes: {
    accel_x: { noise_density: 0.0026, random_walk: 0.0008, noise_density_ok: true, random_walk_ok: true, fit_quality: 'good' },
    gyro_z: { noise_density: 0.00015, random_walk: 0.00002, noise_density_ok: true, random_walk_ok: false, fit_quality: 'short data' },
  },
  warnings: ['random-walk fit for gyro_z is below the recommended confidence'],
  analyzed_at: 1700000500,
}

test.describe('Allan variance page (mocked)', () => {
  test('record → analyze → apply flow', async ({ page }) => {
    await mockAllStatus(page)

    // Stateful mock: recording toggles via POST/DELETE, analysis appears
    // after the analyze POST, apply writes the IMU config.
    const state = {
      recording: false,
      samples: 0,
      last: null as typeof analysis | null,
      startBody: null as Record<string, unknown> | null,
      stopped: false,
      applied: false,
    }

    await page.route('**/api/imu/allan/status', (route) => {
      if (state.recording) state.samples += 300
      return route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({
          recording: {
            recording: state.recording,
            file: state.recording ? 'imu-log-1.bin' : null,
            samples: state.samples,
            bytes: state.samples * 32,
            rate_hz: 400,
            remaining_s: state.recording ? 28_000 : 0,
          },
          last_analysis: state.last,
        }),
      })
    })
    await page.route('**/api/imu/allan/recording', (route) => {
      if (route.request().method() === 'POST') {
        state.recording = true
        state.startBody = route.request().postDataJSON()
        return route.fulfill({
          status: 200,
          contentType: 'application/json',
          body: JSON.stringify({ recording: true, file: 'imu-log-1.bin', duration_s: state.startBody!.duration_s }),
        })
      }
      // DELETE — stop early, log kept.
      state.recording = false
      state.stopped = true
      return route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ stopped: true, samples: state.samples, file: 'imu-log-1.bin' }),
      })
    })
    await page.route('**/api/imu/allan/analyze', (route) => {
      state.last = analysis
      return route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify(analysis) })
    })
    await page.route('**/api/imu/allan/apply', (route) => {
      state.applied = true
      return route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({
          rate_hz: 400,
          ...analysis.suggested,
          t_imu_robot: null,
          updated_at: 1700000600,
          restarted: true,
        }),
      })
    })

    await page.goto('/calibration/allan')

    // 1 · Record (default duration: 8 hours).
    await page.getByRole('button', { name: 'Start' }).click()
    await expect.poll(() => state.startBody).not.toBeNull()
    expect(state.startBody).toEqual({ duration_s: 8 * 3600 })
    await expect(page.getByText(/samples/)).toBeVisible()
    await expect(page.getByText(/min remaining/)).toBeVisible()

    // Stop early through the confirm dialog.
    await page.getByRole('button', { name: 'Stop early' }).click()
    await page.getByRole('alertdialog').getByRole('button', { name: 'Stop recording' }).click()
    await expect.poll(() => state.stopped).toBe(true)

    // 2 · Analyze.
    await page.getByRole('button', { name: 'Analyze latest log' }).click()
    await expect(page.getByText('Analysis complete')).toBeVisible()

    // 3 · Results: axis table, warning, suggested values.
    await expect(page.getByRole('row', { name: /accel_x/ })).toContainText('good')
    await expect(page.getByRole('row', { name: /gyro_z/ })).toContainText('short data')
    await expect(page.getByText(/below the recommended confidence/)).toBeVisible()
    await expect(page.getByText('accel N: 2.800e-3')).toBeVisible()
    await expect(page.getByText('gyro K: 2.200e-5')).toBeVisible()

    // Apply through the confirm dialog.
    await page.getByRole('button', { name: 'Apply to IMU config' }).click()
    await page.getByRole('alertdialog').getByRole('button', { name: 'Apply' }).click()
    await expect.poll(() => state.applied).toBe(true)
    await expect(page.getByText('IMU noise parameters updated')).toBeVisible()
  })
})
