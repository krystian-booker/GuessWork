import { test, expect } from '@playwright/test'
import { json, mockAllStatus } from './helpers'

const fusionConfig = {
  enabled: true,
  lag_s: 2,
  min_state_dt_ms: 25,
  output_hz: 50,
  max_extrapolation_ms: 80,
  tag_gate_chi2: 22.5,
  tag_huber_k: 1.345,
  vio_huber_k: 1.345,
  odom_cauchy_k: 0.5,
  odom_sigma_vx: 0.05,
  odom_sigma_vy: 0.05,
  odom_sigma_omega: 0.02,
  vio_sigma_rot: 0.01,
  vio_sigma_trans: 0.005,
  collision_inflation: 10,
  collision_window: 20,
  reinit_pos_std_m: 1,
  updated_at: 1700000000,
}

test.describe('Fusion page (mocked)', () => {
  test('quality, latency stages and source drop counters render', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/fusion/config', fusionConfig)

    await page.goto('/fusion')

    // Headline confidence signal (state.quality, 0-255).
    await expect(page.getByTestId('stat-quality-value')).toHaveText('255')
    await expect(page.getByTestId('stat-quality-value')).toHaveClass(/text-success/)

    await expect(page.getByTestId('stat-staleness-value')).toContainText('41 ms')
    await expect(page.getByText('tag pulse → fusion')).toBeVisible()
    await expect(page.getByText('pose staleness')).toBeVisible()

    // Per-source bus drops column + output queue_dropped counter.
    await expect(page.getByRole('columnheader', { name: 'Bus drops' })).toBeVisible()
    await expect(page.getByText('queue dropped: 0')).toBeVisible()
    await expect(page.getByRole('row', { name: /Tags/ })).toContainText('25.0 Hz')
  })

  test('config form PUTs only the changed fields', async ({ page }) => {
    await mockAllStatus(page)

    let putBody: Record<string, unknown> | null = null
    await page.route('**/api/fusion/config', (route) => {
      if (route.request().method() === 'PUT') {
        putBody = route.request().postDataJSON()
        return route.fulfill({
          status: 200,
          contentType: 'application/json',
          body: JSON.stringify({
            ...fusionConfig,
            ...putBody,
            updated_at: 1700000001,
            restarted: true,
          }),
        })
      }
      return route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify(fusionConfig),
      })
    })

    await page.goto('/fusion')
    await page.getByLabel('Smoother lag').fill('3')
    await page.getByLabel('χ² gate').fill('30')
    await page.getByTestId('config-save').click()

    await expect.poll(() => putBody).not.toBeNull()
    expect(putBody).toEqual({ lag_s: 3, tag_gate_chi2: 30 })
    await expect(page.getByText('Saved — engine rebuilt')).toBeVisible()
  })

  test('reset requires confirmation and POSTs /api/fusion/reset', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/fusion/config', fusionConfig)

    let resetHit = false
    await page.route('**/api/fusion/reset', (route) => {
      resetHit = true
      return route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ reinits: 1 }),
      })
    })

    await page.goto('/fusion')
    await page.getByRole('button', { name: 'Reset' }).click()
    await page.getByRole('alertdialog').getByRole('button', { name: 'Reset' }).click()

    await expect.poll(() => resetHit).toBe(true)
    await expect(page.getByText('Fusion reset')).toBeVisible()
  })
})
