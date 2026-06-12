import { test, expect } from '@playwright/test'
import { defaultCanStatus, defaultStatus, json, mockAllStatus } from './helpers'

test.describe('Dashboard (mocked)', () => {
  test('stat cards and header health render from status endpoints', async ({ page }) => {
    await mockAllStatus(page)
    await page.goto('/')

    // Header health cluster.
    await expect(page.getByTestId('header-cameras')).toContainText('1/1')
    await expect(page.getByTestId('header-fusion-mode')).toContainText('tags+vio+odom')

    // Stat cards.
    await expect(page.getByTestId('stat-cameras-value')).toContainText('1/1')
    await expect(page.getByTestId('stat-fusion-value')).toContainText('41 ms')
    await expect(page.getByTestId('stat-teensy-value')).toContainText('online')
    await expect(page.getByTestId('stat-vio-value')).toContainText('tracking')

    // 41 ms < 50 ms → good (primary green) staleness tone.
    await expect(page.getByTestId('stat-fusion-value')).toHaveClass(/text-primary/)

    // No alerts with everything healthy.
    await expect(page.getByTestId('alerts-strip')).toHaveCount(0)
  })

  test('alerts strip surfaces offline camera and disconnected Teensy', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/status', {
      ...defaultStatus,
      cameras: [{ ...defaultStatus.cameras[0], online: false, fps_1s: 0 }],
    })
    await json(page, '**/api/can/status', { ...defaultCanStatus, teensy_connected: false })

    await page.goto('/')
    const alerts = page.getByTestId('alerts-strip')
    await expect(alerts).toBeVisible()
    await expect(alerts).toContainText('Camera "front" is offline')
    await expect(alerts).toContainText('Teensy is not connected')
  })
})
