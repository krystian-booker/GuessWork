import { test, expect } from '@playwright/test'
import { defaultImuStatus, defaultRobotStatus, defaultStatus, json, mockAllStatus } from './helpers'

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
    await expect(page.getByTestId('stat-controller-value')).toContainText('online')
    await expect(page.getByTestId('stat-vio-value')).toContainText('tracking')

    // 41 ms < 50 ms → good (success green) staleness tone.
    await expect(page.getByTestId('stat-fusion-value')).toHaveClass(/text-success/)

    // No alerts with everything healthy.
    await expect(page.getByTestId('alerts-strip')).toHaveCount(0)
  })

  test('alerts strip surfaces offline camera, disconnected sync controller and dead robot link', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/status', {
      ...defaultStatus,
      cameras: [{ ...defaultStatus.cameras[0], online: false, fps_1s: 0 }],
    })
    await json(page, '**/api/imu/status', { ...defaultImuStatus, controller_connected: false })
    await json(page, '**/api/robot/status', { ...defaultRobotStatus, running: false })

    await page.goto('/')
    const alerts = page.getByTestId('alerts-strip')
    await expect(alerts).toBeVisible()
    await expect(alerts).toContainText('Camera "front" is offline')
    await expect(alerts).toContainText('sync controller is not connected')
    await expect(alerts).toContainText('Robot link is down')
  })
})
