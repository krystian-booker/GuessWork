import { test, expect } from '@playwright/test'
import { defaultVioStatus, json, mockAllStatus } from './helpers'

const vioConfig = {
  enabled: true,
  num_pts: 200,
  fast_threshold: 15,
  downsample: false,
  max_reproj_std_px: 0.5,
  auto_reinit: true,
  reinit_min_features: 15,
  reinit_window_frames: 30,
  reinit_max_pos_std_m: 2,
  updated_at: 1700000000,
}

test.describe('VIO page (mocked)', () => {
  test('status, counters and last odometry pose render', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/vio/config', vioConfig)
    await json(page, '**/api/vio/status', {
      ...defaultVioStatus,
      last_pose: {
        t_ns: 123456,
        epoch: 2,
        T_odom_imu: [
          [1, 0, 0, 0.5],
          [0, 1, 0, -0.25],
          [0, 0, 1, 0.05],
          [0, 0, 0, 1],
        ],
      },
    })

    await page.goto('/vio')

    await expect(page.getByText('tracking')).toBeVisible()
    await expect(page.getByRole('row', { name: /vio-left/ })).toContainText('running')
    await expect(page.getByText('imu bus drops: 0')).toBeVisible()

    const pose = page.getByTestId('vio-last-pose')
    await expect(pose).toContainText('x 0.50 m')
    await expect(pose).toContainText('y -0.25 m')
    await expect(pose).toContainText('z 0.05 m')
    await expect(pose).toContainText('epoch 2')
  })

  test('config form PUTs only the changed fields', async ({ page }) => {
    await mockAllStatus(page)

    let putBody: Record<string, unknown> | null = null
    await page.route('**/api/vio/config', (route) => {
      if (route.request().method() === 'PUT') {
        putBody = route.request().postDataJSON()
        return route.fulfill({
          status: 200,
          contentType: 'application/json',
          body: JSON.stringify({
            ...vioConfig,
            ...putBody,
            updated_at: 1700000001,
            restarted: true,
          }),
        })
      }
      return route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify(vioConfig),
      })
    })

    await page.goto('/vio')
    await page.getByLabel('Feature count').fill('250')
    await page.getByTestId('config-save').click()

    await expect.poll(() => putBody).not.toBeNull()
    expect(putBody).toEqual({ num_pts: 250 })
    await expect(page.getByText('Saved — VIO restarted')).toBeVisible()
  })

  test('restart requires confirmation and POSTs /api/vio/restart', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/vio/config', vioConfig)

    let restartHit = false
    await page.route('**/api/vio/restart', (route) => {
      restartHit = true
      return route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ epoch: 3, running: true }),
      })
    })

    await page.goto('/vio')
    await page.getByRole('button', { name: 'Restart' }).click()
    await page.getByRole('alertdialog').getByRole('button', { name: 'Restart' }).click()

    await expect.poll(() => restartHit).toBe(true)
    await expect(page.getByText('VIO restarted (epoch 3)')).toBeVisible()
  })
})
