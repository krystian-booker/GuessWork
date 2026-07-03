import { test, expect } from '@playwright/test'
import { defaultRobotConfig, json, mockAllStatus } from './helpers'

const imuConfig = {
  rate_hz: 400,
  accel_noise_density: 0.0028,
  accel_random_walk: 0.00086,
  gyro_noise_density: 0.00016,
  gyro_random_walk: 0.000022,
  t_imu_robot: null,
  updated_at: 1700000000,
}

test.describe('Robot page (mocked)', () => {
  test('link status, clock sync and pose downlink cards render', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/imu/config', imuConfig)

    await page.goto('/robot')

    const link = page.getByTestId('robot-link-card')
    await expect(link).toContainText('5800')
    await expect(link).toContainText('10.28.52.2:5800')
    await expect(link).toContainText('75 Hz')
    await expect(link).toContainText('40,000 packets · 2 rejected · 1 counter gaps')
    await expect(link).toContainText('vx 1.20 m/s')

    // Both hops of the timestamp chain + the combined chain badge.
    const sync = page.getByTestId('clock-sync-card')
    await expect(sync).toContainText('chain healthy')
    await expect(sync).toContainText('RIO ↔ host')
    await expect(sync).toContainText('Host ↔ Teensy')
    await expect(sync).toContainText('offset 120 µs')
    await expect(sync).toContainText('drift 4.2 ppm')
    await expect(sync).toContainText('offset -80 µs')
    await expect(sync).toContainText('drift 1.1 ppm')
    await expect(sync).toContainText('240 samples')
    await expect(sync).toContainText('1 resets')

    const pose = page.getByTestId('pose-downlink-card')
    await expect(pose).toContainText('Poses sent')
    await expect(pose).toContainText('5,000')
    await expect(pose).toContainText('No destination')
  })

  test('config form PUTs only the changed fields', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/imu/config', imuConfig)

    let putBody: Record<string, unknown> | null = null
    await page.route('**/api/robot/config', (route) => {
      if (route.request().method() === 'PUT') {
        putBody = route.request().postDataJSON()
        return route.fulfill({
          status: 200,
          contentType: 'application/json',
          body: JSON.stringify({
            ...defaultRobotConfig,
            ...putBody,
            updated_at: 1700000001,
            restarted: true,
            restart_error: null,
          }),
        })
      }
      return route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify(defaultRobotConfig),
      })
    })

    await page.goto('/robot')
    await page.getByLabel('Robot IP').fill('10.28.52.2')
    await page.getByLabel('Bind port').fill('5801')
    // The link-config form is the first ConfigForm in the DOM (IMU noise is
    // the other); only the dirty form's Save is enabled anyway.
    await page.getByTestId('config-save').first().click()

    await expect.poll(() => putBody).not.toBeNull()
    expect(putBody).toEqual({ robot_ip: '10.28.52.2', bind_port: 5801 })
    await expect(page.getByText('Saved — robot link restarted')).toBeVisible()
  })

  test('a failed link rebind surfaces restart_error as a toast', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/imu/config', imuConfig)

    await page.route('**/api/robot/config', (route) => {
      if (route.request().method() === 'PUT') {
        return route.fulfill({
          status: 200,
          contentType: 'application/json',
          body: JSON.stringify({
            ...defaultRobotConfig,
            bind_port: 5801,
            updated_at: 1700000001,
            restarted: false,
            restart_error: 'bind failed: address already in use',
          }),
        })
      }
      return route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify(defaultRobotConfig),
      })
    })

    await page.goto('/robot')
    await page.getByLabel('Bind port').fill('5801')
    await page.getByTestId('config-save').first().click()

    await expect(page.getByText(/bind failed: address already in use/)).toBeVisible()
  })

  test('bench pose sender POSTs /api/robot/pose', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/imu/config', imuConfig)

    let postBody: Record<string, number> | null = null
    await page.route('**/api/robot/pose', (route) => {
      postBody = route.request().postDataJSON()
      return route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ sent: true, mode: 'nominal' }),
      })
    })

    await page.goto('/robot')
    await page.getByLabel('x (m)').fill('1.5')
    await page.getByLabel('y (m)').fill('2')
    await page.getByLabel('θ (deg)').fill('90')
    await page.getByRole('button', { name: 'Send pose' }).click()

    await expect.poll(() => postBody).not.toBeNull()
    expect(postBody!.x).toBe(1.5)
    expect(postBody!.y).toBe(2)
    expect(postBody!.theta).toBeCloseTo(Math.PI / 2, 6)
    await expect(page.getByText('Pose sent')).toBeVisible()
  })
})
