import { test, expect } from '@playwright/test'
import { json, mockAllStatus } from './helpers'

const cam = (over: Record<string, unknown> = {}) => ({
  id: 1,
  name: 'front',
  serial: '11111111',
  focal_length_mm: 6,
  mode: 'Mode0',
  mode_width: 2048,
  mode_height: 1536,
  mode_max_fps: 30,
  gain_auto: true,
  gain: null,
  exposure_auto: true,
  exposure: null,
  hardware_sync_enabled: false,
  trigger_output_pin: null,
  role: 'apriltag',
  online: true,
  created_at: 1700000000,
  calibrated_at: 1700000100,
  reprojection_error_px: 0.31,
  extrinsics_calibrated_at: null,
  ...over,
})

test.describe('Cameras (mocked)', () => {
  test('list renders role and calibration badges', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/cameras', [
      cam(),
      cam({ id: 2, name: 'left', serial: '22222222', role: 'vio_left', calibrated_at: null, reprojection_error_px: null }),
    ])

    await page.goto('/cameras')
    await expect(page.getByTestId('camera-row-1')).toContainText('front')
    await expect(page.getByTestId('camera-row-1')).toContainText('0.31 px')
    await expect(page.getByTestId('camera-row-1')).toContainText('AprilTag detection')
    await expect(page.getByTestId('camera-row-2')).toContainText('uncalibrated')
    await expect(page.getByTestId('camera-row-2')).toContainText('VIO left')
  })

  test('add-camera dialog posts serial, focal length and mode', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/cameras', [])
    await json(page, '**/api/cameras/available', [
      { serial: '33333333', model: 'BFS-U3-32S4M', vendor: 'FLIR' },
    ])
    await json(page, '**/api/cameras/available/33333333/modes', {
      supported: true,
      current: 'Mode0',
      options: [
        { name: 'Mode0', display_name: 'Full', description: '', width: 2048, height: 1536, max_fps: 30 },
        { name: 'Mode1', display_name: 'Binned', description: '2x2 binning', width: 1024, height: 768, max_fps: 60 },
      ],
    })

    let postBody: Record<string, unknown> | null = null
    await page.route('**/api/cameras', (route) => {
      if (route.request().method() === 'POST') {
        postBody = route.request().postDataJSON()
        return route.fulfill({
          status: 201,
          contentType: 'application/json',
          body: JSON.stringify(cam({ id: 9, name: 'new-cam', serial: '33333333' })),
        })
      }
      return route.fulfill({ status: 200, contentType: 'application/json', body: '[]' })
    })

    await page.goto('/cameras')
    await page.getByTestId('add-camera').click()

    await page.getByLabel('Detected camera').click()
    await page.getByRole('option', { name: /33333333/ }).click()
    await page.getByLabel('Name').fill('new-cam')
    await page.getByLabel('Focal length (mm)').fill('6')
    await page.getByLabel('Camera mode').click()
    await page.getByRole('option', { name: /Binned/ }).click()
    // Hardware sync is configured on the camera detail page, not at creation.
    await expect(page.getByLabel('Hardware sync')).toHaveCount(0)
    await page.getByTestId('add-camera-submit').click()

    await expect.poll(() => postBody).not.toBeNull()
    expect(postBody).toMatchObject({
      name: 'new-cam',
      serial: '33333333',
      focal_length_mm: 6,
      mode: 'Mode1',
    })
    expect(postBody).not.toHaveProperty('hardware_sync_enabled')
    expect(postBody).not.toHaveProperty('trigger_output_pin')
  })

  test('role conflict 409 surfaces as a toast', async ({ page }) => {
    await mockAllStatus(page)
    const offline = cam({ online: false, role: null })
    await json(page, '**/api/cameras', [offline])
    await json(page, '**/api/cameras/1', offline)
    await page.route('**/api/cameras/1', (route) => {
      if (route.request().method() === 'PUT') {
        return route.fulfill({
          status: 409,
          contentType: 'application/json',
          body: JSON.stringify({ error: "role 'vio_left' is already assigned to camera 'left'" }),
        })
      }
      return route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify(offline),
      })
    })

    await page.goto('/cameras/1')
    await page.getByLabel('Camera role').click()
    await page.getByRole('option', { name: 'VIO left' }).click()

    await expect(
      page.getByText("role 'vio_left' is already assigned to camera 'left'"),
    ).toBeVisible()
  })
})
