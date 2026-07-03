import { test, expect } from '@playwright/test'
import { defaultAprilTagStatus, json, mockAllStatus } from './helpers'

test.describe('AprilTag page (mocked)', () => {
  test('gate badges and per-camera diagnostics render', async ({ page }) => {
    await mockAllStatus(page)
    await page.goto('/apriltag')

    await expect(page.getByTestId('gate-layout-value')).toHaveText('test-field')
    await expect(page.getByTestId('gate-t-robot-imu-value')).toHaveText('set')
    await expect(page.getByTestId('gate-detectors-value')).toHaveText('1/1')

    const cam = page.getByTestId('apriltag-cam-1')
    await expect(cam).toContainText('front')
    await expect(cam).toContainText('publishing')
    await expect(cam).toContainText('22.4 /s')
    await expect(cam).toContainText('12.5 ms')
    await expect(cam).toContainText('ewma 12.1 ms')
    await expect(cam).toContainText('0.42 px')
    await expect(cam).toContainText('29,000')
    // Last tags with range and decision margin.
    await expect(cam).toContainText('#7 · 2.4 m · dm 60')
    await expect(cam).toContainText('#8 · 3.1 m · dm 55')
    await expect(cam).toContainText('no pose published yet')
  })

  test('skip-reason breakdown highlights the dominant nonzero reason', async ({ page }) => {
    await mockAllStatus(page)
    await page.goto('/apriltag')

    const noTags = page.getByTestId('skip-skipped_no_tags')
    await expect(noTags).toContainText('No tags in frame')
    await expect(noTags).toContainText('900')
    await expect(noTags.locator('.text-warning').first()).toBeVisible()

    // A non-dominant reason is rendered but not highlighted.
    const ambiguous = page.getByTestId('skip-skipped_ambiguous')
    await expect(ambiguous).toContainText('Ambiguous single tag')
    await expect(ambiguous).toContainText('50')
    await expect(ambiguous.locator('.text-warning')).toHaveCount(0)
  })

  test('gate badges go red without layout / T_robot_imu; last pose renders', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/apriltag/status', {
      t_robot_imu_set: false,
      cameras: [
        {
          ...defaultAprilTagStatus.cameras[0],
          reason: 'no_extrinsics_chain',
          last_pose: [
            [1, 0, 0, 3.5],
            [0, 1, 0, 1.25],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
          ],
          last_pose_t_ns: 424242,
        },
      ],
    })
    await page.goto('/apriltag')

    await expect(page.getByTestId('gate-layout-value')).toHaveText('none')
    await expect(page.getByTestId('gate-t-robot-imu-value')).toHaveText('not set')
    await expect(page.getByText('set on the Robot page →')).toBeVisible()

    const cam = page.getByTestId('apriltag-cam-1')
    await expect(cam).toContainText('no_extrinsics_chain')
    const pose = page.getByTestId('apriltag-pose-1')
    await expect(pose).toContainText('x 3.50 m')
    await expect(pose).toContainText('y 1.25 m')
    await expect(pose).toContainText('θ 0.0°')
  })
})
