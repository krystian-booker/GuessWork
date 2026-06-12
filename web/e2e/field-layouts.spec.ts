import { test, expect } from '@playwright/test'
import { defaultLayouts, json, mockAllStatus, testLayoutJson } from './helpers'

test.describe('Field page (mocked)', () => {
  test('renders tags and the fused robot pose on the SVG field', async ({ page }) => {
    await mockAllStatus(page)
    await page.goto('/field')

    const field = page.getByTestId('field-view')
    await expect(field).toBeVisible()
    await expect(field.locator('[data-tag-id]')).toHaveCount(2)

    // Robot triangle placed at the fused pose (x=4.2, y=2.1, θ=0.5 rad).
    const robot = page.getByTestId('robot-pose')
    await expect(robot).toBeVisible()
    await expect(robot).toHaveAttribute('transform', /translate\(4\.2 2\.1\)/)

    // Pose readout under the field.
    await expect(page.getByText('staleness p95 41 ms')).toBeVisible()
  })

  test('activate posts to the layout endpoint', async ({ page }) => {
    await mockAllStatus(page)
    const two = [
      defaultLayouts[0],
      { ...defaultLayouts[0], id: 2, name: 'practice-field', active: false },
    ]
    await json(page, '**/api/field-layouts', two)
    await json(page, '**/api/field-layouts/2', { ...two[1], json: JSON.stringify(testLayoutJson) })

    let activated = false
    await page.route('**/api/field-layouts/2/activate', (route) => {
      activated = true
      return route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ ...two[1], active: true }),
      })
    })

    await page.goto('/field')
    await page
      .getByTestId('layout-2')
      .getByRole('button', { name: /Activate/ })
      .click()
    await expect.poll(() => activated).toBe(true)
  })

  test('deleting the active layout surfaces the server 409', async ({ page }) => {
    await mockAllStatus(page)
    await page.route('**/api/field-layouts/1', (route) => {
      if (route.request().method() === 'DELETE') {
        return route.fulfill({
          status: 409,
          contentType: 'application/json',
          body: JSON.stringify({ error: 'cannot delete the active layout' }),
        })
      }
      return route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ ...defaultLayouts[0], json: JSON.stringify(testLayoutJson) }),
      })
    })

    await page.goto('/field')
    // Open the delete confirm on the active layout and confirm.
    await page.getByTestId('layout-1').getByRole('button', { name: /Delete/ }).click()
    await page.getByRole('button', { name: 'Delete', exact: true }).click()

    await expect(page.getByText('cannot delete the active layout')).toBeVisible()
  })
})
