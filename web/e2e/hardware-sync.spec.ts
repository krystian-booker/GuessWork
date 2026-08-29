import { test, expect } from '@playwright/test'
import { json, mockAllStatus } from './helpers'

const groups = [
  { id: 1, name: 'stereo-pair', fps: 30, output_pins: [1, 2], created_at: 1700000000 },
]

test.describe('Hardware sync (mocked)', () => {
  test('status banner and group table render', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/hardware-sync/groups', groups)
    await json(page, '**/api/cameras', [])

    await page.goto('/hardware-sync')
    await expect(page.getByText('sync controller connected')).toBeVisible()
    await expect(page.getByText('Armed')).toBeVisible()
    await expect(page.getByText('123,456 pulses')).toBeVisible()
    await expect(page.getByRole('row', { name: /stereo-pair/ })).toBeVisible()
  })

  test('create dialog posts name, fps and pins; warns on claimed pin', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/cameras', [])

    let postBody: Record<string, unknown> | null = null
    await page.route('**/api/hardware-sync/groups', (route) => {
      if (route.request().method() === 'POST') {
        postBody = route.request().postDataJSON()
        return route.fulfill({
          status: 201,
          contentType: 'application/json',
          body: JSON.stringify({ id: 2, ...postBody, created_at: 1700000001 }),
        })
      }
      return route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify(groups) })
    })

    await page.goto('/hardware-sync')
    await page.getByRole('button', { name: 'New group' }).click()

    await page.getByLabel('Name').fill('apriltag-cams')
    await page.getByLabel('FPS').fill('50')
    // Pin 1 is claimed by stereo-pair → advisory warning.
    await page.getByLabel('Pin 1').click()
    await expect(page.getByText(/already claimed by another group/)).toBeVisible()
    await page.getByLabel('Pin 3').click()
    // Unselect the conflicting pin again.
    await page.getByLabel('Pin 1').click()

    await page.getByRole('button', { name: 'Create' }).click()
    await expect.poll(() => postBody).not.toBeNull()
    expect(postBody).toMatchObject({ name: 'apriltag-cams', fps: 50, output_pins: [3] })
  })

  test('arm posts and stop requires confirmation while armed', async ({ page }) => {
    await mockAllStatus(page)
    await json(page, '**/api/hardware-sync/groups', groups)
    await json(page, '**/api/cameras', [])

    let armed = false
    let stopped = false
    await page.route('**/api/hardware-sync/arm', (route) => {
      armed = true
      return route.fulfill({ status: 200, contentType: 'application/json', body: '{}' })
    })
    await page.route('**/api/hardware-sync/stop', (route) => {
      stopped = true
      return route.fulfill({ status: 200, contentType: 'application/json', body: '{}' })
    })

    await page.goto('/hardware-sync')
    await page.getByRole('button', { name: 'Arm' }).click()
    await expect.poll(() => armed).toBe(true)

    await page.getByRole('button', { name: 'Stop', exact: true }).click()
    await page.getByRole('button', { name: 'Stop outputs' }).click()
    await expect.poll(() => stopped).toBe(true)
  })
})
