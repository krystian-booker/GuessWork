import { test, expect } from '@playwright/test'
import { mockAllStatus } from './helpers'

test.describe('Settings (mocked)', () => {
  test('export downloads the config snapshot', async ({ page }) => {
    await mockAllStatus(page)
    await page.route('**/api/config/export', (route) =>
      route.fulfill({
        status: 200,
        contentType: 'application/json',
        headers: { 'Content-Disposition': 'attachment; filename="guesswork-config-20260703.json"' },
        body: JSON.stringify({ cameras: [], field_layouts: [] }),
      }),
    )

    await page.goto('/settings')
    const downloadPromise = page.waitForEvent('download')
    await page.getByTestId('export-config').click()
    const download = await downloadPromise
    expect(download.suggestedFilename()).toBe('guesswork-config-20260703.json')
  })

  test('import posts the snapshot and renders the per-section report', async ({ page }) => {
    await mockAllStatus(page)

    const snapshot = {
      cameras: [{ serial: '11111111', name: 'front' }],
      field_layouts: [{ name: 'test-field' }],
      imu_config: { rate_hz: 400 },
    }
    let postBody: Record<string, unknown> | null = null
    await page.route('**/api/config/import', (route) => {
      postBody = route.request().postDataJSON()
      return route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({
          ok: true,
          report: {
            cameras: { created: 1, updated: 0, errors: [] },
            field_layouts: { created: 0, updated: 1, errors: ['layout "test-field": invalid tag 99'] },
            imu_config: { created: 0, updated: 1, errors: [] },
          },
          note: 'trigger groups stored but not armed',
        }),
      })
    })

    await page.goto('/settings')
    await page.setInputFiles('input[type="file"]', {
      name: 'snapshot.json',
      mimeType: 'application/json',
      buffer: Buffer.from(JSON.stringify(snapshot)),
    })

    // Pre-import preview lists the snapshot's top-level sections.
    await expect(page.getByText('snapshot.json')).toBeVisible()
    await expect(page.getByText('cameras (1)')).toBeVisible()
    await expect(page.getByText('field_layouts (1)')).toBeVisible()

    await page.getByRole('button', { name: 'Import snapshot' }).click()
    await page.getByRole('alertdialog').getByRole('button', { name: 'Import', exact: true }).click()

    await expect.poll(() => postBody).not.toBeNull()
    expect(postBody).toEqual(snapshot)

    const report = page.getByTestId('import-report')
    await expect(report).toBeVisible()
    await expect(report).toContainText('cameras')
    await expect(report).toContainText('1 created, 0 updated')
    await expect(report).toContainText('layout "test-field": invalid tag 99')
    await expect(report).toContainText('trigger groups stored but not armed')
    await expect(page.getByText('Imported with 1 error(s) — see report')).toBeVisible()
  })
})
