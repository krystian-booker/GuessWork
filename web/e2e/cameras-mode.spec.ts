import { test, expect, type Page } from '@playwright/test'

// All /api/cameras* endpoints are mocked at the browser level. The Vite dev
// server still proxies un-intercepted /api/* to :8080, but page.route() catches
// the request before it leaves the browser — so these tests don't depend on a
// physically-connected Spinnaker camera and run regardless of build-fresh
// state.

interface CamRow {
  id: number
  name: string
  serial: string
  mode: string | null
  mode_width: number | null
  mode_height: number | null
  mode_max_fps: number | null
  online: boolean
  created_at: number
}

interface MockState {
  cameras: CamRow[]
  modesById: Record<number, { supported: boolean; current: string | null; options: ModeOpt[] } | 'offline'>
  modesBySerial: Record<string, { supported: boolean; current: string | null; options: ModeOpt[] }>
  // Captured request bodies for assertions.
  posts: unknown[]
  puts: Array<{ id: number; body: unknown }>
}

interface ModeOpt {
  name: string
  display_name: string
  description: string
  width: number | null
  height: number | null
  max_fps: number | null
}

const SERIAL = 'SN-MOCK-001'

const DEFAULT_MODES: ModeOpt[] = [
  { name: 'Mode0', display_name: 'Mode 0', description: 'Full resolution, no binning.',
    width: 1280, height: 1024, max_fps: 30 },
  { name: 'Mode1', display_name: 'Mode 1', description: '2x2 binning, quarter resolution, higher frame rate.',
    width: 640, height: 512, max_fps: 60 },
  { name: 'Mode5', display_name: 'Mode 5', description: 'Faster binned readout.',
    width: 640, height: 512, max_fps: 120 },
]

function freshState(): MockState {
  return {
    cameras: [],
    modesById: {},
    modesBySerial: {
      [SERIAL]: { supported: true, current: 'Mode0', options: DEFAULT_MODES },
    },
    posts: [],
    puts: [],
  }
}

async function installMocks(page: Page, state: MockState) {
  await page.route('**/api/cameras', async (route) => {
    const req = route.request()
    if (req.method() === 'GET') {
      return route.fulfill({ json: state.cameras })
    }
    if (req.method() === 'POST') {
      const body = JSON.parse(req.postData() ?? '{}')
      state.posts.push(body)
      const id = state.cameras.length + 1
      const row: CamRow = {
        id,
        name: body.name,
        serial: body.serial,
        mode: body.mode ?? null,
        mode_width: null,
        mode_height: null,
        mode_max_fps: null,
        online: true,
        created_at: Math.floor(Date.now() / 1000),
      }
      state.cameras.push(row)
      // Seed modes-by-id so the Edit modal has data to read.
      state.modesById[id] = {
        supported: true,
        current: row.mode ?? 'Mode0',
        options: DEFAULT_MODES,
      }
      return route.fulfill({ status: 201, json: row })
    }
    return route.fallback()
  })

  await page.route('**/api/cameras/available', async (route) => {
    return route.fulfill({
      json: [{ serial: SERIAL, model: 'MockCam', vendor: 'MockCo' }],
    })
  })

  await page.route('**/api/cameras/available/*/modes', async (route) => {
    const url = new URL(route.request().url())
    const segments = url.pathname.split('/')
    // /api/cameras/available/<serial>/modes
    const serial = decodeURIComponent(segments[segments.length - 2])
    const entry = state.modesBySerial[serial]
    if (!entry) return route.fulfill({ status: 404, json: { error: 'serial not connected' } })
    return route.fulfill({ json: entry })
  })

  await page.route('**/api/cameras/*/modes', async (route) => {
    const url = new URL(route.request().url())
    const segments = url.pathname.split('/')
    // /api/cameras/<id>/modes
    const id = Number(segments[segments.length - 2])
    const entry = state.modesById[id]
    if (!entry) return route.fulfill({ status: 404, json: { error: 'camera not found' } })
    if (entry === 'offline') {
      return route.fulfill({ status: 409, json: { error: 'camera is offline' } })
    }
    return route.fulfill({ json: entry })
  })

  await page.route('**/api/cameras/*', async (route) => {
    const req = route.request()
    const url = new URL(req.url())
    const id = Number(url.pathname.split('/').pop())
    const idx = state.cameras.findIndex((c) => c.id === id)
    if (req.method() === 'PUT') {
      const body = JSON.parse(req.postData() ?? '{}')
      state.puts.push({ id, body })
      if (idx < 0) return route.fulfill({ status: 404, json: { error: 'camera not found' } })
      const cur = state.cameras[idx]
      const next: CamRow = {
        ...cur,
        name: body.name ?? cur.name,
        mode: 'mode' in body ? body.mode : cur.mode,
      }
      state.cameras[idx] = next
      return route.fulfill({ json: next })
    }
    if (req.method() === 'GET') {
      if (idx < 0) return route.fulfill({ status: 404, json: { error: 'camera not found' } })
      return route.fulfill({ json: state.cameras[idx] })
    }
    if (req.method() === 'DELETE') {
      if (idx >= 0) state.cameras.splice(idx, 1)
      return route.fulfill({ status: 204, body: '' })
    }
    return route.fallback()
  })
}

test.describe('Cameras page — mode selection', () => {
  test('Add modal: mode dropdown populates and POST body includes mode', async ({ page }) => {
    const state = freshState()
    await installMocks(page, state)

    await page.goto('/cameras')
    await page.getByRole('button', { name: 'Add camera' }).click()

    // Mode dropdown shows entries with the default selected (Mode0).
    const modeSelect = page.getByLabel('Camera mode')
    await expect(modeSelect).toBeVisible()
    await expect(modeSelect).toHaveValue('Mode0')
    await expect(page.getByText('Full resolution, no binning.')).toBeVisible()

    // Switch to Mode1; description updates.
    await modeSelect.selectOption('Mode1')
    await expect(page.getByText(/2x2 binning/)).toBeVisible()

    // Fill name and submit.
    await page.getByLabel('New camera name').fill('front-left')
    await page.getByRole('button', { name: 'Add' }).click()

    await expect.poll(() => state.posts.length).toBe(1)
    expect(state.posts[0]).toEqual({
      name: 'front-left',
      serial: SERIAL,
      mode: 'Mode1',
    })
    // Row shows up with Mode1 in the table.
    await expect(page.getByRole('cell', { name: 'Mode1' })).toBeVisible()
  })

  test('Edit modal (online): pre-selects mode and PUT contains only changed mode', async ({ page }) => {
    const state = freshState()
    state.cameras = [
      { id: 1, name: 'front', serial: SERIAL, mode: 'Mode0',
        mode_width: 1280, mode_height: 1024, mode_max_fps: 30,
        online: true, created_at: 0 },
    ]
    state.modesById[1] = { supported: true, current: 'Mode0', options: DEFAULT_MODES }
    await installMocks(page, state)

    await page.goto('/cameras')
    await page.getByRole('button', { name: 'Edit' }).click()

    const modeSelect = page.getByLabel('Camera mode')
    await expect(modeSelect).toHaveValue('Mode0')

    await modeSelect.selectOption('Mode5')
    await page.getByRole('button', { name: 'Save' }).click()

    await expect.poll(() => state.puts.length).toBe(1)
    expect(state.puts[0]).toEqual({ id: 1, body: { mode: 'Mode5' } })
  })

  test('Edit modal (offline): dropdown disabled, hint visible, PUT carries only name', async ({ page }) => {
    const state = freshState()
    state.cameras = [
      { id: 1, name: 'front', serial: SERIAL, mode: 'Mode0',
        mode_width: null, mode_height: null, mode_max_fps: null,
        online: false, created_at: 0 },
    ]
    state.modesById[1] = 'offline'
    await installMocks(page, state)

    await page.goto('/cameras')
    await page.getByRole('button', { name: 'Edit' }).click()

    await expect(page.getByText(/Camera must be online to change Mode\./).first()).toBeVisible()

    // Change only the name.
    const nameInput = page.getByLabel(/Edit name for camera 1/)
    await nameInput.fill('front-renamed')
    await page.getByRole('button', { name: 'Save' }).click()

    await expect.poll(() => state.puts.length).toBe(1)
    expect(state.puts[0]).toEqual({ id: 1, body: { name: 'front-renamed' } })
  })

  test('Edit modal (no VideoMode node): hint shown, dropdown hidden', async ({ page }) => {
    const state = freshState()
    state.cameras = [
      { id: 1, name: 'front', serial: SERIAL, mode: null,
        mode_width: null, mode_height: null, mode_max_fps: null,
        online: true, created_at: 0 },
    ]
    state.modesById[1] = { supported: false, current: null, options: [] }
    await installMocks(page, state)

    await page.goto('/cameras')
    await page.getByRole('button', { name: 'Edit' }).click()

    await expect(page.getByText(/doesn't expose a Mode setting/)).toBeVisible()
    await expect(page.getByLabel('Camera mode')).toHaveCount(0)
  })
})
