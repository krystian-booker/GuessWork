import { test, expect, type APIRequestContext } from '@playwright/test'

// Hardware suite — requires a physically connected Spinnaker camera and the
// guesswork binary (GW_E2E_HW=1 npx playwright test).

const API = 'http://localhost:8080'

interface Camera {
  id: number
  name: string
  serial: string
  online: boolean
}

interface AvailableCamera {
  serial: string
  model: string
  vendor: string
}

interface CameraStatus {
  id: number
  online: boolean
  frames_produced: number
  fps_1s: number
}

interface StatusResponse {
  ok: boolean
  cameras: CameraStatus[]
}

// Reset the cameras table to a known state and ensure exactly one camera is
// mapped to a physically-connected Spinnaker device.
async function seedSingleCamera(request: APIRequestContext): Promise<Camera> {
  const existing = (await request.get(`${API}/api/cameras`).then((r) => r.json())) as Camera[]
  for (const c of existing) {
    const del = await request.delete(`${API}/api/cameras/${c.id}`)
    expect(del.ok(), `cleanup DELETE /api/cameras/${c.id}`).toBeTruthy()
  }

  const available = (await request.get(`${API}/api/cameras/available`).then((r) =>
    r.json(),
  )) as AvailableCamera[]
  expect(available.length, 'at least one Spinnaker camera must be physically connected').toBeGreaterThan(0)

  const pick = available[0]
  const created = await request.post(`${API}/api/cameras`, {
    headers: { 'Content-Type': 'application/json' },
    data: { name: `e2e-${pick.serial}`, serial: pick.serial, focal_length_mm: 6 },
  })
  expect(created.ok(), `POST /api/cameras (status ${created.status()})`).toBeTruthy()
  return (await created.json()) as Camera
}

async function waitOnline(request: APIRequestContext, id: number) {
  await expect
    .poll(async () => {
      const s = (await request.get(`${API}/api/status`).then((r) => r.json())) as StatusResponse
      return s.cameras.find((c) => c.id === id)?.online ?? false
    }, { timeout: 20_000, intervals: [500, 1000] })
    .toBe(true)
}

test.describe('WebRTC streaming', () => {
  let seeded: Camera

  test.beforeAll(async ({ request }) => {
    seeded = await seedSingleCamera(request)
    await waitOnline(request, seeded.id)
  })

  test.afterAll(async ({ request }) => {
    if (seeded) await request.delete(`${API}/api/cameras/${seeded.id}`)
  })

  test('camera detail page streams H.264 to the video element', async ({ page }) => {
    const consoleErrors: string[] = []
    page.on('pageerror', (e) => consoleErrors.push(`pageerror: ${e.message}`))
    page.on('console', (msg) => {
      if (msg.type() === 'error') consoleErrors.push(`console: ${msg.text()}`)
    })

    await page.goto(`/cameras/${seeded.id}`)

    const player = page.getByTestId('webrtc-player')
    await expect(player).toBeVisible({ timeout: 10_000 })
    await expect(player).toHaveAttribute('data-stream-state', 'streaming', { timeout: 20_000 })

    const video = player.locator('video')

    // videoWidth flips to non-zero only after the first IDR is decoded.
    await expect
      .poll(async () => video.evaluate((el: HTMLVideoElement) => el.videoWidth), {
        timeout: 25_000,
        intervals: [500, 1000, 2000],
      })
      .toBeGreaterThan(0)

    const stats = await video.evaluate((el: HTMLVideoElement) => ({
      readyState: el.readyState,
      videoWidth: el.videoWidth,
      videoHeight: el.videoHeight,
      paused: el.paused,
    }))
    expect(stats.readyState).toBeGreaterThanOrEqual(2)
    expect(stats.videoWidth).toBeGreaterThan(0)
    expect(stats.paused).toBe(false)

    // Liveness: painted frames over ~2 s at a 30 fps target.
    const framesIn2s = await video.evaluate((el: HTMLVideoElement) => {
      return new Promise<number>((resolve) => {
        let count = 0
        const start = performance.now()
        const tick = () => {
          count++
          if (performance.now() - start > 2000) resolve(count)
          else el.requestVideoFrameCallback(tick)
        }
        el.requestVideoFrameCallback(tick)
        setTimeout(() => resolve(count), 5000)
      })
    })
    expect(framesIn2s).toBeGreaterThan(20)

    // RTCPeerConnection introspection via the WebRtcPlayer's window.__pc hook.
    const inbound = await page.evaluate(async () => {
      const pc = (window as unknown as { __pc?: RTCPeerConnection }).__pc
      if (!pc) return null
      const report = await pc.getStats()
      let hit: Record<string, unknown> | null = null
      report.forEach((s) => {
        if (s.type === 'inbound-rtp' && s.kind === 'video') hit = s as unknown as Record<string, unknown>
      })
      return hit
    })
    expect(inbound, 'expected an inbound-rtp video stat').not.toBeNull()
    const inboundStats = inbound as unknown as Record<string, number>
    expect(inboundStats.bytesReceived).toBeGreaterThan(10_000)
    expect(inboundStats.framesDecoded).toBeGreaterThan(10)

    expect(consoleErrors, consoleErrors.join('\n')).toEqual([])
  })

  test('status endpoint reports producer fps stable while streaming', async ({ request }) => {
    const fetchCam = async (): Promise<CameraStatus> => {
      const s = (await request.get(`${API}/api/status`).then((r) => r.json())) as StatusResponse
      const cam = s.cameras.find((c) => c.id === seeded.id)
      expect(cam, `camera ${seeded.id} missing from /api/status`).toBeDefined()
      return cam!
    }

    const s1 = await fetchCam()
    await new Promise((res) => setTimeout(res, 2000))
    const s2 = await fetchCam()

    expect(s1.online).toBe(true)
    expect(s2.online).toBe(true)
    expect(s2.frames_produced - s1.frames_produced).toBeGreaterThan(50)
    expect(s2.fps_1s).toBeGreaterThan(20)
  })
})

// Regression test for a SIGTRAP-on-re-add heap corruption: after streaming,
// FrameChannel destruction released a Frame whose recycle callback pointed
// into an already-freed SpinnakerUserBufferPool. Driving the same UI sequence
// (add → stream → navigate away → delete → re-add) exercises that exact
// destruction order.
test.describe('Camera lifecycle regression', () => {
  test('add → stream → delete → re-add does not corrupt the heap', async ({ page, request }) => {
    const existing = (await request.get(`${API}/api/cameras`).then((r) => r.json())) as Camera[]
    for (const c of existing) await request.delete(`${API}/api/cameras/${c.id}`)

    const available = (await request.get(`${API}/api/cameras/available`).then((r) =>
      r.json(),
    )) as AvailableCamera[]
    expect(available.length, 'at least one Spinnaker camera must be physically connected').toBeGreaterThan(0)
    const serial = available[0].serial

    const cycle = async (round: number) => {
      const created = await request.post(`${API}/api/cameras`, {
        headers: { 'Content-Type': 'application/json' },
        data: { name: `lifecycle-${round}-${serial}`, serial, focal_length_mm: 6 },
      })
      expect(created.ok(), `POST round ${round} (status ${created.status()})`).toBeTruthy()
      const cam = (await created.json()) as Camera

      await waitOnline(request, cam.id)

      // Drive a real WebRTC peer through the UI so the StreamConsumer adds a
      // peer to its broadcast set — without that, the original bug doesn't
      // reproduce.
      await page.goto(`/cameras/${cam.id}`)
      const player = page.getByTestId('webrtc-player')
      await expect(player).toHaveAttribute('data-stream-state', 'streaming', { timeout: 20_000 })
      const video = player.locator('video')
      await expect
        .poll(async () => video.evaluate((el: HTMLVideoElement) => el.videoWidth), {
          timeout: 25_000,
          intervals: [250, 500, 1000],
        })
        .toBeGreaterThan(0)

      // Navigating away triggers the player cleanup → pc.close() → the
      // server's WebRtcPeer enters the destruction path that used to corrupt
      // the heap when the camera was deleted right after.
      await page.goto('/cameras')

      const del = await request.delete(`${API}/api/cameras/${cam.id}`)
      expect(del.ok(), `DELETE round ${round} (status ${del.status()})`).toBeTruthy()
    }

    await cycle(1)
    await cycle(2)

    const status = (await request.get(`${API}/api/status`).then((r) => r.json())) as StatusResponse
    expect(status.ok).toBe(true)
  })
})
