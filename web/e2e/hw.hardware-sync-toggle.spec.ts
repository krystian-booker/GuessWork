import { test, expect, type APIRequestContext } from '@playwright/test'

// Hardware suite — requires a physically connected Spinnaker camera and the
// guesswork binary (GW_E2E_HW=1 npx playwright test). No sync controller required:
// with hw-sync on and no trigger pulses the camera legitimately produces no
// frames; what this guards is the producer restart surviving the mode flip.
//
// Regression test: enabling hardware sync used to enable ChunkModeActive on
// the camera, inflating PayloadSize past width*height — producer start failed
// and the chunk setting latched in the camera (volatile until power-cycle),
// so even toggling back to freerun left the camera dead until USB replug.

const API = 'http://localhost:8080'

interface Camera {
  id: number
  name: string
  serial: string
}

interface AvailableCamera {
  serial: string
}

interface CameraStatus {
  id: number
  online: boolean
  frames_produced: number
  fps_1s: number
  last_start_error?: string
}

interface StatusResponse {
  ok: boolean
  cameras: CameraStatus[]
}

async function camStatus(request: APIRequestContext, id: number): Promise<CameraStatus | null> {
  const s = (await request.get(`${API}/api/status`).then((r) => r.json())) as StatusResponse
  return s.cameras.find((c) => c.id === id) ?? null
}

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

  const created = await request.post(`${API}/api/cameras`, {
    headers: { 'Content-Type': 'application/json' },
    data: { name: `hwsync-${available[0].serial}`, serial: available[0].serial, focal_length_mm: 6 },
  })
  expect(created.ok(), `POST /api/cameras (status ${created.status()})`).toBeTruthy()
  return (await created.json()) as Camera
}

test.describe('Hardware-sync toggle lifecycle', () => {
  let seeded: Camera

  test.beforeAll(async ({ request }) => {
    seeded = await seedSingleCamera(request)
  })

  test.afterAll(async ({ request }) => {
    if (seeded) await request.delete(`${API}/api/cameras/${seeded.id}`)
  })

  test('enable → disable cycles restart the producer cleanly', async ({ request }) => {
    const put = async (body: Record<string, unknown>) => {
      const resp = await request.put(`${API}/api/cameras/${seeded.id}`, {
        headers: { 'Content-Type': 'application/json' },
        data: body,
      })
      expect(resp.ok(), `PUT ${JSON.stringify(body)} (status ${resp.status()})`).toBeTruthy()
    }

    const expectFreerunStreaming = () =>
      expect
        .poll(async () => (await camStatus(request, seeded.id))?.fps_1s ?? 0, {
          timeout: 25_000,
          intervals: [500, 1000],
        })
        .toBeGreaterThan(20)

    const cycle = async (round: number) => {
      await put({ hardware_sync_enabled: true, trigger_output_pin: 1 })

      // The producer must come back up in trigger-slave mode: online, no
      // start error. With no sync controller pulsing, fps settles at 0.
      await expect
        .poll(async () => {
          const s = await camStatus(request, seeded.id)
          return s ? { online: s.online, err: s.last_start_error ?? null } : null
        }, { timeout: 20_000, intervals: [500, 1000] })
        .toEqual({ online: true, err: null })
      await expect
        .poll(async () => (await camStatus(request, seeded.id))?.fps_1s ?? -1, {
          timeout: 10_000,
          intervals: [500, 1000],
        })
        .toBeLessThan(1)

      await put({ hardware_sync_enabled: false })
      await expectFreerunStreaming()
      const s = await camStatus(request, seeded.id)
      expect(s?.last_start_error, `round ${round}: no start error after disable`).toBeUndefined()
    }

    await expectFreerunStreaming() // baseline before the first flip
    await cycle(1)
    await cycle(2)

    const status = (await request.get(`${API}/api/status`).then((r) => r.json())) as StatusResponse
    expect(status.ok).toBe(true)
  })
})
