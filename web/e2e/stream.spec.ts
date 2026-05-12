import { test, expect } from '@playwright/test'

test.describe('WebRTC streaming', () => {
  test('video element receives H.264 frames from the C++ server', async ({ page }) => {
    const consoleErrors: string[] = []
    page.on('pageerror', (e) => consoleErrors.push(`pageerror: ${e.message}`))
    page.on('console', (msg) => {
      if (msg.type() === 'error') consoleErrors.push(`console: ${msg.text()}`)
    })

    await page.goto('/')

    // The Stream component renders a <video> immediately, then connects.
    const video = page.locator('video').first()
    await expect(video).toBeVisible({ timeout: 10_000 })

    // Stream component surfaces connection state via text; wait for "streaming".
    await expect(page.getByText(/State:\s*streaming/i)).toBeVisible({ timeout: 20_000 })

    // Poll the video element until it has decoded at least one frame. After
    // connectionState=connected, the browser still needs a keyframe + a few
    // RTP packets before videoWidth/Height resolve.
    // Wait for the browser to actually decode a frame: videoWidth becomes
    // non-zero only after the first IDR is received and decoded.
    await expect
      .poll(
        async () =>
          video.evaluate((el: HTMLVideoElement) => el.videoWidth),
        { timeout: 25_000, intervals: [500, 1000, 2000] },
      )
      .toBeGreaterThan(0)

    const stats = await video.evaluate((el: HTMLVideoElement) => ({
      readyState:  el.readyState,
      videoWidth:  el.videoWidth,
      videoHeight: el.videoHeight,
      paused:      el.paused,
    }))
    console.log('video stats:', stats)
    expect(stats.readyState).toBeGreaterThanOrEqual(2)
    expect(stats.videoWidth).toBeGreaterThan(0)
    expect(stats.videoHeight).toBeGreaterThan(0)

    // Strong liveness check: count painted video frames over ~2 seconds via
    // requestVideoFrameCallback. We expect >= 30 frames at 30fps target.
    const framesIn2s = await video.evaluate((el: HTMLVideoElement) => {
      return new Promise<number>((resolve) => {
        let count = 0
        const start = performance.now()
        const tick = () => {
          count++
          if (performance.now() - start > 2000) {
            resolve(count)
          } else {
            el.requestVideoFrameCallback(tick)
          }
        }
        if (typeof el.requestVideoFrameCallback === 'function') {
          el.requestVideoFrameCallback(tick)
        } else {
          // Fallback (unlikely in Chromium): poll currentTime.
          const startTime = el.currentTime
          setTimeout(() => {
            resolve(Math.round((el.currentTime - startTime) * 30))
          }, 2000)
        }
        setTimeout(() => resolve(count), 5000)
      })
    })
    expect(framesIn2s).toBeGreaterThan(20)

    // Confirm via RTCPeerConnection.getStats that we have an inbound-rtp
    // entry for H.264 video with received bytes.
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
    expect((inbound as Record<string, number>).bytesReceived).toBeGreaterThan(10_000)
    expect((inbound as Record<string, number>).framesDecoded).toBeGreaterThan(10)

    console.log(`painted ${framesIn2s} frames in 2s; inbound:`, JSON.stringify(inbound))

    // No JS errors during streaming.
    expect(consoleErrors, consoleErrors.join('\n')).toEqual([])
  })

  test('status endpoint reports producer fps stable while streaming', async ({ request }) => {
    // Hit /api/status twice ~2s apart and confirm frames_produced increases
    // by at least ~50 frames (~25 fps, well above noise floor).
    const s1 = await request.get('http://localhost:8080/api/status').then((r) => r.json())
    await new Promise((res) => setTimeout(res, 2000))
    const s2 = await request.get('http://localhost:8080/api/status').then((r) => r.json())

    expect(s1.ok).toBe(true)
    expect(s2.ok).toBe(true)
    expect(s2.pipeline.frames_produced - s1.pipeline.frames_produced).toBeGreaterThan(50)
    expect(s2.pipeline.fps_1s).toBeGreaterThan(20)
  })
})
