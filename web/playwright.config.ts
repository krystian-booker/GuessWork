import { defineConfig, devices } from '@playwright/test'

// Two projects:
//  - mocked (default): every API call is intercepted with page.route(); only
//    the Vite dev server runs. Works with no camera and no backend.
//  - hardware (GW_E2E_HW=1): also boots ../build-fresh/guesswork on :8080 and
//    runs the hw.*.spec.ts suite against real Spinnaker hardware.
const HW = !!process.env.GW_E2E_HW

export default defineConfig({
  testDir: './e2e',
  timeout: 60_000,
  expect: { timeout: 15_000 },
  fullyParallel: false,
  // The hw.*.spec.ts files all seed/wipe the cameras table around the single
  // physical camera — parallel workers would stomp each other's rows.
  workers: HW ? 1 : undefined,
  retries: 0,
  reporter: 'list',

  webServer: [
    ...(HW
      ? [
          {
            command: '../build-fresh/guesswork --port 8080 --stream-fps 30',
            url: 'http://localhost:8080/api/status',
            reuseExistingServer: false,
            timeout: 30_000,
            stdout: 'pipe' as const,
            stderr: 'pipe' as const,
          },
        ]
      : []),
    {
      command: 'npm run dev -- --strictPort',
      url: 'http://localhost:5173',
      reuseExistingServer: false,
      timeout: 30_000,
    },
  ],

  use: {
    baseURL: 'http://localhost:5173',
    headless: true,
    video: 'off',
  },

  projects: [
    {
      name: 'mocked',
      testIgnore: /hw\./,
      use: { ...devices['Desktop Chrome'] },
    },
    ...(HW
      ? [
          {
            name: 'hardware',
            testMatch: /hw\./,
            use: { ...devices['Desktop Chrome'] },
          },
        ]
      : []),
  ],
})
