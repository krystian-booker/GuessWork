import { defineConfig, devices } from '@playwright/test'

// Two webServers: the headless guesswork binary on :8080 (pipeline + WebRTC
// signaling + status), and Vite on :5173 (proxies /api to :8080). The test
// drives a real Chromium against the Vite dev server, exercising the same
// codepath a developer browser would.
export default defineConfig({
  testDir: './e2e',
  timeout: 60_000,
  expect: { timeout: 15_000 },
  fullyParallel: false,
  retries: 0,
  reporter: 'list',

  webServer: [
    {
      command: '../build-fresh/guesswork --port 8080 --stream-fps 30',
      url: 'http://localhost:8080/api/status',
      reuseExistingServer: false,
      timeout: 30_000,
      stdout: 'pipe',
      stderr: 'pipe',
    },
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
      name: 'chromium',
      use: { ...devices['Desktop Chrome'] },
    },
  ],
})
