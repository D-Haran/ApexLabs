import { defineConfig } from '@playwright/test';
const baseURL = process.env.PLAYWRIGHT_BASE_URL || 'http://127.0.0.1:5173';
export default defineConfig({
  testDir: './e2e', timeout: 60000, workers: 1,
  use: {channel: 'chromium', baseURL, viewport: {width: 1512, height: 1100}, deviceScaleFactor: 1},
  webServer: process.env.PLAYWRIGHT_BASE_URL ? undefined : {command:'npm run dev', url:baseURL, reuseExistingServer:true},
  reporter:'list'
});
