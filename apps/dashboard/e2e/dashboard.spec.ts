import { test, expect } from "@playwright/test";
import { mkdir, writeFile } from "node:fs/promises";
import { resolve } from "node:path";
const screenshots = resolve("../../docs/screenshots");
const cursor = (page: import("@playwright/test").Page) =>
  page.getByTestId("cursor-time");
test.beforeEach(async ({ page }) => {
  await page.goto("/");
  await expect(
    page.getByRole("heading", { name: "Spa-Francorchamps" }),
  ).toBeVisible({
    timeout: 30000,
  });
  await page
    .getByLabel("Current circuit")
    .selectOption("ApexLab Technical Test Circuit");
  await expect(
    page.getByRole("heading", { name: "Technical Test Circuit" }),
  ).toBeVisible({ timeout: 30000 });
  await page
    .getByLabel("Lap result mode")
    .getByRole("button", { name: "Reference", exact: true })
    .click();
  await page.getByRole("button", { name: /Peak tire utilization/ }).click();
  await expect(cursor(page)).toHaveText("0:35.000");
  await expect(page.locator(".scene-error")).toHaveCount(0);
  await page.locator(".viewport canvas").waitFor();
});
test("reference, optimized and comparison modes use validated replay data", async ({
  page,
}) => {
  await page
    .getByLabel("Lap result mode")
    .getByRole("button", { name: "Compare", exact: true })
    .click();
  await expect(page.locator(".comparison-readout")).toContainText("-15.075 s");
  await expect(
    page.getByRole("button", { name: "Optimized", exact: true }),
  ).toBeEnabled();
  await page.getByRole("button", { name: "Optimized", exact: true }).click();
  await expect(page.locator(".session-selected")).toContainText(
    "locally optimized racing line",
  );
  await expect(page.locator(".lap-result")).toContainText("0:35.635");
  await page.getByRole("button", { name: "Reference", exact: true }).click();
  await expect(page.locator(".lap-result")).toContainText("0:50.710");
  await page
    .getByLabel("Lap result mode")
    .getByRole("button", { name: "Compare", exact: true })
    .click();
  await expect(page.locator(".map-panel")).toContainText("Optimized");
  await expect(page.locator(".event-list")).toContainText(
    "Largest time-gain region",
  );
  await page.screenshot({
    path: resolve(screenshots, "milestone-6-reference-vs-optimized.png"),
    fullPage: true,
  });
});
test("normal replay, pause, rate, restart and exact frame stepping", async ({
  page,
}) => {
  await page.getByRole("button", { name: "Play", exact: true }).click();
  await expect(
    page.getByRole("button", { name: "Pause", exact: true }),
  ).toBeVisible();
  await expect(cursor(page)).not.toHaveText("0:35.000");
  await page.getByRole("button", { name: "Pause", exact: true }).click();
  const paused = await cursor(page).textContent();
  await page.waitForTimeout(120);
  await expect(cursor(page)).toHaveText(paused!);
  await page.getByLabel("Playback speed").selectOption("2");
  await page.getByRole("button", { name: "Restart", exact: true }).click();
  await expect(cursor(page)).toHaveText("0:00.000");
  await page.getByRole("button", { name: "Next frame", exact: true }).click();
  await expect(cursor(page)).toHaveText("0:00.005");
  await page
    .getByRole("button", { name: "Previous frame", exact: true })
    .click();
  await expect(cursor(page)).toHaveText("0:00.000");
});
test("event, chart, timeline and track-map cross-seeking", async ({ page }) => {
  await page.getByRole("button", { name: /Peak braking/ }).click();
  await expect(cursor(page)).toHaveText("0:09.560");
  const chart = page.locator(".u-over").first();
  await chart.click({ position: { x: 100, y: 25 } });
  await expect(cursor(page)).not.toHaveText("0:09.560");
  const afterChart = await cursor(page).textContent();
  const map = page.getByRole("img", {
    name: "Interactive circuit map; click to seek",
  });
  await map.click({ position: { x: 90, y: 70 } });
  await expect(cursor(page)).not.toHaveText(afterChart!);
  await page.getByRole("button", { name: /Peak tire utilization/ }).click();
  await expect(cursor(page)).toHaveText("0:35.000");
  await expect(page.locator(".force-readout")).toContainText("SATURATED");
  const positions = await page
    .locator(".shared-cursor")
    .evaluateAll((nodes) => nodes.map((n) => (n as HTMLElement).style.left));
  expect(new Set(positions).size).toBe(1);
  await page.getByLabel("Replay timeline").focus();
  await page.getByLabel("Replay timeline").press("Home");
  await expect(cursor(page)).toHaveText("0:00.000");
  await page.getByLabel("Replay timeline").press("End");
  await expect(cursor(page)).toHaveText("0:50.710");
});
test("cameras, overlays, tire inspection and visual captures", async ({
  page,
}) => {
  await mkdir(screenshots, { recursive: true });
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.waitForTimeout(700);
  await page.screenshot({
    path: resolve(screenshots, "milestone-5-engineering.png"),
    fullPage: true,
  });
  await page.screenshot({
    path: resolve(screenshots, "milestone-5-workspace.png"),
  });
  await page.getByRole("button", { name: "Overview", exact: true }).click();
  await page.waitForTimeout(700);
  await page.screenshot({
    path: resolve(screenshots, "milestone-5-overview.png"),
    fullPage: true,
  });
  await page.getByRole("button", { name: "Chase", exact: true }).click();
  await page.waitForTimeout(200);
  await page.getByRole("button", { name: "Orbit", exact: true }).click();
  await page.getByRole("button", { name: "Engineering", exact: true }).click();
  await page.getByRole("button", { name: "Overlays", exact: true }).click();
  for (const label of ["Normal loads", "Road/body axes + wheel contacts + CG", "Velocity vector"])
    await page.getByLabel(label, { exact: true }).check();
  await page.getByRole("button", { name: "Overlays", exact: true }).click();
  await page.getByRole("button", { name: "Inspect FR tire" }).click();
  await expect(page.locator(".force-readout")).toContainText(
    "FR OPERATING POINT",
  );
  await page.getByRole("button", { name: "Inspect RL tire" }).click();
  await page.waitForTimeout(400);
  await page.screenshot({
    path: resolve(screenshots, "milestone-5-forces.png"),
    fullPage: true,
  });
  expect(errors).toEqual([]);
});
test("two-lap spatial delta and regional statistics", async ({ page }) => {
  await page.getByRole("button", { name: "Compare higher grip" }).click();
  await expect(page.locator(".comparison-title")).toContainText("higher grip");
  await expect(page.locator(".comparison-readout")).toContainText("-1.865 s");
  await expect(
    page.locator(".plot-label").filter({ hasText: "Time delta" }),
  ).toBeVisible();
  await expect(page.locator(".region-panel tbody tr")).toHaveCount(2);
  await page.getByLabel("Region start").fill("700");
  await page.getByLabel("Region end").fill("800");
  await expect(page.locator(".region-panel")).not.toContainText(
    "Choose increasing",
  );
  await page.waitForTimeout(300);
  await page.screenshot({
    path: resolve(screenshots, "milestone-5-comparison.png"),
    fullPage: true,
  });
  await page.getByLabel("Region end").fill("600");
  await expect(page.locator(".region-panel")).toContainText(
    "Choose increasing bounds",
  );
  await page.getByRole("button", { name: "Remove", exact: true }).click();
  await expect(page.locator(".comparison-title")).toHaveCount(0);
});
test("chart zoom persists during playback; all required channels can be enabled", async ({
  page,
}) => {
  const chart = page.locator(".u-over").first();
  const box = (await chart.boundingBox())!;
  await page.mouse.move(box.x + 100, box.y + 30);
  await page.mouse.down();
  await page.mouse.move(box.x + 400, box.y + 30, { steps: 8 });
  await page.mouse.up();
  const prior = await page
    .locator(".plot-row")
    .first()
    .getAttribute("data-range");
  expect(prior).not.toBe("full");
  await page.getByRole("button", { name: "Play", exact: true }).click();
  await page.waitForTimeout(150);
  await page.getByRole("button", { name: "Pause", exact: true }).click();
  expect(
    await page.locator(".plot-row").first().getAttribute("data-range"),
  ).toBe(prior);

  const range = prior!.split(",").map(Number);
  await page.keyboard.down("Shift");
  await page.mouse.move(box.x + 200, box.y + 30);
  await page.mouse.down();
  await page.mouse.move(box.x + 260, box.y + 30, { steps: 5 });
  await page.mouse.up();
  await page.keyboard.up("Shift");
  const panned = (await page
    .locator(".plot-row")
    .first()
    .getAttribute("data-range"))!
    .split(",")
    .map(Number);
  expect(panned[1] - panned[0]).toBeCloseTo(range[1] - range[0], 6);
  expect(panned[0]).toBeLessThan(range[0]);
  await page.getByRole("button", { name: "Channels 4", exact: true }).click();
  for (const label of [
    "Lateral acceleration",
    "Longitudinal acceleration",
    "Yaw rate",
    "Lateral error",
    "FL utilization",
    "FR utilization",
    "RL utilization",
    "RR utilization",
    "FL Fz",
    "FR Fz",
    "RL Fz",
    "RR Fz",
  ])
    await page.getByLabel(label, { exact: false }).check();
  await expect(page.locator(".plot-row")).toHaveCount(16);
  await page.getByRole("button", { name: "Time", exact: true }).click();
  await expect(page.locator(".plot-row").first()).toHaveAttribute(
    "data-range",
    "full",
  );
});
test("file loading and clear schema rejection", async ({ page }) => {
  const dir = resolve("public/demo/baseline");
  await page
    .locator("input[type=file]")
    .first()
    .setInputFiles(
      [
        "session.json",
        "telemetry.csv",
        "wheels.csv",
        "geometry.csv",
        "spatial.csv",
      ].map((f) => resolve(dir, f)),
    );
  await expect(page.getByRole("status")).toHaveCount(0);
  await expect(cursor(page)).toHaveText("0:35.000");
  await page
    .locator("input[type=file]")
    .first()
    .setInputFiles({
      name: "session.json",
      mimeType: "application/json",
      buffer: Buffer.from('{"schema_version":99}'),
    });
  await expect(page.getByRole("alert")).toContainText(
    "Unsupported session schema",
  );
});
test("desktop resizing at 1024 and 1920 pixels", async ({ page }) => {
  for (const width of [1024, 1920]) {
    await page.setViewportSize({ width, height: 1000 });
    await expect(
      page.getByRole("button", { name: "Play", exact: true }),
    ).toBeVisible();
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth <= innerWidth,
      ),
    ).toBe(true);
    await page.screenshot({
      path: resolve(screenshots, `milestone-5-${width}.png`),
      fullPage: true,
    });
  }
});
test("elevated Spa and McLaren P1 showcase loads without render errors", async ({
  page,
}) => {
  await mkdir(screenshots, { recursive: true });
  const errors: string[] = [];
  page.on("pageerror", (error) => errors.push(error.message));
  await page.getByLabel("Current circuit").selectOption("Spa-Francorchamps");
  await expect(
    page.getByRole("heading", { name: "Spa-Francorchamps" }),
  ).toBeVisible({ timeout: 30000 });
  await expect(page.locator(".session-selected")).toContainText(
    "McLaren P1 — approximate model",
  );
  await expect(page.locator(".scene-caption")).toContainText(
    "Mclaren P1 visual asset",
  );
  await expect(page.locator(".scene-error")).toHaveCount(0);
  await page.getByRole("button", { name: "Chase", exact: true }).click();
  await page.waitForTimeout(1200);
  await page.screenshot({
    path: resolve(screenshots, "milestone-6-spa-p1.png"),
    fullPage: true,
  });
  await page.getByText("Performance measurements", { exact: true }).click();
  await page
    .getByRole("button", { name: "Measure current view", exact: true })
    .click();
  await page.getByRole("button", { name: "Play", exact: true }).click();
  const link = page.getByRole("link", { name: "Download measurements" });
  await expect(link).toBeVisible({ timeout: 30000 });
  await page.getByRole("button", { name: "Pause", exact: true }).click();
  const href = (await link.getAttribute("href"))!;
  const metrics = JSON.parse(decodeURIComponent(href.split(",")[1]));
  metrics.renderer = await page
    .locator(".viewport canvas")
    .evaluate((canvas: HTMLCanvasElement) => {
      const gl = canvas.getContext("webgl2")!;
      const info = gl.getExtension("WEBGL_debug_renderer_info");
      return info
        ? gl.getParameter(info.UNMASKED_RENDERER_WEBGL)
        : gl.getParameter(gl.RENDERER);
    });
  metrics.scenario =
    "Spa/P1 processed asset and sprung body; elevated real track, terrain, kerbs, barriers, pit context and instanced forest; chase camera; 1512×1100";
  await writeFile(
    resolve("../../docs/experiments/milestone-5_5-performance.json"),
    JSON.stringify(metrics, null, 2) + "\n",
  );
  expect(errors).toEqual([]);
});
test("measure real rendering, seek and session load latency", async ({
  page,
}) => {
  await page.getByRole("button", { name: "Compare higher grip" }).click();
  await expect(page.locator(".comparison-title")).toBeVisible();
  await page.getByText("Performance measurements", { exact: true }).click();
  await page
    .getByRole("button", { name: "Measure current view", exact: true })
    .click();
  await page.getByRole("button", { name: "Play", exact: true }).click();
  const link = page.getByRole("link", { name: "Download measurements" });
  await expect(link).toBeVisible({ timeout: 30000 });
  await page.getByRole("button", { name: "Pause", exact: true }).click();
  const href = (await link.getAttribute("href"))!;
  const metrics = JSON.parse(decodeURIComponent(href.split(",")[1]));
  metrics.renderer = await page
    .locator(".viewport canvas")
    .evaluate((canvas: HTMLCanvasElement) => {
      const gl = canvas.getContext("webgl2")!;
      const info = gl.getExtension("WEBGL_debug_renderer_info");
      return info
        ? gl.getParameter(info.UNMASKED_RENDERER_WEBGL)
        : gl.getParameter(gl.RENDERER);
    });
  metrics.scenario =
    "Headless Chromium / native Apple GPU; engineering follow; Lap A+B, five telemetry plots, default force overlays; playback 1x; 1512×1100";
  await writeFile(
    resolve("../../docs/experiments/milestone-5-performance.json"),
    JSON.stringify(metrics, null, 2) + "\n",
  );
  expect(metrics.frame_samples).toBe(300);
  expect(metrics.average_frame_ms).toBeGreaterThan(0);
  console.log("PERFORMANCE", metrics);
});
