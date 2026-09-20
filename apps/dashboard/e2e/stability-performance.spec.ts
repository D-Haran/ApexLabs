import { test, expect } from "@playwright/test";
import { writeFileSync } from "node:fs";
test("moving DRIVE and replay frame profile", async ({ page }) => {
  test.setTimeout(120000);
  await page.setViewportSize({ width: 1672, height: 944 });
  await page.goto("/?drive");
  await expect(page.getByRole("button", { name: "Start driving" })).toBeEnabled(
    { timeout: 45000 },
  );
  const report: Record<string, unknown> = {};
  for (const mode of ["drive", "optimized", "compare"]) {
    await page.getByLabel("Session view").selectOption(mode);
    if (mode === "drive") {
      await page.getByRole("button", { name: "Start driving" }).click();
      await page.keyboard.down("w");
    } else {
      await page
        .getByLabel("Replay time")
        .fill(mode === "compare" ? "175" : "40");
      if (mode === "compare")
        await page.getByLabel("Tire forces", { exact: true }).check();
      await page.getByRole("button", { name: "Play replay" }).click();
    }
    await page.waitForTimeout(2000);
    report[mode] = await page.evaluate(async () => {
      const intervals: number[] = [];
      let last = performance.now();
      const start = last;
      await new Promise<void>((resolve) => {
        function frame() {
          const now = performance.now();
          intervals.push(now - last);
          last = now;
          if (now - start < 8000) requestAnimationFrame(frame);
          else resolve();
        }
        requestAnimationFrame(frame);
      });
      intervals.shift();
      intervals.sort((a, b) => a - b);
      return {
        mean: intervals.reduce((a, b) => a + b, 0) / intervals.length,
        p95: intervals[Math.floor(intervals.length * 0.95)],
        frames: intervals.length,
        details: Reflect.get(window, "__apexDrivePerformance"),
        cpu: Reflect.get(window, "__apexDriveCPU"),
        clock: Reflect.get(window, "__apexDriveClock"),
      };
    });
    await page.screenshot({ path: `../../docs/screenshots/m61-${mode}.png` });
    if (mode === "drive") {
      await page.keyboard.up("w");
      await page.getByRole("button", { name: "Pause driving" }).click();
    } else {
      const pause = page.getByRole("button", { name: "Pause replay" });
      if (await pause.count()) await pause.click();
    }
  }
  report.native = await page.request
    .get("http://127.0.0.1:8765/state")
    .then((r) => r.json())
    .then((j) => j.performance);
  writeFileSync(
    `../../data/generated/dynamics/m61-${process.env.APEX_PROFILE_PHASE ?? "after"}-render.json`,
    JSON.stringify(report, null, 2),
  );
});
