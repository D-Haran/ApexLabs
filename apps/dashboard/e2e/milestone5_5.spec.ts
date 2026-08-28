import { test, expect } from "@playwright/test";
import { resolve } from "node:path";
import { mkdir, writeFile } from "node:fs/promises";
const screenshots = resolve("../../docs/screenshots");
test("P1/F1 assets, chassis telemetry and deterministic paused replay", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto("/");
  await expect(page.locator(".scene-caption")).toContainText("Mesh ready", {
    timeout: 60000,
  });
  await expect(page.locator(".scene-caption")).toContainText("SPRUNG BODY");
  await page.getByRole("button", { name: "Chase", exact: true }).click();
  await mkdir(screenshots, { recursive: true });
  await page.waitForTimeout(1500);
  await page.screenshot({
    path: resolve(screenshots, "milestone-5_5-spa-p1.png"),
    fullPage: true,
  });
  const canvas = page.locator(".viewport canvas");
  const before = await canvas.screenshot();
  await page.waitForTimeout(500);
  const after = await canvas.screenshot();
  expect(before.equals(after)).toBe(true);
  await page.getByRole("button", { name: "Overlays", exact: true }).click();
  await page.getByLabel("Road/body axes + wheel contacts + CG").check();
  await expect(page.getByTestId("chassis-readout")).toContainText(
    "SIMULATED CHASSIS",
  );
  await page.getByRole("button", { name: "Overlays", exact: true }).click();
  await page.screenshot({
    path: resolve(screenshots, "milestone-5_5-grounding-debug.png"),
    fullPage: true,
  });
  await page.getByLabel("Visual model").selectOption("mclaren-f1-2022");
  await expect(page.locator(".scene-caption")).toContainText("Gulf Mclaren");
  await expect(page.locator(".scene-caption")).toContainText("Mesh ready", {
    timeout: 60000,
  });
  await page.waitForTimeout(1000);
  await page.screenshot({
    path: resolve(screenshots, "milestone-5_5-spa-f1.png"),
    fullPage: true,
  });
  await page.getByLabel("Visual model").selectOption("mclaren-p1");
  await expect(page.locator(".scene-caption")).toContainText("Mclaren P1");
  await expect(page.locator(".scene-caption")).toContainText("Mesh ready");
  await page.getByText("Performance measurements", { exact: true }).click();
  await page
    .getByRole("button", { name: "Measure current view", exact: true })
    .click();
  await page.getByRole("button", { name: "Play", exact: true }).click();
  const link = page.getByRole("link", { name: "Download measurements" });
  await expect(link).toBeVisible({ timeout: 30000 });
  await page.getByRole("button", { name: "Pause", exact: true }).click();
  const metrics = JSON.parse(
    decodeURIComponent((await link.getAttribute("href"))!.split(",")[1]),
  );
  await writeFile(
    resolve("../../docs/experiments/milestone-5_5-p1-debug-performance.json"),
    JSON.stringify(metrics, null, 2) + "\n",
  );
  expect(errors).toEqual([]);
});
for (const scene of [
  "flat",
  "uphill",
  "downhill",
  "left",
  "right",
  "braking",
  "acceleration",
]) {
  test(`physical validation scene: ${scene}`, async ({ page }) => {
    const errors: string[] = [];
    page.on("pageerror", (e) => errors.push(e.message));
    await page.goto(`/?scene=${scene}`);
    await expect(page.locator(".scene-caption")).toContainText("Mesh ready", {
      timeout: 60000,
    });
    await expect(page.locator(".lap-result")).toContainText("NOT A LAP");
    await page
      .getByRole("button", { name: "Engineering", exact: true })
      .click();
    await page.getByRole("button", { name: "Overlays", exact: true }).click();
    await page.getByLabel("Road/body axes + wheel contacts + CG").check();
    await page.getByRole("button", { name: "Overlays", exact: true }).click();
    await page.waitForTimeout(300);
    await page.screenshot({
      path: resolve(screenshots, `milestone-5_5-${scene}.png`),
      fullPage: true,
    });
    expect(errors).toEqual([]);
  });
}
test("F1 render performance with full source silhouette", async ({ page }) => {
  await page.goto("/");
  await expect(page.locator(".scene-caption")).toContainText("Mesh ready", {
    timeout: 60000,
  });
  await page.getByLabel("Visual model").selectOption("mclaren-f1-2022");
  await expect(page.locator(".scene-caption")).toContainText("Gulf Mclaren");
  await expect(page.locator(".scene-caption")).toContainText("Mesh ready", {
    timeout: 60000,
  });
  await page.getByRole("button", { name: "Chase", exact: true }).click();
  await page.getByText("Performance measurements", { exact: true }).click();
  await page
    .getByRole("button", { name: "Measure current view", exact: true })
    .click();
  await page.getByRole("button", { name: "Play", exact: true }).click();
  const link = page.getByRole("link", { name: "Download measurements" });
  await expect(link).toBeVisible({ timeout: 30000 });
  await page.getByRole("button", { name: "Pause", exact: true }).click();
  const metrics = JSON.parse(
    decodeURIComponent((await link.getAttribute("href"))!.split(",")[1]),
  );
  await writeFile(
    resolve("../../docs/experiments/milestone-5_5-f1-performance.json"),
    JSON.stringify(metrics, null, 2) + "\n",
  );
});
