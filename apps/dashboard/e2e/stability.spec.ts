import { spawn } from "node:child_process";
import { fileURLToPath } from "node:url";
import { test, expect, type Page } from "@playwright/test";
async function ready(page: Page) {
  await page.goto("/?drive");
  await expect(page.getByRole("button", { name: "Start driving" })).toBeEnabled(
    { timeout: 45000 },
  );
  await page.getByLabel("Session view").selectOption("optimized");
}
const time = (page: Page) =>
  page.evaluate(
    () => Reflect.get(window, "__apexDriveClock")?.simulationTime as number,
  );
const camera = (page: Page) =>
  page.evaluate(() => Reflect.get(window, "__apexDriveCamera"));
test("forward playback from start, middle, seek, end and exact sample stepping", async ({
  page,
}) => {
  await ready(page);
  const seek = page.getByLabel("Replay time");
  await page.getByRole("button", { name: "Play replay" }).click();
  await expect.poll(() => time(page)).toBeGreaterThan(0.2);
  await page.getByRole("button", { name: "Pause replay" }).click();
  await expect
    .poll(() =>
      page.evaluate(
        () => Reflect.get(window, "__apexDriveClock")?.playbackState,
      ),
    )
    .toBe("paused");
  const paused = await time(page);
  await page.waitForTimeout(150);
  expect(await time(page)).toBe(paused);
  await page.getByRole("button", { name: "Play replay" }).click();
  await expect.poll(() => time(page)).toBeGreaterThan(paused + 0.1);
  for (const selected of [80, 12, 40]) {
    await seek.fill(String(selected));
    await expect(
      page.getByRole("button", { name: "Play replay" }),
    ).toBeVisible();
    await page.getByRole("button", { name: "Play replay" }).click();
    await expect.poll(() => time(page)).toBeGreaterThan(selected + 0.1);
  }
  await seek.focus();
  await seek.press("End");
  await page.getByRole("button", { name: "Play replay" }).click();
  await expect.poll(() => time(page)).toBeLessThan(2);
  await expect.poll(() => time(page)).toBeGreaterThan(0.15);
  await seek.fill("10.01");
  const rows = await page.request
    .get("http://127.0.0.1:8765/ghost")
    .then((r) => r.json())
    .then((j) => j.samples);
  const next = rows.find((r: { time: number }) => r.time > 10.01 + 1e-9).time;
  await page.getByRole("button", { name: "Frame step" }).click();
  await expect.poll(() => time(page)).toBeCloseTo(next, 8);
  await page.waitForTimeout(150);
  expect(await time(page)).toBeCloseTo(next, 8);
  const snapshot = await page.evaluate(() =>
    Reflect.get(window, "__apexDriveClock"),
  );
  expect(snapshot.carTime).toBe(snapshot.simulationTime);
  expect(snapshot.direction).toBe("forward");
});
test("orbit preserves user radius and angles while moving, paused, seeking and switching modes", async ({
  page,
}) => {
  await ready(page);
  await page.getByLabel("Replay time").fill("40");
  await page.getByRole("button", { name: "Orbit camera", exact: true }).click();
  const canvas = page.locator(".drive-viewport canvas");
  const box = (await canvas.boundingBox())!;
  await page.mouse.move(box.x + box.width * 0.5, box.y + box.height * 0.5);
  await page.mouse.down();
  await page.mouse.move(
    box.x + box.width * 0.5 + 90,
    box.y + box.height * 0.5 + 35,
    { steps: 8 },
  );
  await page.mouse.up();
  const initial = await camera(page);
  expect(initial.mode).toBe("orbit");
  await page.getByRole("button", { name: "Play replay" }).click();
  await page.waitForTimeout(700);
  await page.getByRole("button", { name: "Pause replay" }).click();
  let current = await camera(page);
  for (let i = 0; i < 3; i++)
    expect(current.offset[i]).toBeCloseTo(initial.offset[i], 4);
  expect(current.pivot).not.toEqual(initial.pivot);
  await page.getByLabel("Replay time").fill("120");
  await page.waitForTimeout(100);
  current = await camera(page);
  for (let i = 0; i < 3; i++)
    expect(current.offset[i]).toBeCloseTo(initial.offset[i], 4);
  await page.getByRole("button", { name: "Chase camera", exact: true }).click();
  await page.waitForTimeout(150);
  await page.getByRole("button", { name: "Orbit camera", exact: true }).click();
  await page.waitForTimeout(100);
  current = await camera(page);
  for (let i = 0; i < 3; i++)
    expect(current.offset[i]).toBeCloseTo(initial.offset[i], 4);
});
test("input modes, diagnostics and honest path labels", async ({ page }) => {
  await ready(page);
  await page.getByLabel("Session view").selectOption("drive");
  await expect(page.getByLabel("Input mode")).toHaveValue("assisted");
  await page.getByLabel("Input mode").selectOption("authentic");
  await page.getByRole("button", { name: "Start driving" }).click();
  await page.keyboard.down("w");
  await expect
    .poll(async () =>
      page.request
        .get("http://127.0.0.1:8765/state")
        .then((r) => r.json())
        .then((s) => s.throttle),
    )
    .toBe(1);
  let s = await page.request
    .get("http://127.0.0.1:8765/state")
    .then((r) => r.json());
  expect(s.throttle).toBe(1);
  await page.keyboard.up("w");
  await page.getByRole("button", { name: "Pause driving" }).click();
  await page.getByRole("button", { name: "Reset session" }).click();
  await page.getByLabel("Input mode").selectOption("assisted");
  await page.getByRole("button", { name: "Start driving" }).click();
  await page.keyboard.down("w");
  await page.keyboard.down("a");
  await page.waitForTimeout(180);
  s = await page.request
    .get("http://127.0.0.1:8765/state")
    .then((r) => r.json());
  expect(s.throttle).toBeGreaterThan(0);
  expect(s.throttle).toBeLessThan(0.7);
  expect(s.steerCommand).toBeLessThan(0.7);
  expect(s.assisted).toBe(false);
  await page.keyboard.up("w");
  await page.keyboard.up("a");
  await page.getByRole("button", { name: "Pause driving" }).click();
  await page.getByText("Overlays", { exact: true }).click();
  await expect(
    page.getByText(
      "Optimized path — Not available: free-path optimizer incomplete",
    ),
  ).toBeVisible();
  await expect(
    page.getByLabel("Policy trajectory", { exact: true }),
  ).toBeVisible();
});
test("native policy optimization remains responsive and exposes solver status", async ({
  page,
}) => {
  test.setTimeout(180000);
  const server = spawn(
    "python3",
    [
      fileURLToPath(
        new URL("../../../tools/dynamics/server.py", import.meta.url),
      ),
      "--port",
      "8766",
    ],
    { stdio: "ignore" },
  );
  try {
    await expect
      .poll(async () => {
        try {
          return (await page.request.get("http://127.0.0.1:8766/state")).ok();
        } catch {
          return false;
        }
      })
      .toBe(true);
    await page.route("http://127.0.0.1:8765/**", (route) =>
      route.continue({ url: route.request().url().replace(":8765", ":8766") }),
    );
    await page.goto("/?drive");
    await expect(
      page.getByRole("button", { name: "Start driving" }),
    ).toBeEnabled({ timeout: 45000 });
    await page.getByLabel("Circuit").selectOption("technical_test_circuit");
    await expect(
      page.getByRole("button", { name: "Start driving" }),
    ).toBeEnabled({ timeout: 45000 });
    await page.getByRole("button", { name: "IMPROVE", exact: true }).click();
    await page.getByLabel("Optimization objective").selectOption("custom");
    for (const label of [
      "Time",
      "Path",
      "Clearance",
      "Balance",
      "Platform",
      "Fuel",
      "Wear",
    ])
      await page
        .getByLabel(`${label} weight`)
        .fill(label === "Time" ? "1" : "0");
    await page.getByRole("button", { name: "Optimize", exact: true }).click();
    await expect
      .poll(() =>
        page.request
          .get("http://127.0.0.1:8766/optimization")
          .then((r) => r.json())
          .then((j) => j.running),
      )
      .toBe(true);
    await page.getByLabel("Session view").selectOption("optimized");
    await page.getByLabel("Replay time").fill("20");
    await page.getByRole("button", { name: "Play replay" }).click();
    await expect.poll(() => time(page)).toBeGreaterThan(20.3);
    await page
      .getByRole("button", { name: "Orbit camera", exact: true })
      .click();
    await expect.poll(async () => (await camera(page))?.mode).toBe("orbit");
    await page.getByText("Solver details", { exact: true }).click();
    await expect(
      page.getByText("Algorithm: SciPy PRIMA COBYLA · no gradients"),
    ).toBeVisible();
    await expect
      .poll(
        () =>
          page.request
            .get("http://127.0.0.1:8766/optimization")
            .then((r) => r.json())
            .then((j) => j.status),
        { timeout: 150000, intervals: [1000] },
      )
      .toBe("accepted");
    await expect(
      page.getByRole("button", { name: "Optimize", exact: true }),
    ).toBeEnabled({ timeout: 5000 });
  } finally {
    await page.unrouteAll({ behavior: "wait" });
    await page.close();
    server.kill();
  }
});
