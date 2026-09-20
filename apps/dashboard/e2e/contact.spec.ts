import { test, expect } from "@playwright/test";
import { resolve } from "node:path";
import { mkdir } from "node:fs/promises";
test("native airborne and curb replays preserve contact state and reset", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto("/?contact");
  await expect(
    page.getByText("P1 mesh ready · recorded contact pose"),
  ).toBeVisible({ timeout: 60000 });
  await expect(page.locator(".contact-hud b")).toHaveText("4 / 4 CONTACTS");
  await page.getByLabel("Contact replay time").fill("1.8");
  await expect(page.locator(".contact-hud b")).toHaveText("0 / 4 CONTACTS");
  await expect(page.locator("tbody tr")).toHaveCount(4);
  await expect(page.locator("tbody .contact-off")).toHaveCount(4);
  await mkdir(resolve("../../docs/screenshots"), { recursive: true });
  await page.screenshot({
    path: resolve("../../docs/screenshots/m6-contact-airborne.png"),
    fullPage: true,
  });
  await page.getByRole("button", { name: "Reset", exact: true }).click();
  await expect(page.locator(".contact-hud b")).toHaveText("4 / 4 CONTACTS");
  await page.getByRole("button", { name: "Play", exact: true }).click();
  await expect(
    page.getByRole("button", { name: "Pause", exact: true }),
  ).toBeVisible();
  await page.getByRole("button", { name: "Pause", exact: true }).click();
  await page.getByLabel("Contact scene").selectOption("curb");
  await expect(
    page.getByText("P1 mesh ready · recorded contact pose"),
  ).toBeVisible();
  await expect(page.getByLabel("Contact replay time")).toHaveValue("0");
  await page.getByLabel("Contact replay time").fill("3");
  await expect(page.locator(".contact-hud b")).toHaveText("4 / 4 CONTACTS");
  await page.screenshot({
    path: resolve("../../docs/screenshots/m6-contact-curb.png"),
    fullPage: true,
  });
  await page.getByLabel("Contact scene").selectOption("drop");
  await expect(
    page.getByText("P1 mesh ready · recorded contact pose"),
  ).toBeVisible();
  await expect(page.getByLabel("Contact replay time")).toHaveValue("0");
  await expect(page.locator(".contact-hud b")).toHaveText("0 / 4 CONTACTS");
  await page.getByLabel("Contact replay time").fill("5.5");
  await expect(page.locator(".contact-hud b")).toHaveText("4 / 4 CONTACTS");
  expect(errors).toEqual([]);
});
test("preserved Spa replay still opens independently", async ({ page }) => {
  await page.goto("/");
  await expect(page.locator(".scene-caption")).toContainText("Mesh ready", {
    timeout: 60000,
  });
  await expect(page.locator(".scene-caption")).toContainText("SPRUNG BODY");
});
