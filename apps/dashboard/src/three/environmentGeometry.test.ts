import { it, expect } from "vitest";
import { readFileSync } from "node:fs";
import { resolve } from "node:path";
import { parseCsv } from "../telemetry/load";
import { geometryFields, type ElevatedGeometry } from "../telemetry/contract";
import {
  terrainGeometry,
  barrierGeometry,
  forestPlacements,
  nearestRoad,
} from "./environmentGeometry";
const rows = parseCsv(
  readFileSync(
    resolve(
      import.meta.dirname,
      "../../public/demo/spa-p1-sprung/geometry.csv",
    ),
    "utf8",
  ),
  geometryFields,
  "geometry",
) as ElevatedGeometry[];
it("terrain is a bounded grid, never a folded offset ribbon", () => {
  const g = terrainGeometry(rows),
    p = g.getAttribute("position"),
    indices = g.index!;
  for (let i = 0; i < indices.count; i += 3)
    for (let j = 0; j < 3; j++) {
      const a = indices.getX(i + j),
        b = indices.getX(i + ((j + 1) % 3));
      expect(
        Math.hypot(p.getX(a) - p.getX(b), p.getZ(a) - p.getZ(b)),
      ).toBeLessThan(24);
    }
  g.dispose();
});
it("barrier segments have bounded edges and do not cross race surface", () => {
  for (const side of [-1, 1]) {
    const g = barrierGeometry(rows, side),
      p = g.getAttribute("position"),
      idx = g.index!;
    for (let i = 0; i < idx.count; i += 6) {
      const a = idx.getX(i),
        b = idx.getX(i + 1);
      expect(
        Math.hypot(
          p.getX(a) - p.getX(b),
          p.getY(a) - p.getY(b),
          p.getZ(a) - p.getZ(b),
        ),
      ).toBeLessThanOrEqual(15);
      const n = nearestRoad(
        rows,
        (p.getX(a) + p.getX(b)) / 2,
        -(p.getZ(a) + p.getZ(b)) / 2,
        2,
      );
      expect(n.distance).toBeGreaterThan(n.width + 0.99);
    }
    g.dispose();
  }
});
it("forest is seeded and excludes nearby branches of the road", () => {
  const a = forestPlacements(rows),
    b = forestPlacements(rows);
  expect(a).toEqual(b);
  expect(a.length).toBeGreaterThan(1000);
  for (const p of a) {
    const n = nearestRoad(rows, p.position.x, -p.position.z, 2);
    expect(n.distance).toBeGreaterThanOrEqual(n.width + 15);
  }
});
