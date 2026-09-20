import { describe, it, expect } from "vitest";
import {
  ghostAt,
  spatialDelta,
  inputAxis,
  type Ghost,
  type DriveSample,
} from "./contract";
describe("driver input and spatial ghost timing", () => {
  it("deadzone is symmetric and preserves analog travel", () => {
    expect(inputAxis(0.03, 0.08)).toBe(0);
    expect(inputAxis(-1, 0.08)).toBe(-1);
    expect(inputAxis(0.54, 0.08)).toBeCloseTo(0.5);
  });
  it("interpolates elapsed time by progress, not Euclidean distance", () => {
    const rows = [
      { s: 0, time: 0, position: [10, 0, 0] },
      { s: 100, time: 4, position: [10, 0, 0] },
      { s: 200, time: 10, position: [20, 0, 0] },
    ].map((s) => ({
      ...s,
      quaternion: [1, 0, 0, 0],
      wheels: [
        {
          center: [s.position[0] - 1, 0, 0],
          patch: [s.position[0] - 1, 0, -0.3],
        },
      ],
    })) as DriveSample[];
    expect(
      spatialDelta(
        { ...rows[1], lapElapsed: 5 } as DriveSample,
        { samples: rows } as Ghost,
        150,
      ),
    ).toBe(-2);
    expect(
      spatialDelta(rows[1], { samples: rows } as Ghost, 250),
    ).toBeUndefined();
    expect(ghostAt(rows, 150, "s")?.time).toBe(7);
    expect(ghostAt(rows, 150, "s")?.wheels[0].center[0]).toBe(14);
    expect(ghostAt(rows, 7, "time")?.s).toBe(150);
    expect(ghostAt(rows, 300, "s")?.time).toBe(10);
  });
});
