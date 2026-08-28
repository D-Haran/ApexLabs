import { describe, it, expect } from "vitest";
import { readFileSync } from "node:fs";
import { resolve } from "node:path";
import { Vector3 } from "three";
import { parseCarManifest } from "./carManifest";
import {
  roadQuaternion,
  springStep,
  wheelSpin,
  roadSurfaceHeight,
} from "./vehiclePose";
import { decodeSession } from "../telemetry/load";
import { sampleAt } from "../telemetry/replay";
const base = resolve(import.meta.dirname, "../../public");
function fixture() {
  const manifest = JSON.parse(
    readFileSync(`${base}/demo/spa-p1-sprung/session.json`, "utf8"),
  );
  const files = Object.fromEntries(
    Object.values(manifest.files).map((f) => [
      f,
      readFileSync(`${base}/demo/spa-p1-sprung/${f}`, "utf8"),
    ]),
  );
  return { manifest, files };
}
describe("semantic cars and road integration", () => {
  it.each(["mclaren-p1", "mclaren-f1-2022"])(
    "validates grounded wheel maps: %s",
    (name) => {
      const m = parseCarManifest(
        JSON.parse(readFileSync(`${base}/assets/${name}.json`, "utf8")),
      );
      expect(m.wheels.FL.steerPivot).not.toBe(m.wheels.FR.steerPivot);
      const bad = structuredClone(m);
      bad.wheels.FL = bad.wheels.RR;
      expect(() => parseCarManifest(bad)).toThrow();
    },
  );
  it.each([-10, -5, 0, 5, 10])(
    "constructs an orthonormal road pose at %s degrees",
    (degrees) => {
      const angle = (degrees * Math.PI) / 180,
        q = roadQuaternion(0.7, Math.tan(angle)),
        forward = new Vector3(1, 0, 0).applyQuaternion(q),
        up = new Vector3(0, 1, 0).applyQuaternion(q);
      expect(forward.y).toBeCloseTo(Math.sin(angle), 12);
      expect(up.dot(forward)).toBeCloseTo(0, 12);
      expect(up.y).toBeGreaterThan(0);
      expect(q.length()).toBeCloseTo(1, 12);
    },
  );
  it("preserves horizontal heading when steering across a grade", () => {
    const q = roadQuaternion(0.4, 0.2, 0.7),
      forward = new Vector3(1, 0, 0).applyQuaternion(q);
    expect(Math.atan2(-forward.z, forward.x)).toBeCloseTo(0.7, 12);
  });
  it("wheel spin is independent of frame time and replay rate", () => {
    expect(wheelSpin(2 * Math.PI * 0.34, 0.34)).toBeCloseTo(-2 * Math.PI, 12);
    expect(() => wheelSpin(1, 0)).toThrow();
  });
  it("second-order camera has frame subdivision invariant settling", () => {
    const a = new Vector3(),
      av = new Vector3(),
      b = a.clone(),
      bv = av.clone(),
      target = new Vector3(10, 5, 1);
    for (let i = 0; i < 60; i++) springStep(a, av, target, 1 / 60);
    for (let i = 0; i < 120; i++) springStep(b, bv, target, 1 / 120);
    expect(a.distanceTo(b)).toBeLessThan(1e-12);
    expect(a.x).toBeLessThan(10);
    expect(a.distanceTo(target)).toBeLessThan(0.02);
  });
  it("loads/interpolates measured chassis data and exact mesh center heights", () => {
    const f = fixture(),
      s = decodeSession(f.manifest, f.files),
      t = (s.samples[20].time_s + s.samples[21].time_s) / 2,
      r = sampleAt(s, t);
    expect(r.chassis!.roll_rad).toBeCloseTo(
      (s.samples[20].chassis!.roll_rad + s.samples[21].chassis!.roll_rad) / 2,
      10,
    );
    expect(r.visual_distance_m).toBeGreaterThan(0);
    expect(sampleAt(s, t).visual_distance_m).toBe(r.visual_distance_m);
    for (const row of s.geometry.filter((_, i) => i % 30 === 0))
      expect(roadSurfaceHeight(s, row.x_m, -row.y_m, row.s_m)).toBeCloseTo(
        row.elevation_m,
        6,
      );
  });
  it("rejects missing/misaligned chassis companion", () => {
    const { manifest, files } = fixture();
    delete files["chassis.csv"];
    expect(() => decodeSession(manifest, files)).toThrow("Missing chassis");
  });
});

import { patchRollingSpeed } from "../telemetry/wheelMotion";
import type { Sample, Manifest } from "../telemetry/contract";
it("uses inside/outside rigid-body patch velocity and actual wheel heading", () => {
  const v = {
    front_axle_m: 1.5,
    rear_axle_m: 1,
    front_track_m: 2,
    rear_track_m: 2,
  } as Manifest["vehicle"];
  const s = {
    vx_m_s: 10,
    vy_m_s: 1,
    yaw_rate_rad_s: 2,
    steering_angle_rad: 0,
  } as Sample;
  expect(patchRollingSpeed(s, v, 0)).toBe(8);
  expect(patchRollingSpeed(s, v, 1)).toBe(12);
  s.steering_angle_rad = Math.PI / 2;
  expect(patchRollingSpeed(s, v, 0)).toBeCloseTo(4, 12);
  expect(patchRollingSpeed(s, v, 2)).toBe(8);
});
