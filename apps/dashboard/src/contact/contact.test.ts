import { describe, it, expect } from "vitest";
import { readFileSync } from "node:fs";
import { Vector3 } from "three";
import {
  parseContactRecording,
  sampleContact,
  toRender,
  toRenderQuaternion,
  type ContactRecording,
} from "./contract";
const fixture = JSON.parse(
  readFileSync(
    new URL("../../public/demo/contact-3d/crest-40.json", import.meta.url),
    "utf8",
  ),
) as ContactRecording;
describe("native contact recording", () => {
  it("retains zero airborne forces and uses the recorded elevated pose", () => {
    const recording = parseContactRecording(fixture);
    const air = recording.samples.find((s) =>
      s.wheels.every((w) => !w.contact),
    )!;
    expect(air).toBeDefined();
    const sample = sampleContact(recording, air.time + 0.005);
    expect(
      sample.wheels.every((w) => !w.contact && w.force.every((v) => v === 0)),
    ).toBe(true);
    expect(sample.position[2]).toBeGreaterThan(0.45);
    expect(sample.quaternion.reduce((a, b) => a + b * b, 0)).toBeCloseTo(1, 12);
    expect(toRender(sample.position)[1]).toBe(sample.position[2]);
  });
  it("transforms vectors and orientation consistently without changing force signs", () => {
    expect(toRender([0, 100, 1000])).toEqual([0, 1000, -100]);
    const yaw90 = toRenderQuaternion([Math.SQRT1_2, 0, 0, Math.SQRT1_2]);
    const forward = new Vector3(1, 0, 0).applyQuaternion(yaw90);
    expect(forward.z).toBeCloseTo(-1, 12);
    const pitch = toRenderQuaternion([Math.cos(0.1), 0, -Math.sin(0.1), 0]);
    expect(new Vector3(1, 0, 0).applyQuaternion(pitch).y).toBeGreaterThan(0);
  });
  it("rejects airborne forces, invalid quaternions and incomplete road geometry", () => {
    const bad = structuredClone(fixture),
      air = bad.samples.find((s) => s.wheels.every((w) => !w.contact))!;
    air.wheels[0].fz = 1;
    expect(() => parseContactRecording(bad)).toThrow("wheel forces");
    air.wheels[0].fz = 0;
    bad.samples[0].quaternion[0] = 0;
    expect(() => parseContactRecording(bad)).toThrow("pose");
    bad.samples[0].quaternion[0] = 1;
    bad.surface.positions.pop();
    expect(() => parseContactRecording(bad)).toThrow("road mesh");
  });
});
