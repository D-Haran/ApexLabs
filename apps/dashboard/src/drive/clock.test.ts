import { describe, it, expect } from "vitest";
import { DriveClock } from "./clock";
import { spring } from "./camera";
import type { Ghost, DriveSample } from "./contract";
const session = {
  samples: [{ time: 0.02 }, { time: 0.04 }, { time: 0.06 }, { time: 0.1 }],
} as Ghost;
describe("authoritative forward replay clock", () => {
  it("plays from start, middle, seek and end using wall anchors", () => {
    let now = 0;
    const c = new DriveClock(() => now);
    c.load(session);
    c.play();
    now = 15;
    c.pause();
    expect(c.simulationTime).toBeCloseTo(0.035);
    c.play();
    now = 20;
    c.pause();
    expect(c.simulationTime).toBeCloseTo(0.04);
    c.seek(0.06);
    c.play();
    now = 30;
    c.pause();
    expect(c.simulationTime).toBeCloseTo(0.07);
    c.seek(0.1);
    c.play();
    expect(c.simulationTime).toBe(0.02);
    expect(c.playbackState).toBe("playing");
  });
  it("steps one recorded sample and remains paused, including between samples", () => {
    const c = new DriveClock(() => 0);
    c.load(session);
    c.step();
    expect(c.simulationTime).toBe(0.04);
    c.seek(0.05);
    c.step();
    expect(c.simulationTime).toBe(0.06);
    expect(c.playbackState).toBe("paused");
    c.seek(0.1);
    c.step();
    expect(c.simulationTime).toBe(0.1);
  });
  it("rejects reverse, zero and invalid rate; resets session", () => {
    const c = new DriveClock(() => 0);
    c.load(session);
    for (const r of [-1, 0, NaN, Infinity])
      expect(() => c.setRate(r)).toThrow();
    c.play();
    c.load(session);
    expect(c.playbackState).toBe("paused");
    expect(c.simulationTime).toBe(0.02);
  });
  it("is deterministic across repeated play pause seek sequences", () => {
    let now = 0;
    const c = new DriveClock(() => now);
    c.load(session);
    for (let i = 0; i < 100; i++) {
      c.seek(0.04);
      c.play();
      now += 10;
      c.pause();
      expect(c.simulationTime).toBeCloseTo(0.05);
    }
  });
  it("spring is independent of frame subdivision for fixed target", () => {
    let x = 0,
      v = 0;
    for (let i = 0; i < 60; i++) [x, v] = spring(x, v, 10, 11, 1 / 60);
    const [a, b] = spring(0, 0, 10, 11, 1);
    expect(x).toBeCloseTo(a, 10);
    expect(v).toBeCloseTo(b, 10);
  });
});

it("interpolates pose, quaternion and suspension by timestamp without pose history", () => {
  const make = (time: number, x: number, q: number[]) =>
    ({
      time,
      s: x,
      lapElapsed: time,
      position: [x, 0, 0],
      quaternion: q,
      velocity: [x, 0, 0],
      steering: x,
      throttle: 0,
      brake: 0,
      running: true,
      wheels: [
        {
          center: [x, 0, 0],
          patch: [x, 0, -1],
          force: [0, 0, 100],
          fx: 0,
          fy: 0,
          fz: 100,
          contact: true,
          compression: x,
          material: 0,
        },
      ],
    }) as DriveSample;
  let now = 0;
  const c = new DriveClock(() => now),
    a = make(0, 0, [1, 0, 0, 0]),
    b = make(1, 2, [0, 0, 0, 1]);
  c.load({ samples: [a, b] } as Ghost);
  c.seek(0.5);
  const mid = c.read()!;
  expect(mid.position[0]).toBe(1);
  expect(mid.velocity[0]).toBe(1);
  expect(mid.wheels[0].compression).toBe(1);
  expect(mid.quaternion[0]).toBeCloseTo(Math.SQRT1_2);
  expect(mid.quaternion[3]).toBeCloseTo(Math.SQRT1_2);
  c.seek(0.9);
  c.read();
  c.seek(0.5);
  expect(c.read()).toEqual(mid);
  c.load();
  c.push(a);
  now = 1000;
  c.push(b);
  expect(c.read()?.time).toBeCloseTo(0.94);
  now = 2000;
  expect(c.read()?.time).toBe(1);
  c.push({ ...a, time: 0 });
  expect(c.read()?.time).toBe(0);
});
