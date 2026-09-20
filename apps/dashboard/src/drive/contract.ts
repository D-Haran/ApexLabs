import { Quaternion, Vector3 } from "three";
import {
  sampleContact,
  type ContactSample,
  type ContactRecording,
} from "../contact/contract";
import type { ElevatedGeometry, RenderContext } from "../telemetry/contract";
export interface DriveSample extends ContactSample {
  steeringRateActual?: number;
  steerCommand?: number;
  steeringRequest?: number;
  schema: 1;
  model: "dynamic_contact";
  s: number;
  lateral: number;
  lap: number;
  lapElapsed: number;
  bodyAcceleration: [number, number, number];
  omega: [number, number, number];
  gear: number;
  rpm: number;
  fuel: number;
  battery: number;
  deployed: number;
  recovered: number;
  fuelFlow: number;
  electricPower: number;
  drag: number;
  downforce: number;
  frontAeroFraction: number;
  cda: number;
  cla: number;
  frontHeight: number;
  rearHeight: number;
  drs: number;
  aeroState: number;
  aeroForce: [number, number, number];
  assisted: boolean;
  wheels: (ContactSample["wheels"][number] & {
    tread: number;
    carcass: number;
    wear: number;
    requestedUtilization: number;
  })[];
  running?: boolean;
  error?: string;
  inputStale?: boolean;
  performance?: {
    physicsHz: number;
    publishHz: number;
    stepBatchMeanMs: number;
    stepBatchP95Ms: number;
    wallLagMs: number;
  };
}
export interface DriveMetadata {
  provenance?: Record<string, unknown>;
  model: "dynamic_contact";
  name: string;
  family: string;
  track: string;
  length: number;
  sectors: number[];
  physicsHz: number;
  renderContext?: RenderContext;
  physicalKerbs?: {
    columns: number;
    positions: [number, number, number][];
    colors: number[];
  }[];
  values: Record<string, number>;
  parameters: ContactRecording["parameters"];
  geometry: ElevatedGeometry[];
}
export interface Ghost {
  model: "dynamic_contact";
  accepted: boolean;
  samples: DriveSample[];
  lap_time_s: number;
  vehicle: string;
  track: string;
  objective_name?: string;
}
export function spatialDelta(
  sample: DriveSample | undefined,
  baseline: Ghost | undefined,
  distance: number,
) {
  if (
    !sample ||
    !baseline ||
    !baseline.samples.length ||
    distance < baseline.samples[0].s ||
    distance > baseline.samples.at(-1)!.s
  )
    return undefined;
  return sample.lapElapsed - ghostAt(baseline.samples, distance, "s")!.time;
}
export function ghostAt(
  rows: DriveSample[],
  value: number,
  by: "time" | "s",
): DriveSample | undefined {
  if (!rows.length) return;
  if (value <= rows[0][by]) return rows[0];
  if (value >= rows.at(-1)![by]) return rows.at(-1);
  let lo = 0,
    hi = rows.length - 1;
  while (lo + 1 < hi) {
    const m = (lo + hi) >> 1;
    if (rows[m][by] <= value) lo = m;
    else hi = m;
  }
  const a = rows[lo],
    b = rows[hi],
    f = (value - a[by]) / (b[by] - a[by]);
  const time = a.time + f * (b.time - a.time);
  const pose = sampleContact({ samples: [a, b] }, time);
  const result = { ...a, ...pose } as DriveSample;
  // Numeric telemetry and suspension interpolate from the same pair as the pose.
  // Discrete gears, lap/contact/material flags remain sample-and-held.
  for (const key of Object.keys(a) as (keyof DriveSample)[]) {
    if (
      [
        "gear",
        "lap",
        "schema",
        "quaternion",
        "position",
        "wheels",
        "time",
      ].includes(key)
    )
      continue;
    const x = a[key],
      y = b[key];
    if (typeof x === "number" && typeof y === "number")
      Reflect.set(result, key, x + f * (y - x));
    else if (
      Array.isArray(x) &&
      Array.isArray(y) &&
      x.every((v) => typeof v === "number")
    )
      Reflect.set(
        result,
        key,
        (x as number[]).map((v, i) => v + f * ((y as number[])[i] - v)),
      );
  }
  result.wheels = pose.wheels.map((w, i) => {
    const wheel = { ...a.wheels[i], ...w };
    for (const key of Object.keys(wheel)) {
      if (["material", "contact"].includes(key)) continue;
      const x = Reflect.get(a.wheels[i], key),
        y = Reflect.get(b.wheels[i], key);
      if (typeof x === "number" && typeof y === "number")
        Reflect.set(wheel, key, x + f * (y - x));
      else if (Array.isArray(x) && Array.isArray(y))
        Reflect.set(
          wheel,
          key,
          x.map((v, j) => v + f * (y[j] - v)),
        );
    }
    // Never invent airborne force while holding the left sample contact flag.
    if (!wheel.contact) {
      wheel.fx = wheel.fy = wheel.fz = wheel.utilization = 0;
      wheel.force = [0, 0, 0];
    }
    return wheel;
  });
  return result;
}
export function inputAxis(value: number, deadzone: number) {
  const a = Math.abs(value);
  return a <= deadzone
    ? 0
    : Math.sign(value) * Math.min(1, (a - deadzone) / (1 - deadzone));
}

export function forceComponents(
  sample: DriveSample,
  wheel: number,
  frame: "Tire Local" | "Body" | "World",
): [number, number, number] {
  const w = sample.wheels[wheel];
  if (frame === "Tire Local") return [w.fx, w.fy, w.fz];
  if (frame === "World") return w.force;
  const [qw, qx, qy, qz] = sample.quaternion;
  const v = new Vector3(...w.force).applyQuaternion(
    new Quaternion(qx, qy, qz, qw).invert(),
  );
  return [v.x, v.y, v.z];
}

export function sideslip(sample: DriveSample) {
  const [w, x, y, z] = sample.quaternion;
  const v = new Vector3(...sample.velocity).applyQuaternion(
    new Quaternion(x, y, z, w).invert(),
  );
  return Math.atan2(v.y, Math.max(0.01, v.x));
}
