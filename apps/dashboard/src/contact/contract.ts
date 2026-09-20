import { Quaternion } from "three";
export type V3 = [number, number, number];
export interface ContactWheel {
  center: V3;
  patch: V3;
  force: V3;
  contact: boolean;
  fz: number;
  fx: number;
  fy: number;
  slip: number;
  utilization: number;
  compression: number;
  tireCompression: number;
  damperVelocity: number;
  damperForce: number;
  material: number;
}
export interface ContactSample {
  time: number;
  position: V3;
  quaternion: [number, number, number, number];
  velocity: V3;
  acceleration: V3;
  steering: number;
  throttle: number;
  brake: number;
  wheels: ContactWheel[];
}
export interface ContactRecording {
  schema_version: 1;
  kind: "contact_validation_interval";
  name: string;
  source: string;
  samples: ContactSample[];
  parameters: {
    cg_height_m: number;
    body_axle_midpoint_m: number;
    total_mass_kg: number;
    corner: { radius_m: number }[];
  };
  summary: {
    dt_s: number;
    all_airborne_duration_s: number;
    max_body_height_above_static_road_m: number;
    peak_total_normal_force_n: number;
  };
  surface: {
    columns: number;
    positions: number[];
    normals: number[];
    materials: number[];
  };
}
const vector = (a: unknown, length: number) =>
  Array.isArray(a) && a.length === length && a.every(Number.isFinite);
export function parseContactRecording(value: unknown): ContactRecording {
  const r = value as ContactRecording;
  if (
    r?.schema_version !== 1 ||
    r.kind !== "contact_validation_interval" ||
    !Array.isArray(r.samples) ||
    r.samples.length < 2
  )
    throw new Error("Unsupported contact recording");
  const p = r.parameters;
  if (
    !p ||
    !Number.isFinite(p.cg_height_m) ||
    !Number.isFinite(p.body_axle_midpoint_m) ||
    !Number.isFinite(p.total_mass_kg) ||
    p.total_mass_kg <= 0 ||
    p.corner?.length !== 4 ||
    p.corner.some((c) => !Number.isFinite(c.radius_m) || c.radius_m <= 0)
  )
    throw new Error("Invalid contact vehicle geometry");
  for (const [i, s] of r.samples.entries()) {
    if (
      !Number.isFinite(s.time) ||
      (i && s.time <= r.samples[i - 1].time) ||
      !vector(s.position, 3) ||
      !vector(s.quaternion, 4) ||
      !vector(s.velocity, 3) ||
      !vector(s.acceleration, 3) ||
      ![s.steering, s.throttle, s.brake].every(Number.isFinite) ||
      s.wheels?.length !== 4 ||
      Math.abs(s.quaternion.reduce((a, b) => a + b * b, 0) - 1) > 1e-6
    )
      throw new Error("Invalid contact pose/timestamp");
    for (const w of s.wheels) {
      if (
        !vector(w.center, 3) ||
        !vector(w.patch, 3) ||
        !vector(w.force, 3) ||
        ![
          w.fz,
          w.fx,
          w.fy,
          w.slip,
          w.utilization,
          w.compression,
          w.tireCompression,
          w.damperVelocity,
          w.damperForce,
        ].every(Number.isFinite) ||
        typeof w.contact !== "boolean" ||
        w.fz < 0 ||
        w.utilization < 0 ||
        w.utilization > 1 + 1e-8 ||
        !Number.isInteger(w.material) ||
        w.material < 0 ||
        w.material > 3 ||
        (!w.contact &&
          (w.fz !== 0 ||
            w.fx !== 0 ||
            w.fy !== 0 ||
            w.force.some((x) => x !== 0)))
      )
        throw new Error("Invalid contact wheel forces");
    }
  }
  const m = r.surface;
  if (
    !m ||
    !Number.isInteger(m.columns) ||
    m.columns < 2 ||
    !Array.isArray(m.positions) ||
    m.positions.length < m.columns * 6 ||
    m.positions.length % (m.columns * 3) !== 0 ||
    !m.positions.every(Number.isFinite) ||
    !Array.isArray(m.normals) ||
    m.normals.length !== m.positions.length ||
    !m.normals.every(Number.isFinite) ||
    !Array.isArray(m.materials) ||
    m.materials.length * 3 !== m.positions.length ||
    m.materials.some((x) => !Number.isInteger(x) || x < 0 || x > 3)
  )
    throw new Error("Invalid authoritative road mesh");
  if (
    !r.summary ||
    ![
      r.summary.dt_s,
      r.summary.all_airborne_duration_s,
      r.summary.max_body_height_above_static_road_m,
      r.summary.peak_total_normal_force_n,
    ].every(Number.isFinite) ||
    r.summary.dt_s <= 0
  )
    throw new Error("Invalid contact summary");
  return r;
}
export const toRender = ([x, y, z]: V3): V3 => [x, z, -y];
export const toRenderQuaternion = ([w, x, y, z]: ContactSample["quaternion"]) =>
  new Quaternion(x, z, -y, w);
export function sampleContact(
  recording: Pick<ContactRecording, "samples">,
  time: number,
): ContactSample {
  const rows = recording.samples;
  if (time <= rows[0].time) return rows[0];
  if (time >= rows.at(-1)!.time) return rows.at(-1)!;
  let lo = 0,
    hi = rows.length - 1;
  while (lo + 1 < hi) {
    const mid = (lo + hi) >> 1;
    if (rows[mid].time <= time) lo = mid;
    else hi = mid;
  }
  const a = rows[lo],
    b = rows[hi],
    f = (time - a.time) / (b.time - a.time);
  const mix = (x: V3, y: V3) => x.map((v, i) => v + f * (y[i] - v)) as V3;
  const q = new Quaternion(
    a.quaternion[1],
    a.quaternion[2],
    a.quaternion[3],
    a.quaternion[0],
  );
  q.slerp(
    new Quaternion(
      b.quaternion[1],
      b.quaternion[2],
      b.quaternion[3],
      b.quaternion[0],
    ),
    f,
  );
  // Continuous pose only; force/contact telemetry is held from the recorded left sample.
  return {
    ...a,
    time,
    position: mix(a.position, b.position),
    quaternion: [q.w, q.x, q.y, q.z],
    wheels: a.wheels.map((w, i) => ({
      ...w,
      center: mix(w.center, b.wheels[i].center),
      patch: mix(w.patch, b.wheels[i].patch),
    })),
  };
}
