import { Matrix4, Quaternion, Vector3 } from "three";
import type { Session } from "../telemetry/contract";
import { angleMix } from "../telemetry/replay";
export function roadAt(session: Session, progress: number) {
  const rows = session.geometry,
    L = session.manifest.track.length_m,
    s = ((progress % L) + L) % L;
  let lo = 0,
    hi = rows.length - 1;
  while (lo + 1 < hi) {
    const mid = (lo + hi) >> 1;
    if (rows[mid].s_m <= s) lo = mid;
    else hi = mid;
  }
  const a = rows[lo],
    b = rows[hi],
    f = (s - a.s_m) / (b.s_m - a.s_m);
  return {
    elevation: a.elevation_m + f * (b.elevation_m - a.elevation_m),
    heading: angleMix(a.heading_rad, b.heading_rad, f),
    grade: a.grade + f * (b.grade - a.grade),
  };
}
/** Three +X forward,+Y up,+Z right. Physics heading is its horizontal projection. */
export function roadQuaternion(heading: number, grade: number, yaw = heading) {
  const q = 1 / Math.hypot(1, grade),
    t = new Vector3(Math.cos(heading) * q, grade * q, -Math.sin(heading) * q),
    l = new Vector3(-Math.sin(heading), 0, -Math.cos(heading));
  const n = new Vector3().crossVectors(t, l).normalize();
  const difference = yaw - heading,
    e = Math.atan2(q * Math.sin(difference), Math.cos(difference));
  const forward = t
    .clone()
    .multiplyScalar(Math.cos(e))
    .addScaledVector(l, Math.sin(e));
  const right = new Vector3().crossVectors(forward, n).normalize();
  return new Quaternion().setFromRotationMatrix(
    new Matrix4().makeBasis(forward, n, right),
  );
}
export const wheelSpin = (distance: number, radius: number) => {
  if (!Number.isFinite(distance) || !Number.isFinite(radius) || radius <= 0)
    throw new Error("Invalid wheel spin inputs");
  return -distance / radius;
};
/** Exact critically damped update for a target held over the frame. */
export function springStep(
  position: Vector3,
  velocity: Vector3,
  target: Vector3,
  dt: number,
  omega = 9,
) {
  const d = Math.max(0, Math.min(dt, 0.1)),
    decay = Math.exp(-omega * d),
    error = position.clone().sub(target),
    j = velocity.clone().addScaledVector(error, omega);
  position.copy(target).add(error.addScaledVector(j, d).multiplyScalar(decay));
  velocity.addScaledVector(j, -omega * d).multiplyScalar(decay);
}
/** Height on the same piecewise-linear, unbanked road used by the render ribbon. */
export function roadSurfaceHeight(
  session: Session,
  x: number,
  z: number,
  progress: number,
) {
  const rows = session.geometry,
    L = session.manifest.track.length_m,
    s = ((progress % L) + L) % L;
  let lo = 0,
    hi = rows.length - 1;
  while (lo + 1 < hi) {
    const mid = (lo + hi) >> 1;
    if (rows[mid].s_m <= s) lo = mid;
    else hi = mid;
  }
  let best = Infinity,
    height = rows[lo].elevation_m;
  for (let k = -4; k <= 4; k++) {
    const i = (lo + k + rows.length - 1) % (rows.length - 1),
      a = rows[i],
      b = rows[i + 1];
    const dx = b.x_m - a.x_m,
      dy = b.y_m - a.y_m,
      f = Math.max(
        0,
        Math.min(
          1,
          ((x - a.x_m) * dx + (-z - a.y_m) * dy) / (dx * dx + dy * dy),
        ),
      );
    const d = (x - a.x_m - f * dx) ** 2 + (-z - a.y_m - f * dy) ** 2;
    if (d < best) {
      best = d;
      height = a.elevation_m + f * (b.elevation_m - a.elevation_m);
    }
  }
  return height;
}
