import {
  allFields,
  chassisFields,
  type Chassis,
  angleFields,
  discreteFields,
  type Sample,
  type Session,
  type Lap,
} from "./contract";
export function lowerBound<T>(
  rows: T[],
  value: number,
  key: (r: T) => number,
): number {
  let lo = 0,
    hi = rows.length;
  while (lo < hi) {
    const mid = (lo + hi) >>> 1;
    if (key(rows[mid]) < value) lo = mid + 1;
    else hi = mid;
  }
  return lo;
}
export const angleMix = (a: number, b: number, f: number) =>
  a + Math.atan2(Math.sin(b - a), Math.cos(b - a)) * f;
export function interpolate(
  a: Sample,
  b: Sample,
  t: number,
  length: number,
): Sample {
  if (t <= a.time_s) return a;
  if (t >= b.time_s) return b;
  const f = (t - a.time_s) / (b.time_s - a.time_s),
    result = {} as Sample;
  for (const k of allFields)
    result[k] = discreteFields.has(k)
      ? a[k]
      : angleFields.has(k)
        ? angleMix(a[k], b[k], f)
        : a[k] + f * (b[k] - a[k]);
  if (a.chassis && b.chassis)
    result.chassis = Object.fromEntries(
      chassisFields.map((k) => [
        k,
        a.chassis![k] + f * (b.chassis![k] - a.chassis![k]),
      ]),
    ) as Chassis;
  if (a.visual_distance_m !== undefined && b.visual_distance_m !== undefined)
    result.visual_distance_m =
      a.visual_distance_m + f * (b.visual_distance_m - a.visual_distance_m);
  if (a.visual_wheel_distance_m && b.visual_wheel_distance_m)
    result.visual_wheel_distance_m = a.visual_wheel_distance_m.map(
      (v, i) => v + f * (b.visual_wheel_distance_m![i] - v),
    );
  result.time_s = t;
  result.track_s_m = ((result.unwrapped_track_s_m % length) + length) % length;
  result.track_progress_fraction = result.track_s_m / length;
  // Elapsed time resets at a recorded lap boundary; never blend the reset.
  if (
    a.lap_number !== b.lap_number ||
    b.lap_elapsed_time_s < a.lap_elapsed_time_s
  )
    result.lap_elapsed_time_s = a.lap_elapsed_time_s + (t - a.time_s);
  return result;
}
export function sampleAt(session: Session, time: number): Sample {
  const rows = session.samples,
    i = lowerBound(rows, time, (r) => r.time_s);
  if (i === 0) return rows[0];
  if (i === rows.length) return rows.at(-1)!;
  return interpolate(
    rows[i - 1],
    rows[i],
    time,
    session.manifest.track.length_m,
  );
}
export function seekDistance(session: Session, lap: Lap, s: number): number {
  const length = session.manifest.track.length_m;
  if (s <= 0) return lap.start_time_s;
  if (s >= length) return lap.end_time_s;
  const target = lap.number * length + s;
  const rows = session.samples,
    i = lowerBound(rows, target, (r) => r.unwrapped_track_s_m);
  if (i === 0) return lap.start_time_s;
  if (i === rows.length) return lap.end_time_s;
  const a = rows[i - 1],
    b = rows[i],
    d = b.unwrapped_track_s_m - a.unwrapped_track_s_m;
  const t =
    d === 0
      ? a.time_s
      : a.time_s +
        ((target - a.unwrapped_track_s_m) / d) * (b.time_s - a.time_s);
  return Math.max(lap.start_time_s, Math.min(lap.end_time_s, t));
}
export function lapDistance(session: Session, lap: Lap, time: number): number {
  if (time <= lap.start_time_s) return 0;
  if (time >= lap.end_time_s) return session.manifest.track.length_m;
  return Math.max(
    0,
    Math.min(
      session.manifest.track.length_m,
      sampleAt(session, time).unwrapped_track_s_m -
        lap.number * session.manifest.track.length_m,
    ),
  );
}
