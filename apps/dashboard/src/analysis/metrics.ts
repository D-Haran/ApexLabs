import {
  wheels,
  type Session,
  type Sample,
  type Lap,
  type Spatial,
} from "../telemetry/contract";
import { sampleAt, lowerBound } from "../telemetry/replay";
export const utilization = (r: Sample) =>
  Math.max(...wheels.map((w) => r[`friction_utilization_${w}`]));
export interface Event {
  label: string;
  time: number;
  s: number;
  value: number;
  unit: string;
  wheel?: string;
}
export const lapRows = (session: Session, lap: Lap) =>
  session.samples.filter(
    (r) => r.time_s >= lap.start_time_s && r.time_s <= lap.end_time_s,
  );
export function events(session: Session, lap: Lap): Event[] {
  const rows = lapRows(session, lap);
  if (!rows.length) return [];
  const peak = (
    label: string,
    fn: (s: Sample) => number,
    unit: string,
  ): Event => {
    const r = rows.reduce((a, b) => (fn(b) > fn(a) ? b : a));
    return { label, time: r.time_s, s: r.track_s_m, value: fn(r), unit };
  };
  const result = [
    peak("Peak tire utilization", utilization, "%"),
    peak("Peak braking", (r) => -r.longitudinal_accel_m_s2, "m/s²"),
    peak("Peak acceleration", (r) => r.longitudinal_accel_m_s2, "m/s²"),
    peak(
      "Peak lateral acceleration",
      (r) => Math.abs(r.lateral_accel_m_s2),
      "m/s²",
    ),
    peak("Maximum speed", (r) => r.speed_m_s, "m/s"),
    peak("Minimum speed", (r) => -r.speed_m_s, "m/s"),
    peak("Largest path error", (r) => Math.abs(r.lateral_error_m), "m"),
  ];
  result[5].value = -result[5].value;
  // Match M4's lexicographic (utilization, wheel suffix, track s) maximum. No rounding.
  let tirePeak = { value: -Infinity, wheel: "", row: rows[0] };
  for (const row of rows)
    for (const w of wheels) {
      const value = row[`friction_utilization_${w}`];
      if (
        value > tirePeak.value ||
        (value === tirePeak.value &&
          (w > tirePeak.wheel ||
            (w === tirePeak.wheel && row.track_s_m > tirePeak.row.track_s_m)))
      )
        tirePeak = { value, wheel: w, row };
    }
  result[0] = {
    label: "Peak tire utilization",
    time: tirePeak.row.time_s,
    s: tirePeak.row.track_s_m,
    value: tirePeak.value,
    unit: "%",
    wheel: tirePeak.wheel.toUpperCase(),
  };
  for (let i = 1; i < rows.length; i++)
    for (const w of wheels)
      if (rows[i][`tire_saturated_${w}`] && !rows[i - 1][`tire_saturated_${w}`])
        result.push({
          label: `${w.toUpperCase()} saturation onset`,
          time: rows[i].time_s,
          s: rows[i].track_s_m,
          value: rows[i][`friction_utilization_${w}`],
          unit: "%",
          wheel: w.toUpperCase(),
        });
  return result;
}
export function compatible(a: Session, b: Session) {
  if (
    a.manifest.track.reference_offset_m !==
      b.manifest.track.reference_offset_m ||
    a.manifest.track.length_m !== b.manifest.track.length_m ||
    a.geometry.length !== b.geometry.length ||
    a.geometry.some((r, i) =>
      Object.keys(r).some(
        (k) => r[k as keyof typeof r] !== b.geometry[i][k as keyof typeof r],
      ),
    )
  )
    throw new Error(
      "Comparison requires identical track geometry and reference line",
    );
}
export function spatialAt(rows: Spatial[], s: number): Spatial | null {
  if (s < rows[0].s_m || s > rows.at(-1)!.s_m) return null;
  const i = lowerBound(rows, s, (r) => r.s_m);
  if (i === 0) return rows[0];
  const a = rows[i - 1],
    b = rows[i],
    f = (s - a.s_m) / (b.s_m - a.s_m),
    out = { ...a };
  for (const k of Object.keys(a) as (keyof Spatial)[])
    if (k !== "schema_version" && k !== "lap_number")
      out[k] = a[k] + f * (b[k] - a[k]);
  return out;
}
export interface Delta {
  s: number;
  timeDelta: number;
  speedDelta: number;
}
export function compare(a: Session, al: Lap, b: Session, bl: Lap): Delta[] {
  compatible(a, b);
  const ar = a.spatial.filter((r) => r.lap_number === al.number),
    br = b.spatial.filter((r) => r.lap_number === bl.number);
  return ar.flatMap((r) => {
    const v = spatialAt(br, r.s_m);
    return v
      ? [
          {
            s: r.s_m,
            timeDelta:
              v.time_s - bl.start_time_s - (r.time_s - al.start_time_s),
            speedDelta: v.speed_m_s - r.speed_m_s,
          },
        ]
      : [];
  });
}

/** Events derived from two recorded laps. B is conventionally the optimized replay. */
export function optimizationEvents(
  a: Session,
  al: Lap,
  b: Session,
  bl: Lap,
): Event[] {
  const delta = compare(a, al, b, bl);
  const window = Math.max(
    1,
    Math.round(25 / Math.max(1, a.manifest.track.length_m / delta.length)),
  );
  let gain = { value: Infinity, index: window };
  for (let i = window; i < delta.length; i++) {
    const change = delta[i].timeDelta - delta[i - window].timeDelta;
    if (change < gain.value) gain = { value: change, index: i };
  }
  const optimized = lapRows(b, bl);
  const peakUse = optimized.reduce((left, right) =>
    utilization(right) > utilization(left) ? right : left,
  );
  const largestDeviation = delta.reduce(
    (best, row) => {
      const ar = spatialAt(
        a.spatial.filter((v) => v.lap_number === al.number),
        row.s,
      );
      const br = spatialAt(
        b.spatial.filter((v) => v.lap_number === bl.number),
        row.s,
      );
      if (!ar || !br) return best;
      const ap = sampleAt(a, ar.time_s),
        bp = sampleAt(b, br.time_s);
      const distance = Math.hypot(
        bp.position_x_m - ap.position_x_m,
        bp.position_y_m - ap.position_y_m,
      );
      return distance > best.value ? { value: distance, s: row.s } : best;
    },
    { value: 0, s: 0 },
  );
  const brakingOnsets = (session: Session, lap: Lap) => {
    const rows = session.spatial.filter((row) => row.lap_number === lap.number);
    return rows.filter(
      (row, index) =>
        row.brake > 0.05 && (index === 0 || rows[index - 1].brake <= 0.05),
    );
  };
  const referenceBrakes = brakingOnsets(a, al),
    optimizedBrakes = brakingOnsets(b, bl);
  let brakingShift = { value: 0, s: 0 };
  for (const onset of optimizedBrakes) {
    if (!referenceBrakes.length) break;
    const nearest = referenceBrakes.reduce((left, right) =>
      Math.abs(right.s_m - onset.s_m) < Math.abs(left.s_m - onset.s_m)
        ? right
        : left,
    );
    const shift = onset.s_m - nearest.s_m;
    if (Math.abs(shift) > Math.abs(brakingShift.value))
      brakingShift = { value: shift, s: onset.s_m };
  }
  const timeAtA = (s: number) => {
    const row = spatialAt(
      a.spatial.filter((v) => v.lap_number === al.number),
      s,
    );
    return row?.time_s ?? al.start_time_s;
  };
  return [
    {
      label: "Largest time-gain region",
      s: delta[gain.index].s,
      time: timeAtA(delta[gain.index].s),
      value: -gain.value,
      unit: "s / 25 m",
    },
    {
      label: "Largest braking shift",
      s: brakingShift.s,
      time: timeAtA(brakingShift.s),
      value: brakingShift.value,
      unit: "m",
    },
    {
      label: "Highest optimized tire use",
      s: peakUse.track_s_m,
      time: timeAtA(peakUse.track_s_m),
      value: utilization(peakUse),
      unit: "%",
    },
    {
      label: "Largest trajectory deviation",
      s: largestDeviation.s,
      time: timeAtA(largestDeviation.s),
      value: largestDeviation.value,
      unit: "m",
    },
  ];
}
export function summary(rows: Sample[]) {
  return {
    mean: rows.reduce((a, r) => a + r.speed_m_s, 0) / rows.length,
    max: Math.max(...rows.map((r) => r.speed_m_s)),
    min: Math.min(...rows.map((r) => r.speed_m_s)),
    lat: Math.max(...rows.map((r) => Math.abs(r.lateral_accel_m_s2))),
    braking: -Math.min(...rows.map((r) => r.longitudinal_accel_m_s2)),
    accel: Math.max(...rows.map((r) => r.longitudinal_accel_m_s2)),
    util: Math.max(...rows.map(utilization)),
  };
}
export function region(session: Session, lap: Lap, start: number, end: number) {
  const grid = session.spatial.filter((r) => r.lap_number === lap.number);
  const a = spatialAt(grid, start),
    b = spatialAt(grid, end);
  if (!a || !b || end <= start)
    throw new Error(
      `Choose increasing bounds inside ${grid[0].s_m}–${grid.at(-1)!.s_m} m`,
    );
  const rows = [
    sampleAt(session, a.time_s),
    ...session.samples.filter(
      (r) => r.time_s > a.time_s && r.time_s < b.time_s,
    ),
    sampleAt(session, b.time_s),
  ];
  return {
    ...summary(rows),
    entry: a.speed_m_s,
    exit: b.speed_m_s,
    time: b.time_s - a.time_s,
  };
}
