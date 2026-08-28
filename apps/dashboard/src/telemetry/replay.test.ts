import { describe, it, expect } from "vitest";
import { readFileSync } from "node:fs";
import { resolve } from "node:path";
import {
  allFields,
  geometryFields,
  wheels,
  type ElevatedGeometry,
  type Sample,
  type Session,
  type Lap,
  type Spatial,
} from "./contract";
import { decodeSession, parseCsv, validateManifest } from "./load";
import {
  interpolate,
  angleMix,
  sampleAt,
  seekDistance,
  lapDistance,
} from "./replay";
import { ReplayStore } from "../state/replay";
import { events, compare, region, compatible } from "../analysis/metrics";
import { renderTrackPoint } from "../three/TrackEnvironment";
const dataDir = resolve(import.meta.dirname, "../../public/demo");
function fixture(name = "baseline") {
  const manifest = JSON.parse(
    readFileSync(`${dataDir}/${name}/session.json`, "utf8"),
  );
  const files = Object.fromEntries(
    Object.values(manifest.files).map((file) => [
      file,
      readFileSync(`${dataDir}/${name}/${file}`, "utf8"),
    ]),
  );
  return { manifest, files };
}
const f = fixture(),
  real = decodeSession(f.manifest, f.files);
const lap = real.manifest.laps[0];
function sample(t: number, x: number): Sample {
  const r = Object.fromEntries(allFields.map((k) => [k, 0])) as unknown as Sample;
  return {
    ...r,
    schema_version: 4,
    time_s: t,
    step: t,
    position_x_m: x,
    speed_m_s: 10,
    track_s_m: x,
    unwrapped_track_s_m: x,
    on_track: 1,
  };
}
function synthetic(duration: number): Session {
  const rows = [0, 1, 2, 3, 4].map((i) => sample((i * duration) / 4, i * 25));
  const lap: Lap = {
    number: 0,
    start_time_s: 0,
    end_time_s: duration,
    lap_time_s: duration,
    sector_times_s: [duration],
  };
  return {
    ...real,
    manifest: {
      ...real.manifest,
      track: { ...real.manifest.track, length_m: 100 },
      laps: [lap],
    },
    samples: rows,
    spatial: rows.map((r) => ({
      schema_version: 1,
      lap_number: 0,
      s_m: r.track_s_m,
      time_s: r.time_s,
      speed_m_s: r.speed_m_s,
      throttle: 0,
      brake: 0,
      steering_angle_rad: 0,
      lateral_accel_m_s2: 0,
      maximum_tire_utilization: 0,
      lateral_error_m: 0,
    })),
  };
}
describe("strict versioned boundary", () => {
  it("loads actual M4 reproduction with all tire forces and exact lap result", () => {
    expect(real.samples.length).toBeGreaterThan(10000);
    expect(lap.lap_time_s).toBeCloseTo(50.71, 6);
    expect(real.samples.every((r) => r.fz_rl_n > 0)).toBe(true);
  });
  it("rejects unsupported manifest version", () =>
    expect(() =>
      validateManifest({ ...f.manifest, schema_version: 2 }),
    ).toThrow("Unsupported session"));
  it("rejects unsupported telemetry version", () =>
    expect(() =>
      validateManifest({ ...f.manifest, telemetry_schema_version: 5 }),
    ).toThrow("Unsupported telemetry"));
  it("rejects missing companion file", () => {
    const files = { ...f.files };
    delete files["wheels.csv"];
    expect(() => decodeSession(f.manifest, files)).toThrow("Missing file");
  });
  it("rejects missing channel instead of substituting zero", () =>
    expect(() =>
      parseCsv("time_s,speed\n0,1\n1,2", ["time_s", "fz_fl_n"], "wheel"),
    ).toThrow("missing fz_fl_n"));
  it("rejects empty numeric values", () =>
    expect(() => parseCsv("time_s,speed\n0,\n1,2", ["speed"], "data")).toThrow(
      "missing/nonfinite",
    ));
  it("rejects NaN / Infinity", () =>
    expect(() =>
      parseCsv("time_s,speed\n0,NaN\n1,2", ["speed"], "data"),
    ).toThrow("nonfinite"));
  it("rejects duplicate columns", () =>
    expect(() =>
      parseCsv("time_s,time_s\n0,1\n1,2", ["time_s"], "data"),
    ).toThrow("duplicate"));
  it("rejects nonlocal file paths", () =>
    expect(() =>
      validateManifest({
        ...f.manifest,
        files: { ...f.manifest.files, wheels: "../other.csv" },
      }),
    ).toThrow("sibling"));
  it("rejects wheel sample misalignment", () => {
    const files = { ...f.files };
    const lines = files["wheels.csv"].split(/\r?\n/);
    const r = lines[1].split(",");
    r[1] = "0";
    lines[1] = r.join(",");
    files["wheels.csv"] = lines.join("\n");
    expect(() => decodeSession(f.manifest, files)).toThrow("timestamp/step");
  });
  it("rejects missing lap time and inconsistent sectors", () => {
    expect(() =>
      validateManifest({
        ...f.manifest,
        laps: [{ ...lap, lap_time_s: undefined }],
      }),
    ).toThrow("lap time");
    expect(() =>
      validateManifest({
        ...f.manifest,
        laps: [{ ...lap, sector_times_s: [1] }],
      }),
    ).toThrow("sector");
  });
});
describe("deterministic interpolation and synchronization", () => {
  it("interpolates position, scalars and keeps one timestamp", () => {
    const a = sample(0, 0),
      b = { ...sample(2, 20), speed_m_s: 30 };
    const r = interpolate(a, b, 0.5, 100);
    expect(r.time_s).toBe(0.5);
    expect(r.position_x_m).toBe(5);
    expect(r.speed_m_s).toBe(15);
  });
  it("uses shortest angle across ±π", () => {
    const a = { ...sample(0, 0), yaw_rad: Math.PI - 0.1 },
      b = { ...sample(2, 20), yaw_rad: -Math.PI + 0.1 };
    expect(interpolate(a, b, 1, 100).yaw_rad).toBeCloseTo(Math.PI);
    expect(angleMix(3, -3, 0.5)).toBeCloseTo(Math.PI);
  });
  it("left-holds flags, lap and sector with exact-boundary switch", () => {
    const a = sample(0, 0),
      b = {
        ...sample(2, 20),
        on_track: 0,
        tire_saturated_rl: 1,
        lap_number: 1,
        sector_index: 2,
      };
    const mid = interpolate(a, b, 1, 100);
    expect(mid.on_track).toBe(1);
    expect(mid.tire_saturated_rl).toBe(0);
    expect(mid.lap_number).toBe(0);
    expect(mid.sector_index).toBe(0);
    expect(interpolate(a, b, 2, 100)).toBe(b);
  });
  it("interpolates progress across the seam without driving backward", () => {
    const a = { ...sample(0, 99), unwrapped_track_s_m: 99 },
      b = { ...sample(2, 1), unwrapped_track_s_m: 101 };
    expect(interpolate(a, b, 1, 100).track_s_m).toBe(0);
    expect(interpolate(a, b, 0.5, 100).track_s_m).toBe(99.5);
  });
  it("clamps endpoints", () => {
    expect(sampleAt(real, -1)).toBe(real.samples[0]);
    expect(sampleAt(real, 1e9)).toBe(real.samples.at(-1));
  });
  it("shares one immutable snapshot for car, map, charts and inspector", () => {
    const store = new ReplayStore();
    store.load(real);
    store.seek(lap.start_time_s + 17.125);
    const views = Array.from({ length: 5 }, () => store.getSnapshot());
    expect(views.every((v) => v === views[0])).toBe(true);
    expect(views[0].sample?.time_s).toBe(views[0].time);
    expect(views[0].sample).toEqual(sampleAt(real, views[0].time));
  });
  it("plays at chosen rate, pauses, restarts and stops at end", () => {
    const s = synthetic(10),
      store = new ReplayStore();
    store.load(s);
    store.setRate(2);
    store.play();
    store.advance(1);
    expect(store.getSnapshot().time).toBe(2);
    store.pause();
    store.advance(1);
    expect(store.getSnapshot().time).toBe(2);
    store.play();
    store.advance(10);
    expect(store.getSnapshot().time).toBe(10);
    expect(store.getSnapshot().playing).toBe(false);
    store.restart();
    expect(store.getSnapshot().time).toBe(0);
  });
  it("steps exact recorded frames in either direction", () => {
    const s = synthetic(10),
      store = new ReplayStore();
    store.load(s);
    store.seek(3);
    store.step(1);
    expect(store.getSnapshot().time).toBe(5);
    store.step(-1);
    expect(store.getSnapshot().time).toBe(2.5);
  });
  it("seeks distance on explicitly selected lap", () => {
    expect(sampleAt(real, seekDistance(real, lap, 775)).track_s_m).toBeCloseTo(
      775,
      7,
    );
    expect(seekDistance(real, lap, 0)).toBe(lap.start_time_s);
    expect(seekDistance(real, lap, real.manifest.track.length_m)).toBe(
      lap.end_time_s,
    );
    expect(lapDistance(real, lap, lap.end_time_s)).toBe(
      real.manifest.track.length_m,
    );
  });
  it("handles multiple laps deterministically", () => {
    const s = synthetic(10);
    const later = {
      number: 1,
      start_time_s: 10,
      end_time_s: 20,
      lap_time_s: 10,
      sector_times_s: [10],
    };
    s.manifest.laps.push(later);
    s.samples.push(
      ...s.samples.slice(1).map((r) => ({
        ...r,
        time_s: r.time_s + 10,
        unwrapped_track_s_m: r.unwrapped_track_s_m + 100,
        track_s_m: r.track_s_m % 100,
        lap_number: 1,
      })),
    );
    expect(seekDistance(s, later, 25)).toBe(12.5);
    expect(seekDistance(s, s.manifest.laps[0], 25)).toBe(2.5);
  });
});
describe("spatial analysis", () => {
  it("matches known tB(s)-tA(s), including sign", () => {
    const a = synthetic(10),
      b = synthetic(8),
      delta = compare(a, a.manifest.laps[0], b, b.manifest.laps[0]);
    expect(delta.map((d) => d.timeDelta)).toEqual([0, -0.5, -1, -1.5, -2]);
  });
  it("interpolates differing spatial grids without extrapolation", () => {
    const a = synthetic(10),
      b = synthetic(8);
    b.spatial = [
      { ...b.spatial[0], s_m: 10, time_s: 0.8 },
      { ...b.spatial[4], s_m: 90, time_s: 7.2 },
    ];
    const delta = compare(a, a.manifest.laps[0], b, b.manifest.laps[0]);
    expect(delta.map((d) => d.s)).toEqual([25, 50, 75]);
    expect(delta[1].timeDelta).toBeCloseTo(-1);
  });
  it("rejects incompatible geometry even with same track name", () => {
    const b = {
      ...real,
      geometry: real.geometry.map((r, i) => (i ? r : { ...r, x_m: r.x_m + 1 })),
    };
    expect(() => compatible(real, b)).toThrow("identical");
  });
  it("summarizes interval boundary speeds and elapsed time", () => {
    const a = synthetic(10),
      r = region(a, a.manifest.laps[0], 25, 75);
    expect(r.time).toBe(5);
    expect(r.entry).toBe(10);
    expect(r.min).toBe(10);
    expect(() => region(a, a.manifest.laps[0], 75, 25)).toThrow("increasing");
  });
  it("detects extrema and saturation onset with stable ties", () => {
    const a = synthetic(10);
    a.samples[2].friction_utilization_rl = 1;
    a.samples[2].tire_saturated_rl = 1;
    a.samples[3].friction_utilization_rl = 1;
    const e = events(a, a.manifest.laps[0]);
    expect(e[0].time).toBe(7.5);
    expect(e[0].wheel).toBe("RL");
    expect(e.filter((v) => v.label === "RL saturation onset")).toHaveLength(1);
  });
  it("discovers real peak around the documented location without a hard-coded seek", () => {
    const e = events(real, lap)[0];
    expect(e.wheel).toBe("RL");
    expect(e.s).toBeGreaterThan(700);
    expect(e.s).toBeLessThan(850);
  });
  it("uses C++ spatial export without wrapped finish contamination", () => {
    const rows = real.spatial;
    expect(rows[0].time_s - lap.start_time_s).toBeLessThan(0.1);
    expect(rows.at(-1)!.time_s - lap.start_time_s).toBeGreaterThan(50);
    for (const r of rows.filter((_, i) => i % 100 === 0)) {
      const s = sampleAt(real, r.time_s);
      expect(s.track_s_m).toBeCloseTo(r.s_m, 7);
      expect(s.speed_m_s).toBeCloseTo(r.speed_m_s, 9);
    }
  });
  it("normalizes force from exported capacity, matching utilization at recorded states", () => {
    for (const s of real.samples.filter((_, i) => i % 100 === 0))
      for (const w of wheels)
        expect(
          Math.hypot(s[`fx_${w}_n`], s[`fy_${w}_n`]) /
            s[`force_capacity_${w}_n`],
        ).toBeCloseTo(s[`friction_utilization_${w}`], 10);
  });
  it("real comparison gain agrees near finish with simulator times", () => {
    const b = fixture("high-grip"),
      s = decodeSession(b.manifest, b.files),
      delta = compare(real, lap, s, s.manifest.laps[0]);
    expect(delta.at(-1)!.timeDelta).toBeCloseTo(
      s.manifest.laps[0].lap_time_s - lap.lap_time_s,
      2,
    );
  });
});

describe("physics/render track alignment", () => {
  it("keeps known Spa physics center points on the elevated render surface", () => {
    const rows = parseCsv(
      readFileSync(`${dataDir}/spa-p1/geometry.csv`, "utf8"),
      geometryFields,
      "spa geometry",
    ) as ElevatedGeometry[];
    for (const index of [0, 117, 406, rows.length - 1]) {
      const row = rows[index];
      const center = renderTrackPoint(row, 0);
      expect(center.x).toBeCloseTo(row.x_m, 10);
      expect(center.y).toBeCloseTo(row.elevation_m, 10);
      expect(center.z).toBeCloseTo(-row.y_m, 10);

      const left = renderTrackPoint(row, 1);
      const right = renderTrackPoint(row, -1);
      const width = row.left_width_m + row.right_width_m;
      expect(
        (left.x * row.right_width_m + right.x * row.left_width_m) / width,
      ).toBeCloseTo(row.x_m, 9);
      expect(
        (left.z * row.right_width_m + right.z * row.left_width_m) / width,
      ).toBeCloseTo(-row.y_m, 9);
    }
  });
});
