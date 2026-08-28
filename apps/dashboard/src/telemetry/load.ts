import { patchRollingSpeed } from "./wheelMotion";
import {
  baseFields,
  chassisFields,
  type Chassis,
  telemetryFields,
  wheelFields,
  geometryFields,
  spatialFields,
  wheels,
  type Manifest,
  type Session,
  type Sample,
  type Geometry,
  type ElevatedGeometry,
  type Spatial,
  type RenderContext,
} from "./contract";

export function parseCsv(
  text: string,
  fields: readonly string[],
  label: string,
): Record<string, number>[] {
  const lines = text.trim().split(/\r?\n/);
  const header = lines.shift()?.split(",") ?? [];
  if (new Set(header).size !== header.length)
    throw new Error(`${label}: duplicate columns`);
  for (const field of fields)
    if (!header.includes(field)) throw new Error(`${label}: missing ${field}`);
  const rows = lines.map((line, index) => {
    const cells = line.split(",");
    if (cells.length !== header.length)
      throw new Error(`${label}: malformed row ${index + 2}`);
    const row: Record<string, number> = {};
    header.forEach((key, i) => {
      if (cells[i].trim() === "" || !Number.isFinite(Number(cells[i])))
        throw new Error(
          `${label}: missing/nonfinite ${key} at row ${index + 2}`,
        );
      row[key] = Number(cells[i]);
    });
    return row;
  });
  if (rows.length < 2)
    throw new Error(`${label}: requires at least two samples`);
  return rows;
}
export function validateManifest(value: unknown): Manifest {
  const m = value as Manifest;
  if (!m || m.schema_version !== 1)
    throw new Error("Unsupported session schema; expected version 1");
  if (
    m.telemetry_schema_version !== 4 ||
    m.wheel_schema_version !== 1 ||
    m.spatial_schema_version !== 1
  )
    throw new Error(
      "Unsupported telemetry/wheel/spatial schema; expected 4/1/1",
    );
  const numeric = (n: unknown, label: string, positive = false) => {
    if (typeof n !== "number" || !Number.isFinite(n) || (positive && n <= 0))
      throw new Error(`Manifest: invalid ${label}`);
  };
  if (
    !m.session_name ||
    !m.vehicle?.name ||
    !m.track?.name ||
    !m.files ||
    !m.simulation
  )
    throw new Error("Manifest: missing session metadata");
  for (const key of ["telemetry", "wheels", "geometry", "spatial"] as const) {
    if (typeof m.files[key] !== "string" || !/^[\w.-]+$/.test(m.files[key]))
      throw new Error(`Manifest: ${key} must be a sibling filename`);
  }
  for (const key of ["chassis", "render"] as const) {
    const file = m.files[key];
    if (
      file !== undefined &&
      (typeof file !== "string" || !/^[\w.-]+$/.test(file))
    )
      throw new Error(`Manifest: ${key} must be a sibling filename`);
  }
  if (
    m.vehicle.normal_load_model &&
    !["quasi_static", "sprung_body"].includes(m.vehicle.normal_load_model)
  )
    throw new Error("Unknown normal load model");
  if (m.vehicle.normal_load_model === "sprung_body" && !m.files.chassis)
    throw new Error("Sprung-body session requires chassis companion");
  for (const key of [
    "mass_kg",
    "mu_reference",
    "front_axle_m",
    "rear_axle_m",
    "front_track_m",
    "rear_track_m",
  ] as const)
    numeric(m.vehicle[key], key, true);
  numeric(m.track.length_m, "track length", true);
  numeric(m.track.reference_offset_m, "reference offset");
  numeric(m.simulation.timestep_s, "timestep", true);
  numeric(m.simulation.controller_timestep_s, "controller timestep", true);
  const sectors = m.track.sector_boundaries_fraction;
  if (
    !Array.isArray(sectors) ||
    !sectors.length ||
    sectors.at(-1) !== 1 ||
    sectors.some(
      (v, i) =>
        !Number.isFinite(v) ||
        v <= 0 ||
        v > 1 ||
        (i > 0 && v <= sectors[i - 1]),
    )
  )
    throw new Error("Manifest: invalid sector boundaries");
  if (!Array.isArray(m.laps) || !m.laps.length)
    throw new Error("Manifest: no completed laps");
  for (const [i, lap] of m.laps.entries()) {
    numeric(lap.start_time_s, "lap start");
    numeric(lap.end_time_s, "lap end");
    numeric(lap.lap_time_s, "lap time", true);
    if (
      !Number.isInteger(lap.number) ||
      lap.number < 0 ||
      (i > 0 && lap.number <= m.laps[i - 1].number) ||
      Math.abs(lap.end_time_s - lap.start_time_s - lap.lap_time_s) > 1e-6
    )
      throw new Error("Manifest: inconsistent lap interval");
    if (
      !Array.isArray(lap.sector_times_s) ||
      lap.sector_times_s.length !== sectors.length ||
      lap.sector_times_s.some((n) => !Number.isFinite(n) || n <= 0) ||
      Math.abs(lap.sector_times_s.reduce((a, b) => a + b, 0) - lap.lap_time_s) >
        1e-6
    )
      throw new Error("Manifest: invalid sector timing");
  }
  return m;
}
export function decodeSession(
  manifest: unknown,
  files: Record<string, string>,
): Session {
  const m = validateManifest(manifest);
  const read = (
    name: "telemetry" | "wheels" | "geometry" | "spatial",
    fields: readonly string[],
  ) => {
    const filename = m.files[name];
    const file = files[filename];
    if (file === undefined) throw new Error(`Missing file: ${filename}`);
    return parseCsv(file, fields, name);
  };
  const raw = read("telemetry", telemetryFields),
    tire = read("wheels", ["schema_version", "time_s", "step", ...wheelFields]);
  if (raw.length !== tire.length)
    throw new Error("Wheel companion sample count mismatch");
  const samples = raw.map((r, i) => {
    const w = tire[i];
    if (r.schema_version !== 4 || w.schema_version !== 1)
      throw new Error("Unsupported telemetry row schema");
    if (r.time_s !== w.time_s || r.step !== w.step)
      throw new Error("Wheel companion timestamp/step mismatch");
    if (
      i &&
      (r.time_s <= raw[i - 1].time_s ||
        r.unwrapped_track_s_m < raw[i - 1].unwrapped_track_s_m ||
        r.step <= raw[i - 1].step)
    )
      throw new Error("Nonmonotonic time, progress or step");
    for (const key of [
      "on_track",
      ...wheels.map((v) => `tire_saturated_${v}`),
    ]) {
      const val = key === "on_track" ? r[key] : w[key];
      if (val !== 0 && val !== 1) throw new Error(`Invalid flag ${key}`);
    }
    for (const key of ["step", "lap_number", "sector_index"])
      if (!Number.isInteger(r[key]) || r[key] < 0)
        throw new Error(`Invalid discrete field ${key}`);
    if (
      r.throttle < 0 ||
      r.throttle > 1 ||
      r.brake < 0 ||
      r.brake > 1 ||
      r.track_s_m < 0 ||
      r.track_s_m >= m.track.length_m + 1e-6
    )
      throw new Error("Invalid controls or track position");
    for (const v of wheels)
      if (
        w[`fz_${v}_n`] < 0 ||
        w[`force_capacity_${v}_n`] < 0 ||
        r[`friction_utilization_${v}`] < 0 ||
        r[`friction_utilization_${v}`] > 1 + 1e-8
      )
        throw new Error(`Invalid tire ${v}`);
    return {
      ...r,
      ...Object.fromEntries(wheelFields.map((k) => [k, w[k]])),
    } as unknown as Sample;
  });
  if (m.files.chassis) {
    if (!/^[\w.-]+$/.test(m.files.chassis))
      throw new Error("Chassis file must be a sibling filename");
    const text = files[m.files.chassis];
    if (text === undefined) throw new Error("Missing chassis companion");
    const rows = parseCsv(
      text,
      ["schema_version", "time_s", "step", ...chassisFields],
      "chassis",
    );
    if (rows.length !== samples.length)
      throw new Error("Chassis sample count mismatch");
    rows.forEach((row, i) => {
      if (
        row.schema_version !== 1 ||
        row.time_s !== samples[i].time_s ||
        row.step !== samples[i].step
      )
        throw new Error("Chassis companion version/timestamp/step mismatch");
      samples[i].chassis = Object.fromEntries(
        chassisFields.map((k) => [k, row[k]]),
      ) as Chassis;
    });
  } else if (m.vehicle.normal_load_model === "sprung_body")
    throw new Error("Sprung-body session requires chassis companion");
  let distance = 0;
  const wheelDistance = [0, 0, 0, 0];
  samples.forEach((sample, i) => {
    if (i)
      distance +=
        ((sample.time_s - samples[i - 1].time_s) *
          (sample.speed_m_s + samples[i - 1].speed_m_s)) /
        2;
    sample.visual_distance_m = distance;
    wheelDistance.forEach((_, w) => {
      if (i)
        wheelDistance[w] +=
          ((sample.time_s - samples[i - 1].time_s) *
            (patchRollingSpeed(sample, m.vehicle, w) +
              patchRollingSpeed(samples[i - 1], m.vehicle, w))) /
          2;
    });
    sample.visual_wheel_distance_m = [...wheelDistance];
  });
  const geometry = (
    read("geometry", geometryFields) as (Geometry & Partial<ElevatedGeometry>)[]
  ).map(
    (row) =>
      ({
        ...row,
        elevation_m: row.elevation_m ?? 0,
        grade: row.grade ?? 0,
      }) as ElevatedGeometry,
  );
  if (
    Math.abs(geometry.at(-1)!.s_m - m.track.length_m) > 1e-5 ||
    geometry[0].s_m !== 0
  )
    throw new Error("Geometry length mismatch");
  geometry.forEach((r, i) => {
    if (
      (i && r.s_m <= geometry[i - 1].s_m) ||
      r.left_width_m <= 0 ||
      r.right_width_m <= 0
    )
      throw new Error("Invalid track geometry");
  });
  const spatial = read("spatial", spatialFields) as Spatial[];
  for (const lap of m.laps) {
    if (
      samples[0].time_s > lap.start_time_s + 1e-8 ||
      samples.at(-1)!.time_s < lap.end_time_s - 1e-8
    )
      throw new Error("Telemetry does not cover completed lap");
    const rows = spatial.filter((r) => r.lap_number === lap.number);
    if (rows.length < 2) throw new Error("Missing spatial lap");
    rows.forEach((r, i) => {
      if (
        r.schema_version !== 1 ||
        r.s_m < 0 ||
        r.s_m > m.track.length_m ||
        r.time_s < lap.start_time_s ||
        r.time_s > lap.end_time_s ||
        (i && (r.s_m <= rows[i - 1].s_m || r.time_s <= rows[i - 1].time_s))
      )
        throw new Error("Invalid spatial schema/order/coverage");
    });
  }
  let renderContext: RenderContext | undefined;
  if (m.files.render) {
    const rawContext = files[m.files.render];
    if (rawContext === undefined)
      throw new Error(`Missing file: ${m.files.render}`);
    const value = JSON.parse(rawContext) as RenderContext;
    if (
      value.schema_version !== 1 ||
      value.purpose !== "render_context_only" ||
      !Array.isArray(value.features)
    )
      throw new Error("Unsupported render-context schema");
    renderContext = value;
  }
  return { manifest: m, samples, geometry, spatial, load_ms: 0, renderContext };
}
export { baseFields };
