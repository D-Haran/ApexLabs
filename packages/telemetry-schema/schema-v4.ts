/** Single source of field names for parsing, typing and interpolation. SI internally. */
export const baseFields = [
  "schema_version",
  "time_s",
  "step",
  "position_x_m",
  "position_y_m",
  "yaw_rad",
  "yaw_rate_rad_s",
  "vx_m_s",
  "vy_m_s",
  "speed_m_s",
  "steering_angle_rad",
  "throttle",
  "brake",
  "longitudinal_accel_m_s2",
  "lateral_accel_m_s2",
  "track_s_m",
  "unwrapped_track_s_m",
  "track_progress_fraction",
  "lateral_error_m",
  "heading_error_rad",
  "reference_curvature_1_m",
  "target_speed_m_s",
  "speed_error_m_s",
  "lap_number",
  "lap_elapsed_time_s",
  "sector_index",
  "on_track",
] as const;
export const wheels = ["fl", "fr", "rl", "rr"] as const;
export type Wheel = (typeof wheels)[number];
export type WheelField =
  | `fz_${Wheel}_n`
  | `fx_${Wheel}_n`
  | `fy_${Wheel}_n`
  | `slip_angle_${Wheel}_rad`
  | `force_capacity_${Wheel}_n`
  | `tire_saturated_${Wheel}`;
export const wheelFields = wheels.flatMap(
  (w) =>
    [
      `fz_${w}_n`,
      `fx_${w}_n`,
      `fy_${w}_n`,
      `slip_angle_${w}_rad`,
      `force_capacity_${w}_n`,
      `tire_saturated_${w}`,
    ] as WheelField[],
);
export const utilizationFields = wheels.map(
  (w) => `friction_utilization_${w}` as const,
);
export const telemetryFields = [...baseFields, ...utilizationFields];
export const allFields = [...telemetryFields, ...wheelFields];
export type Field = (typeof allFields)[number];
export const chassisFields = [
  "heave_m",
  "roll_rad",
  "pitch_rad",
  "heave_rate_m_s",
  "roll_rate_rad_s",
  "pitch_rate_rad_s",
  "compression_fl_m",
  "compression_fr_m",
  "compression_rl_m",
  "compression_rr_m",
  "gravity_x_m_s2",
  "gravity_y_m_s2",
  "gravity_z_m_s2",
] as const;
export type Chassis = Record<(typeof chassisFields)[number], number>;
export type Sample = Record<Field, number> & {
  chassis?: Chassis;
  visual_distance_m?: number;
  visual_wheel_distance_m?: number[];
};
export const discreteFields = new Set<Field>([
  "schema_version",
  "step",
  "lap_number",
  "sector_index",
  "on_track",
  ...wheels.map((w) => `tire_saturated_${w}` as const),
]);
export const angleFields = new Set<Field>(["yaw_rad", "heading_error_rad"]);
export const geometryFields = [
  "s_m",
  "x_m",
  "y_m",
  "heading_rad",
  "curvature_1_m",
  "left_width_m",
  "right_width_m",
  "left_x_m",
  "left_y_m",
  "right_x_m",
  "right_y_m",
] as const;
export type Geometry = Record<(typeof geometryFields)[number], number>;
export type ElevatedGeometry = Geometry & {
  elevation_m: number;
  grade: number;
};
export const spatialFields = [
  "schema_version",
  "lap_number",
  "s_m",
  "time_s",
  "speed_m_s",
  "throttle",
  "brake",
  "steering_angle_rad",
  "lateral_accel_m_s2",
  "maximum_tire_utilization",
  "lateral_error_m",
] as const;
export type Spatial = Record<(typeof spatialFields)[number], number>;
export interface Lap {
  number: number;
  start_time_s: number;
  end_time_s: number;
  lap_time_s: number;
  sector_times_s: number[];
}
export interface Manifest {
  session_kind?: "validation_scene";
  schema_version: 1;
  telemetry_schema_version: 4;
  wheel_schema_version: 1;
  spatial_schema_version: 1;
  session_name: string;
  files: {
    telemetry: string;
    wheels: string;
    geometry: string;
    spatial: string;
    render?: string;
    chassis?: string;
  };
  vehicle: {
    name: string;
    mass_kg: number;
    mu_reference: number;
    front_axle_m: number;
    rear_axle_m: number;
    front_track_m: number;
    rear_track_m: number;
    provenance: string;
    normal_load_model?: "quasi_static" | "sprung_body";
    cg_height_m?: number;
    suspension?: Record<string, unknown>;
    aero?: {
      air_density_kgpm3: number;
      drag_coefficient: number;
      lift_coefficient_down: number;
      reference_area_m2: number;
    };
  };
  track: {
    name: string;
    length_m: number;
    reference_offset_m: number;
    geometry_sha256: string;
    sector_boundaries_fraction: number[];
    provenance: string;
    kind?: "real_imported" | "synthetic_validation";
    coordinate_system?: {
      type: string;
      origin_lat_deg: number;
      origin_lon_deg: number;
      origin_elevation_m?: number;
      x_axis: string;
      y_axis: string;
      z_axis: string;
      units: string;
    } | null;
    elevation_source?: string | null;
    elevation_limitation?: string | null;
  };
  simulation: {
    timestep_s: number;
    controller_timestep_s: number;
    integrator: string;
    warmup_laps: number;
  };
  optimization?: {
    kind: "locally_optimized_fixed_line" | "locally_optimized_racing_line";
    profile: string;
    independent_production_replay: boolean;
  };
  laps: Lap[];
}
export interface RenderFeature {
  osm_way_id: number;
  classification: "raceway" | "service_road";
  tags: Record<string, string>;
  points_m: [number, number][];
}
export interface RenderContext {
  curbs?: { start_m: number; end_m: number; side: number }[];
  schema_version: 1;
  purpose: "render_context_only";
  warning: string;
  source: string;
  features: RenderFeature[];
}
export interface Session {
  manifest: Manifest;
  samples: Sample[];
  geometry: ElevatedGeometry[];
  spatial: Spatial[];
  load_ms: number;
  renderContext?: RenderContext;
}
