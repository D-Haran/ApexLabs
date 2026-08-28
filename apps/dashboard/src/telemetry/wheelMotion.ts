import type { Manifest, Sample } from "./contract";

/** Rigid-body patch velocity projected onto wheel heading; visual use only. */
export function patchRollingSpeed(
  sample: Sample,
  vehicle: Manifest["vehicle"],
  wheel: number,
) {
  const front = wheel < 2;
  const x = front ? vehicle.front_axle_m : -vehicle.rear_axle_m;
  const y =
    ((wheel % 2 ? -1 : 1) *
      (front ? vehicle.front_track_m : vehicle.rear_track_m)) /
    2;
  const delta = front ? sample.steering_angle_rad : 0;
  return (
    (sample.vx_m_s - sample.yaw_rate_rad_s * y) * Math.cos(delta) +
    (sample.vy_m_s + sample.yaw_rate_rad_s * x) * Math.sin(delta)
  );
}
