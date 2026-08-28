export const wheelNames = ["FL", "FR", "RL", "RR"] as const;
export interface VisualWheel {
  steerPivot: string;
  spinPivot: string;
  center: [number, number, number];
  radius: number;
  tireNodes: string[];
  rimNodes: string[];
  discNodes: string[];
  caliperNodes: string[];
}
export interface CarManifest {
  schemaVersion: 1;
  assetName: string;
  bodyNode: string;
  localForwardAxis: string;
  localLeftAxis: string;
  localUpAxis: string;
  wheelbase: number;
  frontTrack: number;
  rearTrack: number;
  groundOffset: number;
  cameraTarget: [number, number, number];
  wheels: Record<(typeof wheelNames)[number], VisualWheel>;
  processedTriangleCount: number;
  textureMemoryEstimateBytes: number;
  source: { author: string; source: string; license: string };
}
export function parseCarManifest(value: unknown): CarManifest {
  const m = value as CarManifest;
  if (
    m?.schemaVersion !== 1 ||
    m.localForwardAxis !== "+X" ||
    m.localLeftAxis !== "-Z" ||
    m.localUpAxis !== "+Y" ||
    !m.bodyNode
  )
    throw new Error("Unsupported vehicle visual manifest");
  for (const v of [m.wheelbase, m.frontTrack, m.rearTrack])
    if (!Number.isFinite(v) || v <= 0)
      throw new Error("Invalid visual dimensions");
  const names = new Set<string>();
  for (const key of wheelNames) {
    const w = m.wheels?.[key];
    if (
      !w ||
      !Number.isFinite(w.radius) ||
      w.radius <= 0 ||
      w.center.length !== 3 ||
      w.center.some((v) => !Number.isFinite(v)) ||
      Math.abs(w.center[1] - w.radius) > 1e-6
    )
      throw new Error("Invalid grounded wheel " + key);
    if (
      (key[0] === "F" ? w.center[0] : -w.center[0]) <= 0 ||
      (key[1] === "L" ? -w.center[2] : w.center[2]) <= 0
    )
      throw new Error("Invalid semantic wheel " + key);
    for (const n of [w.steerPivot, w.spinPivot]) {
      if (!n || names.has(n)) throw new Error("Duplicate wheel pivot");
      names.add(n);
    }
  }
  return m;
}
