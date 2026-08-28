import { replay } from "./replay";
let renderStats = { draw_calls: 0, triangles: 0, textures: 0, geometries: 0 },
  assetStats = {
    asset_name: "",
    asset_ready_ms: 0,
    texture_memory_estimate_bytes: 0,
  };
export function recordRenderStats(stats: typeof renderStats) {
  renderStats = stats;
}
export function recordAssetReady(
  asset_name: string,
  asset_ready_ms: number,
  texture_memory_estimate_bytes: number,
) {
  assetStats = { asset_name, asset_ready_ms, texture_memory_estimate_bytes };
}
interface Metrics {
  frame_samples: number;
  average_frame_ms: number;
  p95_frame_ms: number;
  selector_seek_average_ms: number;
  selector_seek_p95_ms: number;
  seek_to_paint_ms: number;
  session_load_ms: number;
  user_agent: string;
  viewport: string;
}
let previous = 0,
  frames: number[] = [],
  active = false,
  finish: ((v: Metrics) => void) | null = null;
const percentile = (a: number[]) =>
  [...a].sort((x, y) => x - y)[Math.floor((a.length - 1) * 0.95)];
export function recordFrame() {
  if (!active) return;
  const now = performance.now();
  if (previous) frames.push(now - previous);
  previous = now;
  if (frames.length >= 300) {
    active = false;
    const before = replay.getSnapshot(),
      times: number[] = [],
      lap = replay.lap!;
    for (let i = 0; i < 100; i++) {
      const start = performance.now();
      replay.seek(
        lap.start_time_s + (lap.end_time_s - lap.start_time_s) * (i / 99),
      );
      times.push(performance.now() - start);
    }
    const start = performance.now();
    replay.seek(before.time);
    requestAnimationFrame(() =>
      requestAnimationFrame(() => {
        finish?.({
          ...renderStats,
          ...assetStats,
          frame_samples: frames.length,
          average_frame_ms: frames.reduce((a, b) => a + b, 0) / frames.length,
          p95_frame_ms: percentile(frames),
          selector_seek_average_ms:
            times.reduce((a, b) => a + b, 0) / times.length,
          selector_seek_p95_ms: percentile(times),
          seek_to_paint_ms: performance.now() - start,
          session_load_ms: replay.session!.load_ms,
          user_agent: navigator.userAgent,
          viewport: `${innerWidth} × ${innerHeight} @ ${devicePixelRatio}`,
        });
      }),
    );
  }
}
export const measure = () =>
  new Promise<Metrics>((resolve) => {
    previous = 0;
    frames = [];
    finish = resolve;
    active = true;
  });
