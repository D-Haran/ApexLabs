import * as THREE from "three";
import type { ElevatedGeometry } from "../telemetry/contract";
/** Closest piecewise-linear road segment; used only for render context/exclusion. */
export function nearestRoad(
  rows: ElevatedGeometry[],
  x: number,
  y: number,
  step = 1,
) {
  let distance2 = Infinity,
    elevation = 0,
    width = 0;
  for (let i = 0; i < rows.length - 1; i += step) {
    const a = rows[i],
      b = rows[Math.min(i + step, rows.length - 1)],
      dx = b.x_m - a.x_m,
      dy = b.y_m - a.y_m;
    const t = Math.max(
        0,
        Math.min(
          1,
          ((x - a.x_m) * dx + (y - a.y_m) * dy) / (dx * dx + dy * dy || 1),
        ),
      ),
      d = (x - a.x_m - t * dx) ** 2 + (y - a.y_m - t * dy) ** 2;
    if (d < distance2) {
      distance2 = d;
      elevation = a.elevation_m + t * (b.elevation_m - a.elevation_m);
      width = Math.max(a.left_width_m, a.right_width_m);
    }
  }
  return { distance: Math.sqrt(distance2), elevation, width };
}
export function terrainHeight(
  rows: ElevatedGeometry[],
  x: number,
  y: number,
  n = nearestRoad(rows, x, y, 3),
) {
  let weighted = 0,
    total = 0;
  for (let k = 0; k < rows.length - 1; k += 6) {
    const r = rows[k],
      d2 = (r.x_m - x) ** 2 + (r.y_m - y) ** 2,
      w = 1 / (d2 + 625) ** 2;
    weighted += w * r.elevation_m;
    total += w;
  }
  const blend = Math.max(0, Math.min(1, (n.distance - 15) / 45));
  return n.elevation * (1 - blend) + (weighted / total) * blend;
}
export function terrainGeometry(rows: ElevatedGeometry[]) {
  const xs = rows.map((r) => r.x_m),
    ys = rows.map((r) => r.y_m),
    minX = Math.min(...xs) - 220,
    maxX = Math.max(...xs) + 220,
    minY = Math.min(...ys) - 220,
    maxY = Math.max(...ys) + 220;
  const nx = Math.ceil((maxX - minX) / 16),
    ny = Math.ceil((maxY - minY) / 16),
    vertices: number[] = [],
    uv: number[] = [],
    colors: number[] = [],
    indices: number[] = [],
    near: { distance: number; width: number }[] = [];
  for (let j = 0; j <= ny; j++)
    for (let i = 0; i <= nx; i++) {
      const x = minX + ((maxX - minX) * i) / nx,
        y = minY + ((maxY - minY) * j) / ny,
        n = nearestRoad(rows, x, y, 3);
      near.push(n);
      // Track-profile-derived field; no unsourced terrain relief is added. Bias below road.
      vertices.push(x, terrainHeight(rows, x, y, n) - 0.45, -y);
      uv.push(x / 5, y / 5);
      const shade = 0.82 + 0.1 * Math.sin(x * 0.018) * Math.cos(y * 0.014);
      colors.push(shade, shade, shade);
    }
  for (let j = 0; j < ny; j++)
    for (let i = 0; i < nx; i++) {
      const a = j * (nx + 1) + i,
        b = a + 1,
        c = a + nx + 1,
        d = c + 1;
      // Remove terrain cells under/adjacent to the road; the elevated verge stitches this corridor.
      if ([a, b, c, d].some((v) => near[v].distance < near[v].width + 2))
        continue;
      indices.push(a, b, c, b, d, c);
    }
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
  g.setAttribute("uv", new THREE.Float32BufferAttribute(uv, 2));
  g.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
  g.setIndex(indices);
  g.computeVertexNormals();
  return g;
}
export function barrierGeometry(
  rows: ElevatedGeometry[],
  side: number,
  bottom = 0.38,
  top = 0.78,
) {
  const vertices: number[] = [],
    indices: number[] = [];
  for (const row of rows) {
    const w = (side > 0 ? row.left_width_m : row.right_width_m) + 7,
      x = row.x_m - Math.sin(row.heading_rad) * side * w,
      y = row.y_m + Math.cos(row.heading_rad) * side * w;
    vertices.push(
      x,
      row.elevation_m + bottom,
      -y,
      x,
      row.elevation_m + top,
      -y,
    );
  }
  for (let i = 0; i < rows.length - 1; i++) {
    const a = i * 6,
      b = (i + 1) * 6,
      d = Math.hypot(
        vertices[a] - vertices[b],
        vertices[a + 1] - vertices[b + 1],
        vertices[a + 2] - vertices[b + 2],
      );
    if (!Number.isFinite(d) || d > 15 || d < 0.001) continue;
    // Never span a nearby hairpin branch with a barrier.
    const mid = nearestRoad(
      rows,
      (vertices[a] + vertices[b]) / 2,
      -(vertices[a + 2] + vertices[b + 2]) / 2,
      2,
    );
    if (mid.distance < mid.width + 1) continue;
    const v = i * 2;
    indices.push(v, v + 2, v + 1, v + 1, v + 2, v + 3);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
  g.setIndex(indices);
  g.computeVertexNormals();
  return g;
}
export function forestPlacements(rows: ElevatedGeometry[]) {
  let seed = 5541;
  const random = () => {
    seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
    return seed / 4294967296;
  };
  const result: {
    position: THREE.Vector3;
    scale: THREE.Vector3;
    rotation: number;
    variant: number;
  }[] = [];
  for (let i = 0; i < rows.length - 1; i += 3)
    for (const side of [-1, 1])
      for (let layer = 0; layer < 2; layer++) {
        const row = rows[i],
          distance = side * (22 + layer * 34 + random() * 28),
          x = row.x_m - Math.sin(row.heading_rad) * distance,
          y = row.y_m + Math.cos(row.heading_rad) * distance;
        const n = nearestRoad(rows, x, y, 2);
        if (n.distance < n.width + 15) continue;
        const h = 9 + random() * 10,
          r = 2.6 + random() * 2.2;
        result.push({
          position: new THREE.Vector3(
            x,
            n.distance < n.width * 5.5
              ? n.elevation - 0.15
              : terrainHeight(rows, x, y, n) - 0.5,
            -y,
          ),
          scale: new THREE.Vector3(r, h, r),
          rotation: random() * Math.PI * 2,
          variant: random() < 0.65 ? 0 : 1,
        });
      }
  return result;
}
