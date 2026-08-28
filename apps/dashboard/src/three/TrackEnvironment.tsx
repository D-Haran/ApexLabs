import { Line, useTexture } from "@react-three/drei";
import { useEffect, useMemo, useRef } from "react";
import * as THREE from "three";
import type { ElevatedGeometry, Session } from "../telemetry/contract";
import {
  terrainGeometry,
  barrierGeometry,
  forestPlacements,
} from "./environmentGeometry";
import { mergeGeometries } from "three/examples/jsm/utils/BufferGeometryUtils.js";
import type { Overlays } from "./Scene";

export function renderTrackPoint(row: ElevatedGeometry, multiplier: number) {
  const width = multiplier >= 0 ? row.left_width_m : row.right_width_m;
  const distance = width * multiplier;
  return {
    x: row.x_m - Math.sin(row.heading_rad) * distance,
    y: row.elevation_m,
    z: -(row.y_m + Math.cos(row.heading_rad) * distance),
  };
}

function strip(
  rows: ElevatedGeometry[],
  outer: number,
  inner: number,
  verticalOffset = 0,
) {
  const vertices: number[] = [];
  const indices: number[] = [];
  const uv: number[] = [];
  for (const row of rows)
    for (const multiplier of [outer, inner]) {
      const width = multiplier > 0 ? row.left_width_m : row.right_width_m;
      const safe =
        multiplier * row.curvature_1_m > 0
          ? Math.sign(multiplier) *
            Math.min(
              Math.abs(multiplier),
              0.8 / (Math.abs(row.curvature_1_m) * width),
            )
          : multiplier;
      const point = renderTrackPoint(row, safe);
      vertices.push(point.x, point.y + verticalOffset, point.z);
      uv.push((safe * width) / 4, row.s_m / 4);
    }
  for (let index = 0; index < rows.length - 1; index++) {
    const vertex = index * 2;
    indices.push(
      vertex,
      vertex + 2,
      vertex + 1,
      vertex + 1,
      vertex + 2,
      vertex + 3,
    );
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    "position",
    new THREE.Float32BufferAttribute(vertices, 3),
  );
  geometry.setAttribute("uv", new THREE.Float32BufferAttribute(uv, 2));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

function kerb(
  rows: ElevatedGeometry[],
  side: 1 | -1,
  sections: { start_m: number; end_m: number; side: number }[],
) {
  const vertices: number[] = [];
  const colors: number[] = [];
  const red = new THREE.Color("#bd3030");
  const yellow = new THREE.Color("#e1b92f");
  for (let index = 0; index < rows.length - 1; index++) {
    if (
      !sections.some(
        (section) =>
          section.side === side &&
          rows[index].s_m >= section.start_m &&
          rows[index + 1].s_m <= section.end_m,
      )
    )
      continue;
    const outer = side * 1.14;
    const inner = side * 0.98;
    const a = renderTrackPoint(rows[index], outer);
    const b = renderTrackPoint(rows[index], inner);
    const c = renderTrackPoint(rows[index + 1], outer);
    const d = renderTrackPoint(rows[index + 1], inner);
    const quad = [a, c, b, b, c, d];
    const color = Math.floor(rows[index].s_m / 6) % 2 ? yellow : red;
    for (const point of quad) {
      vertices.push(point.x, point.y + 0.018, point.z);
      colors.push(color.r, color.g, color.b);
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    "position",
    new THREE.Float32BufferAttribute(vertices, 3),
  );
  geometry.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
  geometry.computeVertexNormals();
  return geometry;
}

export function trackElevationAt(session: Session, progress: number) {
  const rows = session.geometry;
  let s = progress % session.manifest.track.length_m;
  if (s < 0) s += session.manifest.track.length_m;
  let low = 0;
  let high = rows.length - 1;
  while (low + 1 < high) {
    const middle = (low + high) >> 1;
    if (rows[middle].s_m <= s) low = middle;
    else high = middle;
  }
  const span = rows[high].s_m - rows[low].s_m;
  const fraction = span > 0 ? (s - rows[low].s_m) / span : 0;
  return (
    rows[low].elevation_m +
    fraction * (rows[high].elevation_m - rows[low].elevation_m)
  );
}

function nearestElevation(session: Session, x: number, y: number) {
  let best = session.geometry[0];
  let distance = Number.POSITIVE_INFINITY;
  for (let index = 0; index < session.geometry.length; index += 4) {
    const row = session.geometry[index];
    const candidate = (row.x_m - x) ** 2 + (row.y_m - y) ** 2;
    if (candidate < distance) {
      distance = candidate;
      best = row;
    }
  }
  return best.elevation_m;
}

function contextStrip(
  session: Session,
  points: [number, number][],
  halfWidth: number,
) {
  const vertices: number[] = [];
  const indices: number[] = [];
  points.forEach(([x, y], index) => {
    const previous = points[Math.max(0, index - 1)];
    const next = points[Math.min(points.length - 1, index + 1)];
    const dx = next[0] - previous[0];
    const dy = next[1] - previous[1];
    const length = Math.hypot(dx, dy) || 1;
    const nx = -dy / length;
    const ny = dx / length;
    const elevation = nearestElevation(session, x, y) - 0.015;
    vertices.push(x + nx * halfWidth, elevation, -(y + ny * halfWidth));
    vertices.push(x - nx * halfWidth, elevation, -(y - ny * halfWidth));
  });
  for (let index = 0; index < points.length - 1; index++) {
    const vertex = index * 2;
    indices.push(
      vertex,
      vertex + 2,
      vertex + 1,
      vertex + 1,
      vertex + 2,
      vertex + 3,
    );
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    "position",
    new THREE.Float32BufferAttribute(vertices, 3),
  );
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

function PitAndServiceRoads({ session }: { session: Session }) {
  const meshes = useMemo(() => {
    if (!session.renderContext) return [];
    return session.renderContext.features
      .filter((feature) => {
        const name = feature.tags.name ?? "";
        return (
          /^pit\s*lane$/i.test(name.trim()) ||
          feature.tags.service === "pit_lane"
        );
      })
      .flatMap((feature) => {
        const sections: [number, number][][] = [];
        let section: [number, number][] = [];
        for (const point of feature.points_m) {
          const previous = section.at(-1);
          const discontinuous =
            previous &&
            (Math.hypot(point[0] - previous[0], point[1] - previous[1]) > 35 ||
              Math.abs(
                nearestElevation(session, point[0], point[1]) -
                  nearestElevation(session, previous[0], previous[1]),
              ) > 3);
          if (discontinuous) {
            if (section.length > 1) sections.push(section);
            section = [];
          }
          section.push(point);
        }
        if (section.length > 1) sections.push(section);
        return sections.map((points, index) => ({
          id: `${feature.osm_way_id}-${index}`,
          geometry: contextStrip(session, points, 4.2),
        }));
      });
  }, [session]);
  useEffect(
    () => () => meshes.forEach((mesh) => mesh.geometry.dispose()),
    [meshes],
  );
  return (
    <group>
      {meshes.map((mesh) => (
        <mesh key={mesh.id} geometry={mesh.geometry} receiveShadow>
          <meshStandardMaterial
            color="#343a3b"
            roughness={0.96}
            side={THREE.DoubleSide}
          />
        </mesh>
      ))}
    </group>
  );
}

interface Placement {
  position: THREE.Vector3;
  scale: THREE.Vector3;
  rotation: number;
  variant: number;
}

function Forest({ session }: { session: Session }) {
  const first = useRef<THREE.InstancedMesh>(null);
  const second = useRef<THREE.InstancedMesh>(null);
  const trunks = useRef<THREE.InstancedMesh>(null);
  const placements = useMemo<Placement[]>(() => {
    if (session.manifest.track.kind !== "real_imported") return [];
    const pit =
      session.renderContext?.features
        .filter(
          (f) =>
            /pit/i.test(f.tags.name ?? "") || f.tags.service === "pit_lane",
        )
        .flatMap((f) => f.points_m) ?? [];
    return forestPlacements(session.geometry).filter(
      (p) =>
        !pit.some(
          ([x, y]) => Math.hypot(p.position.x - x, p.position.z + y) < 18,
        ),
    );
  }, [session]);
  const crowns = useMemo(() => {
    const tiers = [0, 0.22, 0.42].map((height, i) => {
      const g = new THREE.ConeGeometry(1 - i * 0.22, 0.6 - i * 0.06, 9);
      g.translate(0, height, 0);
      return g;
    });
    const pine = mergeGeometries(tiers);
    tiers.forEach((g) => g.dispose());
    const broad = new THREE.SphereGeometry(0.75, 12, 8);
    broad.scale(1, 0.58, 1);
    return [pine, broad];
  }, []);
  useEffect(() => () => crowns.forEach((g) => g.dispose()), [crowns]);
  useEffect(() => {
    const matrix = new THREE.Matrix4();
    const quaternion = new THREE.Quaternion();
    const trunkScale = new THREE.Vector3();
    const canopyCounts = [0, 0];
    placements.forEach((placement, index) => {
      quaternion.setFromAxisAngle(
        new THREE.Vector3(0, 1, 0),
        placement.rotation,
      );
      trunkScale.set(0.45, placement.scale.y * 0.48, 0.45);
      matrix.compose(
        placement.position
          .clone()
          .add(new THREE.Vector3(0, trunkScale.y / 2, 0)),
        quaternion,
        trunkScale,
      );
      trunks.current?.setMatrixAt(index, matrix);
      const target = placement.variant ? second.current : first.current;
      const canopyIndex = canopyCounts[placement.variant]++;
      matrix.compose(
        placement.position
          .clone()
          .add(new THREE.Vector3(0, placement.scale.y * 0.54, 0)),
        quaternion,
        placement.scale,
      );
      target?.setMatrixAt(canopyIndex, matrix);
    });
    for (const mesh of [first.current, second.current, trunks.current]) {
      if (mesh) {
        mesh.instanceMatrix.needsUpdate = true;
        mesh.computeBoundingSphere();
        mesh.computeBoundingBox();
      }
    }
  }, [placements]);
  const firstCount = placements.filter(
    (placement) => placement.variant === 0,
  ).length;
  const secondCount = placements.length - firstCount;
  if (!placements.length) return null;
  return (
    <group>
      <instancedMesh
        ref={trunks}
        args={[undefined, undefined, placements.length]}
      >
        <cylinderGeometry args={[0.5, 0.7, 1, 6]} />
        <meshStandardMaterial color="#4a392a" roughness={1} />
      </instancedMesh>
      <instancedMesh
        ref={first}
        args={[crowns[0], undefined, firstCount]}
        castShadow
      >
        <meshStandardMaterial color="#25402a" roughness={0.96} />
      </instancedMesh>
      <instancedMesh
        ref={second}
        args={[crowns[1], undefined, secondCount]}
        castShadow
      >
        <meshStandardMaterial color="#425337" roughness={0.96} />
      </instancedMesh>
    </group>
  );
}

function BarrierPosts({ rows }: { rows: ElevatedGeometry[] }) {
  const ref = useRef<THREE.InstancedMesh>(null);
  const placements = useMemo(() => {
    const result: { position: THREE.Vector3; yaw: number }[] = [];
    for (let index = 0; index < rows.length - 1; index += 5) {
      for (const side of [-1, 1] as const) {
        const width =
          side > 0 ? rows[index].left_width_m : rows[index].right_width_m;
        const point = renderTrackPoint(
          rows[index],
          (side * (width + 7)) / width,
        );
        result.push({
          position: new THREE.Vector3(point.x, point.y + 0.65, point.z),
          yaw: rows[index].heading_rad,
        });
      }
    }
    return result;
  }, [rows]);
  useEffect(() => {
    const matrix = new THREE.Matrix4();
    const quaternion = new THREE.Quaternion();
    const scale = new THREE.Vector3(0.08, 1.3, 0.08);
    placements.forEach((placement, index) => {
      quaternion.setFromAxisAngle(new THREE.Vector3(0, 1, 0), placement.yaw);
      matrix.compose(placement.position, quaternion, scale);
      ref.current?.setMatrixAt(index, matrix);
    });
    if (ref.current) {
      ref.current.instanceMatrix.needsUpdate = true;
      ref.current.computeBoundingSphere();
    }
  }, [placements]);
  return (
    <instancedMesh ref={ref} args={[undefined, undefined, placements.length]}>
      <boxGeometry args={[1, 1, 1]} />
      <meshStandardMaterial color="#7d8788" metalness={0.7} roughness={0.42} />
    </instancedMesh>
  );
}

export function TrackEnvironment({
  session,
  overlays,
}: {
  session: Session;
  overlays: Overlays;
}) {
  const texturePaths = ["asphalt", "grass", "gravel"].flatMap((name) =>
    ["color", "normal", "roughness"].map(
      (kind) => `/assets/surfaces/${name}-${kind}.png`,
    ),
  );
  const textures = useTexture(texturePaths);
  const surfaces = useMemo(() => {
    textures.forEach((t, i) => {
      t.wrapS = t.wrapT = THREE.RepeatWrapping;
      t.anisotropy = 8;
      t.colorSpace = i % 3 === 0 ? THREE.SRGBColorSpace : THREE.NoColorSpace;
      t.needsUpdate = true;
    });
    return {
      asphalt: textures.slice(0, 3),
      grass: textures.slice(3, 6),
      gravel: textures.slice(6, 9),
    };
  }, [textures]);
  const geometry = session.geometry;
  const real = session.manifest.track.kind === "real_imported";
  const meshes = useMemo(
    () => ({
      terrain: real
        ? terrainGeometry(geometry)
        : strip(geometry, 1.28, -1.28, -0.1),
      guardLeft: barrierGeometry(geometry, 1),
      guardRight: barrierGeometry(geometry, -1),
      fenceLeft: barrierGeometry(geometry, 1, 0.78, 2.2),
      fenceRight: barrierGeometry(geometry, -1, 0.78, 2.2),
      verge: strip(geometry, 5.5, -5.5, -0.12),
      leftRunoff: strip(geometry, 2.3, 1.28, -0.16),
      rightRunoff: strip(geometry, -1.28, -2.3, -0.16),
      asphalt: strip(geometry, 1, -1, 0),
      leftKerb: kerb(geometry, 1, session.renderContext?.curbs ?? []),
      rightKerb: kerb(geometry, -1, session.renderContext?.curbs ?? []),
    }),
    [geometry, real, session.renderContext],
  );
  useEffect(
    () => () => Object.values(meshes).forEach((mesh) => mesh.dispose()),
    [meshes],
  );
  const lines = useMemo(
    () => ({
      left: geometry.map((row) => {
        const point = renderTrackPoint(row, 1);
        return [point.x, point.y + 0.035, point.z] as [number, number, number];
      }),
      right: geometry.map((row) => {
        const point = renderTrackPoint(row, -1);
        return [point.x, point.y + 0.035, point.z] as [number, number, number];
      }),
      center: geometry.map(
        (row) =>
          [row.x_m, row.elevation_m + 0.05, -row.y_m] as [
            number,
            number,
            number,
          ],
      ),
    }),
    [geometry],
  );
  const barrierLines = useMemo(
    () =>
      [meshes.guardLeft, meshes.guardRight].map((mesh) => {
        const p = mesh.getAttribute("position"),
          index = mesh.getIndex()!,
          points: number[] = [];
        for (let i = 0; i < index.count; i += 6) {
          for (const vertex of [index.getX(i), index.getX(i + 1)])
            points.push(p.getX(vertex), p.getY(vertex) + 0.2, p.getZ(vertex));
        }
        const g = new THREE.BufferGeometry();
        g.setAttribute("position", new THREE.Float32BufferAttribute(points, 3));
        return g;
      }),
    [meshes],
  );
  useEffect(
    () => () => barrierLines.forEach((g) => g.dispose()),
    [barrierLines],
  );
  const start = geometry[0];
  return (
    <group>
      <mesh geometry={meshes.terrain} receiveShadow>
        <meshStandardMaterial
          color={real ? "#ffffff" : "#454c43"}
          map={surfaces.grass[0]}
          normalMap={surfaces.grass[1]}
          normalScale={new THREE.Vector2(0.3, 0.3)}
          roughnessMap={surfaces.grass[2]}
          roughness={1}
          side={THREE.DoubleSide}
        />
      </mesh>
      {real && (
        <>
          <mesh geometry={meshes.leftRunoff} receiveShadow>
            <meshStandardMaterial
              color="#c5c1b6"
              map={surfaces.gravel[0]}
              normalMap={surfaces.gravel[1]}
              roughness={1}
              side={THREE.DoubleSide}
            />
          </mesh>
          <mesh geometry={meshes.rightRunoff} receiveShadow>
            <meshStandardMaterial
              color="#c5c1b6"
              map={surfaces.gravel[0]}
              normalMap={surfaces.gravel[1]}
              roughness={1}
              side={THREE.DoubleSide}
            />
          </mesh>
          <mesh geometry={meshes.verge} receiveShadow>
            <meshStandardMaterial
              color="#ffffff"
              map={surfaces.grass[0]}
              normalMap={surfaces.grass[1]}
              normalScale={new THREE.Vector2(0.3, 0.3)}
              roughness={1}
              side={THREE.DoubleSide}
            />
          </mesh>
        </>
      )}
      <mesh geometry={meshes.asphalt} receiveShadow>
        <meshStandardMaterial
          color="#a5abb0"
          map={surfaces.asphalt[0]}
          normalMap={surfaces.asphalt[1]}
          normalScale={new THREE.Vector2(0.35, 0.35)}
          roughnessMap={surfaces.asphalt[2]}
          roughness={0.94}
          side={THREE.DoubleSide}
        />
      </mesh>
      {real && (
        <>
          <mesh geometry={meshes.leftKerb} receiveShadow>
            <meshStandardMaterial
              vertexColors
              roughness={0.82}
              side={THREE.DoubleSide}
            />
          </mesh>
          <mesh geometry={meshes.rightKerb} receiveShadow>
            <meshStandardMaterial
              vertexColors
              roughness={0.82}
              side={THREE.DoubleSide}
            />
          </mesh>
          <mesh geometry={meshes.guardLeft} receiveShadow castShadow>
            <meshStandardMaterial
              color="#9aabaf"
              metalness={0.8}
              roughness={0.42}
              side={THREE.DoubleSide}
            />
          </mesh>
          <mesh geometry={meshes.guardRight} receiveShadow castShadow>
            <meshStandardMaterial
              color="#9aabaf"
              metalness={0.8}
              roughness={0.42}
              side={THREE.DoubleSide}
            />
          </mesh>
          {overlays.axes &&
            barrierLines.map((geometry, i) => (
              <lineSegments key={i} geometry={geometry}>
                <lineBasicMaterial color="#ef87ff" />
              </lineSegments>
            ))}
          <mesh geometry={meshes.fenceLeft}>
            <meshStandardMaterial
              color="#91a4a5"
              wireframe
              transparent
              opacity={0.22}
              side={THREE.DoubleSide}
            />
          </mesh>
          <mesh geometry={meshes.fenceRight}>
            <meshStandardMaterial
              color="#91a4a5"
              wireframe
              transparent
              opacity={0.22}
              side={THREE.DoubleSide}
            />
          </mesh>
          <BarrierPosts rows={geometry} />
          <Forest session={session} />
          <PitAndServiceRoads session={session} />
          <mesh
            position={[start.x_m, start.elevation_m + 0.025, -start.y_m]}
            rotation={[0, start.heading_rad, 0]}
            receiveShadow
          >
            <boxGeometry
              args={[0.75, 0.035, start.left_width_m + start.right_width_m]}
            />
            <meshStandardMaterial color="#f4f1df" roughness={0.8} />
          </mesh>
        </>
      )}
      {overlays.boundaries && (
        <>
          <Line points={lines.left} color="#e4e4dc" lineWidth={1.4} />
          <Line points={lines.right} color="#e4e4dc" lineWidth={1.4} />
        </>
      )}
      {overlays.reference && (
        <Line
          points={lines.center}
          color="#9eb2bc"
          lineWidth={1}
          dashed
          dashSize={2}
          gapSize={3}
        />
      )}
      {session.manifest.track.sector_boundaries_fraction.map(
        (fraction, index) => {
          const row =
            geometry[
              Math.min(
                geometry.length - 1,
                Math.round(fraction * (geometry.length - 1)),
              )
            ];
          const left = renderTrackPoint(row, 1);
          const right = renderTrackPoint(row, -1);
          return (
            <Line
              key={index}
              points={[
                [left.x, left.y + 0.05, left.z],
                [right.x, right.y + 0.05, right.z],
              ]}
              lineWidth={index === 2 ? 3 : 1.5}
              color={index === 2 ? "#f4f0dc" : "#baa27d"}
            />
          );
        },
      )}
    </group>
  );
}
