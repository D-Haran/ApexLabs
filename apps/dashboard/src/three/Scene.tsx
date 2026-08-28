import { Canvas, useFrame, useThree } from "@react-three/fiber";
import {
  Line,
  OrbitControls,
  Environment,
  Lightformer,
} from "@react-three/drei";
import { useEffect, useMemo, useRef } from "react";
import * as THREE from "three";
import type { OrbitControls as OrbitControlsImpl } from "three-stdlib";
import { replay } from "../state/replay";
import { wheels, type Session } from "../telemetry/contract";
import { Vehicle, type AssetAdapter } from "./Vehicle";
import { recordFrame, recordRenderStats } from "../state/performance";
import { TrackEnvironment, trackElevationAt } from "./TrackEnvironment";
import {
  roadAt,
  roadQuaternion,
  roadSurfaceHeight,
  springStep,
} from "./vehiclePose";
export type CameraMode = "Chase" | "Orbit" | "Overview" | "Engineering";
export interface Overlays {
  forces: boolean;
  loads: boolean;
  axes: boolean;
  velocity: boolean;
  reference: boolean;
  boundaries: boolean;
  trajectory: boolean;
}
export const defaultOverlays: Overlays = {
  forces: true,
  loads: false,
  axes: false,
  velocity: false,
  reference: true,
  boundaries: true,
  trajectory: true,
};
function History({ session }: { session: Session }) {
  const ref = useRef<THREE.Line>(null);
  const line = useMemo(() => {
    const rows = session.samples.filter((_, i) => i % 8 === 0);
    const g = new THREE.BufferGeometry().setFromPoints(
      rows.map(
        (r) =>
          new THREE.Vector3(
            r.position_x_m,
            trackElevationAt(session, r.track_s_m) + 0.075,
            -r.position_y_m,
          ),
      ),
    );
    return new THREE.Line(
      g,
      new THREE.LineBasicMaterial({
        color: "#d99965",
        transparent: true,
        opacity: 0.72,
      }),
    );
  }, [session]);
  useEffect(
    () => () => {
      line.geometry.dispose();
      (line.material as THREE.Material).dispose();
    },
    [line],
  );
  useFrame(() => {
    const t = replay.getSnapshot().time;
    let n = 0;
    for (
      let i = 0;
      i < session.samples.length && session.samples[i].time_s <= t;
      i += 8
    )
      n++;
    line.geometry.setDrawRange(0, n);
  });
  return <primitive ref={ref} object={line} />;
}
function StaticTrajectory({
  session,
  color,
}: {
  session: Session;
  color: string;
}) {
  const points = useMemo(
    () =>
      session.samples
        .filter((_, index) => index % 12 === 0)
        .map(
          (row) =>
            [
              row.position_x_m,
              trackElevationAt(session, row.track_s_m) + 0.11,
              -row.position_y_m,
            ] as [number, number, number],
        ),
    [session],
  );
  return (
    <Line
      points={points}
      color={color}
      lineWidth={2}
      transparent
      opacity={0.82}
    />
  );
}
function ForceOverlay({
  session,
  overlays,
}: {
  session: Session;
  overlays: Overlays;
}) {
  const arrows = useMemo(
    () =>
      wheels.map(
        () =>
          new THREE.ArrowHelper(
            new THREE.Vector3(1, 0, 0),
            new THREE.Vector3(),
            1,
            0xf0ac6d,
            0.3,
            0.15,
          ),
      ),
    [],
  );
  const velocity = useMemo(
    () =>
      new THREE.ArrowHelper(
        new THREE.Vector3(1, 0, 0),
        new THREE.Vector3(0, 0.35, 0),
        1,
        0x83b6cd,
        0.4,
        0.2,
      ),
    [],
  );
  const loads = useRef<(THREE.Mesh | null)[]>([]);
  const v = session.manifest.vehicle;
  const positions: [
    [number, number, number],
    [number, number, number],
    [number, number, number],
    [number, number, number],
  ] = [
    [v.front_axle_m, 0.18, -v.front_track_m / 2],
    [v.front_axle_m, 0.18, v.front_track_m / 2],
    [-v.rear_axle_m, 0.18, -v.rear_track_m / 2],
    [-v.rear_axle_m, 0.18, v.rear_track_m / 2],
  ];
  const direction = useMemo(() => new THREE.Vector3(), []);
  useEffect(() => {
    for (const arrow of [...arrows, velocity])
      arrow.traverse((object) => {
        object.renderOrder = 5;
        if (object instanceof THREE.Line || object instanceof THREE.Mesh) {
          const materials = Array.isArray(object.material)
            ? object.material
            : [object.material];
          materials.forEach((material) => {
            material.depthTest = false;
          });
        }
      });
    return () => {
      for (const arrow of [...arrows, velocity]) arrow.dispose();
    };
  }, [arrows, velocity]);
  useFrame(() => {
    const s = replay.getSnapshot().sample;
    if (!s) return;
    wheels.forEach((w, i) => {
      const fx = s[`fx_${w}_n`],
        fy = s[`fy_${w}_n`],
        steer = i < 2 ? s.steering_angle_rad : 0;
      direction.set(
        fx * Math.cos(steer) - fy * Math.sin(steer),
        0,
        -fx * Math.sin(steer) - fy * Math.cos(steer),
      );
      const length = direction.length() / 1500;
      arrows[i].visible = overlays.forces && length > 0.005;
      arrows[i].setDirection(direction.normalize());
      arrows[i].setLength(
        Math.max(length, 0.001),
        Math.min(0.35, length / 3),
        Math.min(0.2, length / 4),
      );
      arrows[i].setColor(
        s[`friction_utilization_${w}`] > 0.98 ? 0xef755f : 0xf0ac6d,
      );
      const bar = loads.current[i];
      if (bar) {
        const height = s[`fz_${w}_n`] / 2500;
        bar.scale.y = height;
        bar.position.y = height / 2 + 0.15;
      }
    });
    direction.set(s.vx_m_s, 0, -s.vy_m_s);
    velocity.setLength(direction.length() / 5, 0.4, 0.2);
    velocity.setDirection(direction.normalize());
  });
  return (
    <group>
      {arrows.map((arrow, i) => (
        <primitive key={i} object={arrow} position={positions[i]} />
      ))}
      {overlays.loads &&
        positions.map((p, i) => (
          <mesh
            key={i}
            ref={(r) => {
              loads.current[i] = r;
            }}
            position={p}
          >
            <boxGeometry args={[0.1, 1, 0.1]} />
            <meshBasicMaterial color="#89b8cb" transparent opacity={0.85} />
          </mesh>
        ))}
      {overlays.velocity && <primitive object={velocity} />}
      {overlays.axes && (
        <>
          <Line
            points={[
              [0, 0.12, 0],
              [4, 0.12, 0],
            ]}
            color="#e9a072"
          />
          <Line
            points={[
              [0, 0.12, 0],
              [0, 0.12, -3],
            ]}
            color="#83b6cd"
          />
          <mesh position={[0, 0.14, 0]} rotation={[-Math.PI / 2, 0, 0]}>
            <ringGeometry args={[0.23, 0.3, 32]} />
            <meshBasicMaterial color="white" />
          </mesh>
        </>
      )}
    </group>
  );
}
function World({
  session,
  mode,
  overlays,
  asset,
  comparison,
}: {
  session: Session;
  mode: CameraMode;
  overlays: Overlays;
  asset?: AssetAdapter;
  comparison?: Session | null;
}) {
  const car = useRef<THREE.Group>(null),
    orbit = useRef<OrbitControlsImpl>(null),
    sun = useRef<THREE.DirectionalLight>(null),
    sunTarget = useRef<THREE.Object3D>(null),
    { camera, gl } = useThree();
  const bounds = useMemo(
    () =>
      new THREE.Box3().setFromPoints(
        session.geometry.map(
          (r) => new THREE.Vector3(r.x_m, r.elevation_m, -r.y_m),
        ),
      ),
    [session],
  );
  const center = useMemo(() => bounds.getCenter(new THREE.Vector3()), [bounds]);
  const size = useMemo(() => bounds.getSize(new THREE.Vector3()), [bounds]);
  const target = useMemo(() => new THREE.Vector3(), []),
    desired = useMemo(() => new THREE.Vector3(), []),
    previous = useRef(new THREE.Vector3()),
    reset = useRef(true),
    cameraVelocity = useRef(new THREE.Vector3()),
    targetVelocity = useRef(new THREE.Vector3()),
    filteredTarget = useRef(new THREE.Vector3()),
    lastReplayTime = useRef(0);
  useEffect(() => {
    reset.current = true;
  }, [mode, session]);
  useFrame((_, dt) => {
    replay.advance(dt);
    recordRenderStats({
      draw_calls: gl.info.render.calls,
      triangles: gl.info.render.triangles,
      textures: gl.info.memory.textures,
      geometries: gl.info.memory.geometries,
    });
    recordFrame();
  }, -10);
  useFrame((_, dt) => {
    const s = replay.getSnapshot().sample;
    if (!s || !car.current) return;
    car.current.position.set(
      s.position_x_m,
      roadSurfaceHeight(session, s.position_x_m, -s.position_y_m, s.track_s_m),
      -s.position_y_m,
    );
    const road = roadAt(session, s.track_s_m);
    car.current.quaternion.copy(
      roadQuaternion(road.heading, road.grade, s.yaw_rad),
    );
    car.current.updateMatrixWorld(true);
    if (Math.abs(s.time_s - lastReplayTime.current) > Math.max(0.25, dt * 5))
      reset.current = true;
    lastReplayTime.current = s.time_s;
    target.copy(car.current.position);
    target.add(
      new THREE.Vector3(0, 0.7 + (s.chassis?.heave_m ?? 0), 0).applyQuaternion(
        car.current.quaternion,
      ),
    );
    if (sun.current && sunTarget.current) {
      sun.current.position
        .copy(car.current.position)
        .add(new THREE.Vector3(80, 150, 40));
      sunTarget.current.position.copy(car.current.position);
      sunTarget.current.updateMatrixWorld();
      sun.current.target = sunTarget.current;
    }
    if (mode === "Orbit") {
      if (orbit.current) {
        if (reset.current) {
          camera.position.copy(target).add(new THREE.Vector3(-9, 7, 10));
          orbit.current.target.copy(target);
        } else {
          camera.position.add(desired.copy(target).sub(previous.current));
          orbit.current.target.copy(target);
        }
        orbit.current.update();
      }
    } else {
      if (mode === "Overview") {
        target.copy(center);
        desired
          .copy(center)
          .add(
            new THREE.Vector3(
              30,
              Math.max(size.x, size.z) * 0.85,
              Math.max(size.x, size.z) * 0.54,
            ),
          );
      } else {
        const x = mode === "Chase" ? -6.3 : -8,
          z = mode === "Chase" ? 0 : 10;
        desired
          .set(x, mode === "Chase" ? 1.65 : 8, z)
          .applyQuaternion(car.current.quaternion)
          .add(target);
      }
      if (reset.current || mode !== "Chase") {
        camera.position.copy(desired);
        filteredTarget.current.copy(target);
        cameraVelocity.current.set(0, 0, 0);
        targetVelocity.current.set(0, 0, 0);
      } else {
        springStep(camera.position, cameraVelocity.current, desired, dt, 10);
        springStep(
          filteredTarget.current,
          targetVelocity.current,
          target,
          dt,
          14,
        );
      }
      camera.lookAt(filteredTarget.current);
    }
    previous.current.copy(target);
    reset.current = false;
  }, -5);
  return (
    <>
      <color
        attach="background"
        args={[
          session.manifest.track.kind === "real_imported"
            ? "#94a4ad"
            : "#202b2d",
        ]}
      />
      <fog
        attach="fog"
        args={[
          session.manifest.track.kind === "real_imported"
            ? "#94a4ad"
            : "#202b2d",
          session.manifest.track.kind === "real_imported" ? 900 : 450,
          session.manifest.track.kind === "real_imported" ? 6000 : 1200,
        ]}
      />
      <ambientLight intensity={0.35} />
      <hemisphereLight args={["#dbe7eb", "#5b6253", 0.65]} />
      <directionalLight
        ref={sun}
        position={[80, 150, 40]}
        intensity={1.7}
        castShadow
        shadow-mapSize={[2048, 2048]}
        shadow-camera-left={-22}
        shadow-camera-right={22}
        shadow-camera-top={22}
        shadow-camera-bottom={-22}
        shadow-bias={-0.00004}
        shadow-normalBias={0.015}
      />
      <object3D ref={sunTarget} />
      <Environment resolution={64}>
        <Lightformer
          position={[0, 20, 0]}
          rotation={[Math.PI / 2, 0, 0]}
          scale={[40, 40, 1]}
          intensity={2}
        />
      </Environment>
      {session.manifest.track.kind !== "real_imported" && (
        <group
          position={[center.x, center.y - 0.22, center.z]}
          quaternion={roadQuaternion(0, session.geometry[0].grade)}
        >
          <mesh
            rotation={[-Math.PI / 2, 0, 0]}
            position={[0, 0, 0]}
            receiveShadow
          >
            <planeGeometry args={[2000, 2000]} />
            <meshStandardMaterial color="#29342e" roughness={1} />
          </mesh>
          <gridHelper
            args={[1600, 160, "#4b5752", "#4b5752"]}
            position={[0, 0.02, 0]}
          />
        </group>
      )}
      <TrackEnvironment session={session} overlays={overlays} />
      {overlays.trajectory &&
        (comparison ? (
          <>
            <StaticTrajectory
              session={session}
              color={session.manifest.optimization ? "#f0a66c" : "#6ba9df"}
            />
            <StaticTrajectory
              session={comparison}
              color={comparison.manifest.optimization ? "#f0a66c" : "#6ba9df"}
            />
          </>
        ) : (
          <History session={session} />
        ))}
      <group ref={car}>
        <Vehicle
          vehicle={session.manifest.vehicle}
          asset={asset}
          session={session}
          debug={overlays.axes}
        />
        {overlays.axes && <axesHelper args={[2]} />}
        <ForceOverlay session={session} overlays={overlays} />
      </group>
      <OrbitControls
        ref={orbit}
        enabled={mode === "Orbit"}
        enablePan={false}
        minDistance={3}
        maxDistance={150}
        maxPolarAngle={Math.PI / 2 - 0.02}
      />
    </>
  );
}
export function Scene(props: {
  session: Session;
  comparison?: Session | null;
  mode: CameraMode;
  overlays: Overlays;
  asset?: AssetAdapter;
}) {
  return (
    <Canvas
      shadows
      dpr={[1, 1.75]}
      camera={{ fov: 48, near: 0.5, far: 12000, position: [0, 300, 200] }}
      gl={{
        antialias: true,
        powerPreference: "high-performance",
        toneMapping: THREE.ACESFilmicToneMapping,
      }}
    >
      <World {...props} />
    </Canvas>
  );
}
