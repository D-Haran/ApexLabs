import { memo, Profiler, Suspense, useEffect, useMemo, useRef } from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { Environment, Line, OrbitControls, Html } from "@react-three/drei";
import * as THREE from "three";
import { Car, Contacts } from "../contact/ContactLab";
import { toRender, toRenderQuaternion } from "../contact/contract";
import {
  TrackEnvironment,
  type EnvironmentSession,
} from "../three/TrackEnvironment";
import type { CarManifest } from "../three/carManifest";
import {
  ghostAt,
  type DriveMetadata,
  type DriveSample,
  type Ghost,
} from "./contract";
import type { DriveClock } from "./clock";
import { spring, cameraFrequency, type ChaseMode } from "./camera";
import { measure, recordCost, cpuReport } from "./performance";
const StaticEnvironment = memo(TrackEnvironment);
const StaticKerbs = memo(PhysicalKerbs);
const noop = () => {};
const ghostCost = (ms: number) => recordCost("ghost transforms", ms);
const vehicleCost = (ms: number) => recordCost("vehicle transforms", ms);
const forceCost = (ms: number) => recordCost("tire-force arrow updates", ms);
function Camera({
  sample,
  overview,
  chaseMode,
}: {
  sample: DriveSample;
  overview: boolean;
  chaseMode: ChaseMode;
}) {
  const controls = useRef<React.ComponentRef<typeof OrbitControls>>(null);
  const pivot = useRef(new THREE.Vector3(...toRender(sample.position)));
  const velocity = useRef(new THREE.Vector3());
  const orbitOffset = useRef(new THREE.Vector3(-8, 4, 4));
  const previousMode = useRef(overview);
  const initialized = useRef(false);
  useFrame(({ camera }, dt) =>
    measure("camera system", () => {
      const p = new THREE.Vector3(...toRender(sample.position)),
        q = toRenderQuaternion(sample.quaternion);
      const switched = previousMode.current !== overview;
      if (overview && controls.current) {
        if (switched || !initialized.current)
          camera.position.copy(p).add(orbitOffset.current);
        else camera.position.add(p.clone().sub(pivot.current));
        controls.current.target.copy(p);
        controls.current.update();
        orbitOffset.current.copy(camera.position).sub(p);
      } else {
        const desired = new THREE.Vector3(-6.1, 1.65, 1.15)
          .applyQuaternion(q)
          .add(p);
        // Transport with the current pose first. The spring filters relative camera
        // motion, so constant translation cannot create a second vehicle-pose lag.
        if (!initialized.current || p.distanceTo(pivot.current) > 30) {
          camera.position.copy(desired);
          velocity.current.set(0, 0, 0);
        } else {
          camera.position.add(p.clone().sub(pivot.current));
          for (const axis of ["x", "y", "z"] as const)
            [camera.position[axis], velocity.current[axis]] = spring(
              camera.position[axis],
              velocity.current[axis],
              desired[axis],
              cameraFrequency[chaseMode],
              Math.min(dt, 0.1),
            );
        }
        camera.lookAt(
          p.clone().add(new THREE.Vector3(1.8, 0.3, 0).applyQuaternion(q)),
        );
      }
      pivot.current.copy(p);
      previousMode.current = overview;
      initialized.current = true;
      Object.assign(window, {
        __apexDriveCamera: {
          mode: overview ? "orbit" : "chase",
          pivot: p.toArray(),
          offset: camera.position.clone().sub(p).toArray(),
          time: sample.time,
        },
      });
    }),
  );
  return (
    <OrbitControls
      ref={controls}
      makeDefault
      enabled={overview}
      enableDamping={false}
      enablePan={false}
      minDistance={3}
      maxDistance={100}
    />
  );
}
function FrameSnapshot({
  clock,
  sample,
  ghost,
  source,
  mode,
  length,
}: {
  clock: DriveClock;
  sample: DriveSample;
  ghost: DriveSample;
  source?: Ghost;
  mode: "time" | "s";
  length: number;
}) {
  useFrame(() => {
    const current = measure("replay interpolation", () => clock.read());
    if (current) Object.assign(sample, current);
    if (source) {
      const g = measure("ghost", () =>
        ghostAt(
          source.samples,
          mode === "time"
            ? sample.lapElapsed
            : ((sample.s % length) + length) % length,
          mode,
        ),
      );
      if (g) Object.assign(ghost, g);
    }
    Object.assign(window, {
      __apexDriveClock: {
        simulationTime: clock.simulationTime,
        playbackState: clock.playbackState,
        direction: clock.playbackDirection,
        carTime: sample.time,
        position: sample.position,
      },
    });
  }, -10);
  return null;
}
function Performance() {
  const frames = useRef<number[]>([]),
    last = useRef(0);
  const instrumented = useRef(new WeakSet<THREE.Object3D>());
  const draws = useRef<Record<string, number>>({});
  useFrame(({ gl, scene, camera }, dt) => {
    frames.current.push(dt * 1000);
    if (frames.current.length > 600) frames.current.shift();
    // CPU submission cost per scene category; GPU execution remains asynchronous.
    scene.traverseVisible((object) => {
      if (
        !(
          object instanceof THREE.Mesh ||
          object instanceof THREE.Line ||
          object instanceof THREE.Points
        ) ||
        instrumented.current.has(object)
      )
        return;
      let parent: THREE.Object3D | null = object,
        category = "vehicle/other submission";
      while (parent) {
        if (parent.name.startsWith("profile:")) {
          category = parent.name.slice(8) + " submission";
          break;
        }
        parent = parent.parent;
      }
      const before = object.onBeforeRender,
        after = object.onAfterRender;
      let started = 0;
      object.onBeforeRender = function (...args) {
        started = performance.now();
        before.apply(this, args);
      };
      object.onAfterRender = function (...args) {
        after.apply(this, args);
        draws.current[category] =
          (draws.current[category] ?? 0) + performance.now() - started;
      };
      instrumented.current.add(object);
    });
    draws.current = {
      "environment submission": 0,
      "ghost submission": 0,
      "force arrows submission": 0,
      "vehicle/other submission": 0,
    };
    measure("Three.js rendering", () => gl.render(scene, camera));
    for (const [name, cost] of Object.entries(draws.current))
      recordCost(name, cost);
    if (performance.now() - last.current > 1000) {
      last.current = performance.now();
      const sorted = [...frames.current].sort((a, b) => a - b);
      Object.assign(window, {
        __apexDrivePerformance: {
          mean: sorted.reduce((a, b) => a + b, 0) / sorted.length,
          p95: sorted[Math.floor(sorted.length * 0.95)],
          calls: gl.info.render.calls,
          triangles: gl.info.render.triangles,
          textures: gl.info.memory.textures,
        },
        __apexDriveCPU: cpuReport(),
      });
    }
  }, 1);
  return null;
}
export default function DriveScene({
  sample,
  meta,
  manifest,
  forces,
  reference,
  overview,
  history,
  extra,
  policyPath,
  clock,
  chaseMode,
  ghostSource,
  ghostMode,
  onInspect,
}: {
  sample: DriveSample;
  meta: DriveMetadata;
  manifest: CarManifest;
  forces: boolean;
  reference: boolean;
  overview: boolean;

  history: DriveSample[];
  extra: Record<string, boolean>;
  policyPath?: DriveSample[];
  clock: DriveClock;
  chaseMode: ChaseMode;
  ghostSource?: Ghost;
  ghostMode: "time" | "s";
  onInspect: (wheel: number) => void;
}) {
  const scene = useMemo<EnvironmentSession>(
    () => ({
      geometry: meta.geometry,
      renderContext: meta.renderContext
        ? { ...meta.renderContext, curbs: [] }
        : undefined,
      manifest: {
        track: {
          length_m: meta.length,
          kind:
            meta.track === "Technical Test Circuit"
              ? "synthetic_validation"
              : "real_imported",
          sector_boundaries_fraction: meta.sectors,
        },
      },
    }),
    [meta],
  );
  const geometry = useMemo(() => ({ parameters: meta.parameters }), [meta]);
  const assetPath =
    meta.family === "p1"
      ? "/assets/mclaren-p1.glb"
      : "/assets/mclaren-f1-2022.glb";
  const trail = useMemo(
    () =>
      history
        .filter((_, i) => i % 5 === 0)
        .map((s) =>
          toRender([s.position[0], s.position[1], s.position[2] - 0.45]),
        ),
    [history],
  );
  const renderSample = useMemo(() => ({ ...sample }), [clock, meta]);
  const renderGhost = useMemo(() => ({ ...sample }), [clock, meta]);
  const environmentOverlays = useMemo(
    () => ({
      forces: false,
      loads: false,
      axes: false,
      velocity: false,
      reference,
      boundaries: extra.boundaries,
      trajectory: false,
    }),
    [reference, extra.boundaries],
  );
  const policyPoints = useMemo(
    () =>
      policyPath
        ?.filter((_, i) => i % 5 === 0)
        .map((s) =>
          toRender([
            s.position[0],
            s.position[1],
            s.wheels.reduce((v, w) => v + w.patch[2], 0) / 4 + 0.04,
          ]),
        ),
    [policyPath],
  );
  const cameraConfig = useMemo(
    () => ({
      position: [
        sample.position[0] - 8,
        sample.position[2] + 4,
        -sample.position[1] + 4,
      ] as [number, number, number],
      fov: 48,
      far: 12000,
    }),
    [meta],
  );
  return (
    <Canvas
      shadows
      camera={cameraConfig}
      gl={{ toneMapping: THREE.ACESFilmicToneMapping, antialias: true }}
      dpr={[1, 1.5]}
    >
      <FrameSnapshot
        clock={clock}
        sample={renderSample}
        ghost={renderGhost}
        source={ghostSource}
        mode={ghostMode}
        length={meta.length}
      />
      <color attach="background" args={["#a9bdc8"]} />
      <fog attach="fog" args={["#a9bdc8", 350, 5000]} />
      <hemisphereLight args={["#b9d9f3", "#576144", 0.45]} />
      <Sun sample={renderSample} />
      <Suspense fallback={null}>
        <Environment
          files="/assets/environment/sky.hdr"
          background
          environmentIntensity={0.65}
          backgroundIntensity={0.8}
        />
        <group name="profile:environment">
          <Profiler
            id="environment"
            onRender={(_, __, duration) =>
              recordCost("environment React", duration)
            }
          >
            <StaticEnvironment
              detailed
              session={scene}
              overlays={environmentOverlays}
            />
            <StaticKerbs meta={meta} />
          </Profiler>
        </group>
        <Car
          sample={renderSample}
          manifest={manifest}
          recording={geometry}
          assetPath={assetPath}
          onReady={noop}
          onFrameCost={vehicleCost}
        />
        {ghostSource && (
          <group name="profile:ghost">
            <Car
              sample={renderGhost}
              manifest={manifest}
              recording={geometry}
              assetPath={assetPath}
              ghost
              onFrameCost={ghostCost}
              onReady={noop}
            />
          </group>
        )}
      </Suspense>
      {forces && (
        <group name="profile:force arrows">
          <Contacts
            sample={renderSample}
            onInspect={onInspect}
            onFrameCost={forceCost}
          />
        </group>
      )}
      <DebugVectors sample={renderSample} extra={extra} />
      {extra.policy && policyPoints && (
        <Line points={policyPoints} color="#52d99b" lineWidth={1.5} />
      )}{" "}
      {extra.player && trail.length > 1 && (
        <Line points={trail} color="#67b9f5" lineWidth={1.5} />
      )}
      <Camera sample={renderSample} overview={overview} chaseMode={chaseMode} />
      <Performance />
    </Canvas>
  );
}

function Sun({ sample }: { sample: DriveSample }) {
  const light = useRef<THREE.DirectionalLight>(null),
    target = useMemo(() => new THREE.Object3D(), []);
  useFrame(() => {
    target.position.fromArray(toRender(sample.position));
    target.updateMatrixWorld();
    if (light.current) {
      light.current.position
        .copy(target.position)
        .add(new THREE.Vector3(-25, 55, 30));
      light.current.target = target;
    }
  });
  return (
    <>
      <primitive object={target} />
      <directionalLight
        ref={light}
        intensity={2.8}
        castShadow
        shadow-mapSize={[2048, 2048]}
        shadow-camera-left={-35}
        shadow-camera-right={35}
        shadow-camera-top={35}
        shadow-camera-bottom={-35}
        shadow-camera-far={150}
        shadow-bias={-0.0002}
        shadow-normalBias={0.015}
      />
    </>
  );
}

function PhysicalKerbs({ meta }: { meta: DriveMetadata }) {
  const geometries = useMemo(
    () =>
      (meta.physicalKerbs ?? []).map((strip) => {
        const positions = strip.positions.flatMap((p) => toRender(p)),
          colors = strip.colors.flatMap((v) =>
            v ? [0.74, 0.12, 0.08] : [0.85, 0.65, 0.15],
          ),
          indices: number[] = [];
        for (let i = 0; i < strip.positions.length - strip.columns; i++)
          if (i % strip.columns < strip.columns - 1)
            indices.push(
              i,
              i + strip.columns,
              i + 1,
              i + 1,
              i + strip.columns,
              i + strip.columns + 1,
            );
        const g = new THREE.BufferGeometry();
        g.setAttribute(
          "position",
          new THREE.Float32BufferAttribute(positions, 3),
        );
        g.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
        g.setIndex(indices);
        g.computeVertexNormals();
        return g;
      }),
    [meta],
  );
  useEffect(() => () => geometries.forEach((g) => g.dispose()), [geometries]);
  return (
    <>
      {geometries.map((g, i) => (
        <mesh geometry={g} key={i} receiveShadow>
          <meshStandardMaterial
            vertexColors
            roughness={0.85}
            side={THREE.DoubleSide}
          />
        </mesh>
      ))}
    </>
  );
}

function DebugVectors({
  sample,
  extra,
}: {
  sample: DriveSample;
  extra: Record<string, boolean>;
}) {
  const group = useMemo(() => new THREE.Group(), []);
  const cg = useRef<THREE.Mesh>(null);
  const arrows = useMemo(
    () =>
      Array.from(
        { length: 6 },
        () =>
          new THREE.ArrowHelper(
            new THREE.Vector3(0, 1, 0),
            new THREE.Vector3(),
            1,
          ),
      ),
    [],
  );
  useEffect(() => {
    arrows.forEach((a) => group.add(a));
    return () => {
      arrows.forEach((a) => a.dispose());
      group.clear();
    };
  }, [arrows, group]);
  useFrame(() =>
    measure("force arrows", () => {
      cg.current?.position.fromArray(toRender(sample.position));
      const vectors: [number[], number[], string, number, boolean][] = [
        [sample.position, sample.velocity, "#59b7ff", 15, extra.velocity],
        [sample.position, sample.aeroForce, "#c194ff", 10000, extra.aero],
        ...sample.wheels.map(
          (w) =>
            [
              w.patch,
              (w as typeof w & { normal?: number[] }).normal ?? [0, 0, 1],
              "#eee387",
              1,
              extra.road,
            ] as [number[], number[], string, number, boolean],
        ),
      ];
      vectors.forEach(([origin, vector, color, scale, enabled], i) => {
        const a = arrows[i],
          v = new THREE.Vector3(
            ...toRender(vector as [number, number, number]),
          );
        a.visible = enabled && v.length() > 1e-9;
        if (a.visible) {
          a.position.fromArray(toRender(origin as [number, number, number]));
          a.setLength(Math.min(5, v.length() / scale), 0.15, 0.08);
          a.setDirection(v.normalize());
          a.setColor(color);
        }
      });
    }),
  );
  return (
    <>
      <primitive object={group} />
      {extra.cg && (
        <mesh ref={cg} position={toRender(sample.position)}>
          <sphereGeometry args={[0.07, 12, 8]} />
          <meshBasicMaterial color="#ffd14b" depthTest={false} />
        </mesh>
      )}
      {sample.wheels.map((_, i) => (
        <WheelOverlay
          key={i}
          sample={sample}
          index={i}
          contacts={extra.contacts}
          loads={extra.loads}
        />
      ))}
    </>
  );
}

function WheelOverlay({
  sample,
  index,
  contacts,
  loads,
}: {
  sample: DriveSample;
  index: number;
  contacts: boolean;
  loads: boolean;
}) {
  const label = useRef<THREE.Group>(null),
    text = useRef<HTMLSpanElement>(null);
  const line = useMemo(
    () =>
      new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(),
          new THREE.Vector3(),
        ]),
        new THREE.LineBasicMaterial({ color: "#f6c64e" }),
      ),
    [],
  );
  useEffect(
    () => () => {
      line.geometry.dispose();
      line.material.dispose();
    },
    [line],
  );
  useFrame(() => {
    const w = sample.wheels[index];
    label.current?.position.fromArray(toRender(w.center));
    if (text.current)
      text.current.textContent = `${(w.fz / 1000).toFixed(2)} kN`;
    const points = line.geometry.getAttribute("position");
    points.setXYZ(0, ...toRender(w.center));
    points.setXYZ(1, ...toRender(w.patch));
    points.needsUpdate = true;
    line.geometry.computeBoundingSphere();
    line.visible = contacts;
  });
  return (
    <>
      <primitive object={line} />
      {loads && (
        <group ref={label}>
          <Html
            style={{
              whiteSpace: "nowrap",
              fontSize: 10,
              background: "#071827bb",
              color: "#65efa8",
              padding: 3,
            }}
          >
            <span ref={text} />
          </Html>
        </group>
      )}
    </>
  );
}
