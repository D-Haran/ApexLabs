import { Component, Suspense, useMemo, useEffect, type ReactNode } from "react";
import { RoundedBox, useGLTF } from "@react-three/drei";
import { useFrame } from "@react-three/fiber";
import { useRef } from "react";
import {
  Shape,
  ExtrudeGeometry,
  Group,
  Mesh,
  MeshPhysicalMaterial,
  SphereGeometry,
  MeshBasicMaterial,
  Vector3,
  Quaternion,
  AxesHelper,
} from "three";
import type { Session } from "../telemetry/contract";
import { type CarManifest, wheelNames } from "./carManifest";
import { wheelSpin, roadSurfaceHeight } from "./vehiclePose";
import { replay } from "../state/replay";
import type { Manifest } from "../telemetry/contract";
export interface AssetAdapter {
  url: string;
  manifest?: CarManifest;
  onLoaded?: () => void;
  onError?: (message: string) => void;
  scale: number;
  rotation: [number, number, number];
  offset: [number, number, number];
}
function Imported({
  asset,
  vehicle,
  session,
  debug,
}: {
  asset: AssetAdapter;
  vehicle: Manifest["vehicle"];
  session: Session;
  debug: boolean;
}) {
  const gltf = useGLTF(asset.url);
  const rig = useMemo(() => {
    const scene = gltf.scene.clone(true),
      manifest = asset.manifest;
    scene.traverse((object) => {
      if (object instanceof Mesh) {
        object.castShadow = true;
        object.receiveShadow = true;
        if (!Array.isArray(object.material)) {
          object.material = object.material.clone();
          if (
            object.material instanceof MeshPhysicalMaterial &&
            /EXT_Carpaint\.007/.test(object.material.name)
          ) {
            object.material.color.set("#f27619");
            object.material.roughness = 0.22;
            object.material.metalness = 0.55;
            object.material.clearcoat = 1;
            object.material.clearcoatRoughness = 0.08;
          }
        }
      }
    });
    const body = manifest
      ? scene.getObjectByName(manifest.bodyNode)
      : undefined;
    const pivot = new Group();
    pivot.name = "DYNAMIC_CHASSIS";
    const cg = vehicle.cg_height_m ?? 0,
      mid = (vehicle.front_axle_m - vehicle.rear_axle_m) / 2;
    if (body) {
      scene.add(pivot);
      pivot.add(body);
      pivot.position.y = cg;
      body.position.set(mid, -cg, 0);
    }
    const wheels = manifest
      ? wheelNames.map((key) => {
          const w = manifest.wheels[key],
            steer = scene.getObjectByName(w.steerPivot),
            spin = scene.getObjectByName(w.spinPivot);
          if (!steer || !spin)
            throw new Error("Missing semantic wheel pivot " + key);
          steer.position.x += mid;
          return { key, w, steer, spin };
        })
      : [];
    const helpers = new Group();
    helpers.name = "CONTACT_DEBUG";
    scene.add(helpers);
    const bodyAxes = new AxesHelper(1.2);
    pivot.add(bodyAxes);
    const origin = new Mesh(
      new SphereGeometry(0.045, 8, 6),
      new MeshBasicMaterial({ color: "#ff7cce", depthTest: false }),
    );
    helpers.add(origin);
    for (const wheel of wheels) {
      for (const y of [0, wheel.w.radius]) {
        const dot = new Mesh(
          new SphereGeometry(0.055, 8, 6),
          new MeshBasicMaterial({
            color: y ? "#5bcaff" : "#74ffa6",
            depthTest: false,
          }),
        );
        dot.position.set(wheel.steer.position.x, y, wheel.steer.position.z);
        dot.renderOrder = 10;
        dot.name = wheel.key + (y ? "_CENTER" : "_CONTACT");
        helpers.add(dot);
      }
    }
    const cgDot = new Mesh(
      new SphereGeometry(0.08, 10, 8),
      new MeshBasicMaterial({ color: "#ffd477", depthTest: false }),
    );
    cgDot.position.y = cg;
    cgDot.visible = vehicle.cg_height_m !== undefined;
    helpers.add(cgDot);
    return { scene, pivot, wheels, helpers, bodyAxes };
  }, [gltf.scene, asset.manifest, vehicle]);
  useEffect(
    () => () => {
      // Source geometries/textures belong to useGLTF's cache. Dispose only owned clones/helpers.
      rig.scene.traverse((object) => {
        if (object instanceof Mesh && !Array.isArray(object.material))
          object.material.dispose();
      });
      rig.helpers.traverse((object) => {
        if (object instanceof Mesh) object.geometry.dispose();
      });
      rig.bodyAxes.dispose();
    },
    [rig],
  );
  useEffect(() => {
    asset.onLoaded?.();
  }, [asset]);
  useFrame(() => {
    const sample = replay.getSnapshot().sample;
    if (!sample) return;
    const b = sample.chassis;
    rig.pivot.position.y = (vehicle.cg_height_m ?? 0) + (b?.heave_m ?? 0);
    rig.pivot.quaternion
      .copy(
        new Quaternion().setFromAxisAngle(
          new Vector3(0, 0, 1),
          b?.pitch_rad ?? 0,
        ),
      )
      .multiply(
        new Quaternion().setFromAxisAngle(
          new Vector3(1, 0, 0),
          b?.roll_rad ?? 0,
        ),
      );
    rig.helpers.visible = debug;
    rig.bodyAxes.visible = debug;
    rig.scene.updateWorldMatrix(true, false);
    const normal = new Vector3(0, 1, 0).transformDirection(
      rig.scene.matrixWorld,
    );
    for (const { key, w, steer, spin } of rig.wheels) {
      steer.rotation.y = key[0] === "F" ? sample.steering_angle_rad : 0;
      spin.rotation.z = wheelSpin(
        sample.visual_wheel_distance_m?.[wheelNames.indexOf(key)] ??
          sample.visual_distance_m ??
          0,
        w.radius,
      );
      const contact = rig.scene.localToWorld(
        new Vector3(steer.position.x, 0, steer.position.z),
      );
      const road = roadSurfaceHeight(
        session,
        contact.x,
        contact.z,
        sample.track_s_m,
      );
      steer.position.y = w.radius + (road - contact.y) / normal.y;
      rig.helpers.getObjectByName(key + "_CENTER")!.position.y =
        steer.position.y;
      rig.helpers.getObjectByName(key + "_CONTACT")!.position.y =
        steer.position.y - w.radius;
    }
  });
  return (
    <group
      scale={asset.scale}
      rotation={asset.rotation}
      position={asset.offset}
    >
      <primitive object={rig.scene} />
    </group>
  );
}
class AssetBoundary extends Component<
  {
    children: ReactNode;
    fallback: ReactNode;
    onError?: (message: string) => void;
  },
  { failed: boolean }
> {
  state = { failed: false };
  static getDerivedStateFromError() {
    return { failed: true };
  }
  componentDidCatch(error: Error) {
    this.props.onError?.(error.message);
  }
  render() {
    return this.state.failed ? this.props.fallback : this.props.children;
  }
}
function Wheel({
  position,
  front,
}: {
  position: [number, number, number];
  front: boolean;
}) {
  const steering = useRef<Group>(null);
  const rolling = useRef<Group>(null);
  useFrame(() => {
    const sample = replay.getSnapshot().sample;
    if (rolling.current)
      rolling.current.rotation.z = wheelSpin(
        sample?.visual_distance_m ?? 0,
        0.33,
      );
    if (steering.current)
      steering.current.rotation.y = front
        ? (sample?.steering_angle_rad ?? 0)
        : 0;
  });
  return (
    <group ref={steering} position={position}>
      <group ref={rolling}>
        <mesh rotation={[Math.PI / 2, 0, 0]} castShadow>
          <cylinderGeometry args={[0.33, 0.33, 0.26, 24]} />
          <meshStandardMaterial color="#111314" roughness={0.88} />
        </mesh>
        <mesh rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[0.225, 0.225, 0.275, 12]} />
          <meshStandardMaterial
            color="#363b3e"
            metalness={0.92}
            roughness={0.2}
          />
        </mesh>
        <mesh position={[0.21, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
          <boxGeometry args={[0.1, 0.03, 0.285]} />
          <meshStandardMaterial
            color="#c4c6c5"
            metalness={0.86}
            roughness={0.25}
          />
        </mesh>
      </group>
    </group>
  );
}
function Shell({
  points,
  width,
  color,
  metalness = 0.7,
}: {
  points: [number, number][];
  width: number;
  color: string;
  metalness?: number;
}) {
  const geometry = useMemo(() => {
    const shape = new Shape();
    points.forEach((p, i) => (i ? shape.lineTo(...p) : shape.moveTo(...p)));
    shape.closePath();
    const g = new ExtrudeGeometry(shape, {
      depth: width,
      bevelEnabled: true,
      bevelSegments: 2,
      steps: 1,
      bevelSize: 0.04,
      bevelThickness: 0.04,
    });
    g.translate(0, 0, -width / 2);
    return g;
  }, [points, width]);
  useEffect(() => () => geometry.dispose(), [geometry]);
  return (
    <mesh geometry={geometry} castShadow>
      <meshStandardMaterial
        color={color}
        metalness={metalness}
        roughness={0.28}
      />
    </mesh>
  );
}
const bodyShape: [number, number][] = [
  [-2.25, 0.37],
  [1.94, 0.37],
  [2.14, 0.57],
  [1.8, 0.73],
  [0.75, 0.78],
  [-1.4, 0.81],
  [-2.25, 0.69],
];
const glassShape: [number, number][] = [
  [-1.5, 0.81],
  [-0.95, 1.25],
  [0.05, 1.25],
  [0.72, 0.79],
];
export function Placeholder({ vehicle }: { vehicle: Manifest["vehicle"] }) {
  const isP1 = /McLaren P1/i.test(vehicle.name);
  const paint = isP1 ? "#f06a21" : "#cad2d0";
  return (
    <group>
      <Shell points={bodyShape} width={1.73} color={paint} />
      <Shell
        points={glassShape}
        width={1.34}
        color="#243237"
        metalness={0.55}
      />
      <RoundedBox
        args={[1.05, 0.065, 1.3]}
        radius={0.025}
        position={[-0.46, 1.28, 0]}
      >
        <meshStandardMaterial color={paint} metalness={0.76} roughness={0.25} />
      </RoundedBox>
      <mesh position={[1.17, 0.78, 0]}>
        <boxGeometry args={[1.35, 0.035, 0.15]} />
        <meshStandardMaterial color={isP1 ? "#1d2224" : "#ed9d62"} />
      </mesh>
      {[-0.61, 0.61].map((z) => (
        <mesh key={z} position={[2.055, 0.71, z]}>
          <boxGeometry args={[0.025, 0.075, 0.38]} />
          <meshStandardMaterial
            color="#d3eef1"
            emissive="#aac6cb"
            emissiveIntensity={0.5}
          />
        </mesh>
      ))}
      <mesh position={[-2.31, 0.67, 0]}>
        <boxGeometry args={[0.025, 0.06, 1.45]} />
        <meshStandardMaterial
          color="#9d3f2f"
          emissive="#9d3f2f"
          emissiveIntensity={0.3}
        />
      </mesh>
      <Wheel
        front
        position={[vehicle.front_axle_m, 0.33, -vehicle.front_track_m / 2]}
      />
      <Wheel
        front
        position={[vehicle.front_axle_m, 0.33, vehicle.front_track_m / 2]}
      />
      <Wheel
        front={false}
        position={[-vehicle.rear_axle_m, 0.33, -vehicle.rear_track_m / 2]}
      />
      <Wheel
        front={false}
        position={[-vehicle.rear_axle_m, 0.33, vehicle.rear_track_m / 2]}
      />
    </group>
  );
}
export function Vehicle({
  vehicle,
  asset,
  session,
  debug = false,
}: {
  vehicle: Manifest["vehicle"];
  asset?: AssetAdapter;
  session: Session;
  debug?: boolean;
}) {
  const fallback = <Placeholder vehicle={vehicle} />;
  return asset ? (
    <AssetBoundary key={asset.url} fallback={fallback} onError={asset.onError}>
      <Suspense fallback={fallback}>
        <Imported
          asset={asset}
          vehicle={vehicle}
          session={session}
          debug={debug}
        />
      </Suspense>
    </AssetBoundary>
  ) : (
    fallback
  );
}
