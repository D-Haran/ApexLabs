import {
  Component,
  Suspense,
  useEffect,
  useMemo,
  useRef,
  useState,
  type ReactNode,
} from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { OrbitControls, useGLTF } from "@react-three/drei";
import * as THREE from "three";
import {
  parseCarManifest,
  wheelNames,
  type CarManifest,
} from "../three/carManifest";
import {
  parseContactRecording,
  sampleContact,
  toRender,
  toRenderQuaternion,
  type ContactRecording,
  type ContactSample,
} from "./contract";
import "./contact.css";

function Surface({ recording }: { recording: ContactRecording }) {
  const geometry = useMemo(() => {
    const m = recording.surface,
      positions: number[] = [],
      normals: number[] = [],
      colors: number[] = [],
      indices: number[] = [];
    const palette = ["#353e45", "#d0ac4e", "#5a6060", "#52643b"].map(
      (c) => new THREE.Color(c),
    );
    for (let i = 0; i < m.materials.length; i++) {
      positions.push(
        ...toRender(
          m.positions.slice(i * 3, i * 3 + 3) as [number, number, number],
        ),
      );
      normals.push(
        ...toRender(
          m.normals.slice(i * 3, i * 3 + 3) as [number, number, number],
        ),
      );
      const c = palette[m.materials[i]];
      colors.push(c.r, c.g, c.b);
      if (
        i % m.columns < m.columns - 1 &&
        i + m.columns + 1 < m.materials.length
      )
        indices.push(
          i,
          i + m.columns,
          i + 1,
          i + 1,
          i + m.columns,
          i + m.columns + 1,
        );
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    g.setAttribute("normal", new THREE.Float32BufferAttribute(normals, 3));
    g.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
    g.setIndex(indices);
    return g;
  }, [recording]);
  useEffect(() => () => geometry.dispose(), [geometry]);
  return (
    <mesh geometry={geometry} receiveShadow>
      <meshStandardMaterial
        vertexColors
        roughness={0.95}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
}
export function Car({
  sample,
  manifest,
  recording,
  onReady,
  assetPath = "/assets/mclaren-p1.glb",
  ghost = false,
  onFrameCost,
}: {
  sample: ContactSample;
  manifest: CarManifest;
  recording: Pick<ContactRecording, "parameters">;
  assetPath?: string;
  ghost?: boolean;
  onReady: () => void;
  onFrameCost?: (ms: number) => void;
}) {
  const asset = useGLTF(assetPath);
  const rig = useMemo(() => {
    const scene = asset.scene.clone(true),
      body = scene.getObjectByName(manifest.bodyNode);
    if (!body) throw new Error("Missing car body node");
    body.position.set(
      recording.parameters.body_axle_midpoint_m,
      -recording.parameters.cg_height_m,
      0,
    );
    const wheels = wheelNames.map((key, i) => {
      const steer = scene.getObjectByName(manifest.wheels[key].steerPivot),
        spin = scene.getObjectByName(manifest.wheels[key].spinPivot);
      if (!steer || !spin) throw new Error("Missing car wheel node");
      const scale =
        recording.parameters.corner[i].radius_m / manifest.wheels[key].radius;
      steer.scale.setScalar(scale);
      return { steer, spin };
    });
    scene.traverse((object) => {
      if (object instanceof THREE.Mesh) {
        object.castShadow = !ghost;
        if (ghost) {
          const materials = Array.isArray(object.material)
            ? object.material
            : [object.material];
          const clones = materials.map((m) => {
            const c = m.clone();
            c.transparent = true;
            c.opacity = 0.28;
            c.depthWrite = false;
            return c;
          });
          object.material = Array.isArray(object.material) ? clones : clones[0];
        }
        object.receiveShadow = true;
      }
    });
    return { scene, wheels };
  }, [asset.scene, manifest, recording, ghost]);
  const readySent = useRef(false);
  useEffect(() => {
    readySent.current = false;
  }, [rig]);
  useFrame(() => {
    const started = performance.now();
    rig.scene.position.fromArray(toRender(sample.position));
    rig.scene.quaternion.copy(toRenderQuaternion(sample.quaternion));
    const inverse = rig.scene.quaternion.clone().invert();
    rig.wheels.forEach(({ steer, spin }, i) => {
      steer.position
        .fromArray(toRender(sample.wheels[i].center))
        .sub(rig.scene.position)
        .applyQuaternion(inverse);
      steer.rotation.y = i < 2 ? sample.steering : 0;
      // Visual spin only: no wheel rotational state exists in Phase A.
      spin.rotation.z =
        -((sample as ContactSample & { s?: number }).s ?? sample.position[0]) /
        recording.parameters.corner[i].radius_m;
    });
    if (!readySent.current) {
      readySent.current = true;
      onReady();
    }
    onFrameCost?.(performance.now() - started);
  });
  return <primitive object={rig.scene} />;
}
export function Contacts({
  sample,
  onInspect,
  onFrameCost,
}: {
  sample: ContactSample;
  onInspect?: (wheel: number) => void;
  onFrameCost?: (ms: number) => void;
}) {
  const dots = useRef<(THREE.Mesh | null)[]>([]);
  const arrows = useMemo(
    () =>
      Array.from(
        { length: 4 },
        () =>
          new THREE.ArrowHelper(
            new THREE.Vector3(0, 1, 0),
            new THREE.Vector3(),
            1,
            "#65dc99",
          ),
      ),
    [],
  );
  useEffect(
    () => () =>
      arrows.forEach((a) => {
        a.line.geometry.dispose();
        a.cone.geometry.dispose();
      }),
    [arrows],
  );
  useFrame(() => {
    const started = performance.now();
    arrows.forEach((a, i) => {
      const w = sample.wheels[i],
        vector = new THREE.Vector3(...toRender(w.force)),
        magnitude = vector.length();
      dots.current[i]?.position.fromArray(toRender(w.center));
      const material = dots.current[i]?.material as
        THREE.MeshBasicMaterial | undefined;
      material?.color.set(w.contact ? "#59e398" : "#ff755d");
      a.position.fromArray(toRender(w.patch));
      a.visible = w.contact && magnitude > 0;
      if (a.visible) {
        a.setDirection(vector.normalize());
        a.setLength(Math.min(magnitude / 5000, 3), 0.2, 0.1);
      }
    });
    onFrameCost?.(performance.now() - started);
  });
  return (
    <>
      {arrows.map((a, i) => (
        <primitive
          key={i}
          object={a}
          onClick={(event: { stopPropagation: () => void }) => {
            event.stopPropagation();
            onInspect?.(i);
          }}
        />
      ))}
      {sample.wheels.map((w, i) => (
        <mesh
          ref={(node) => {
            dots.current[i] = node;
          }}
          key={i}
          position={toRender(w.center)}
          onClick={() => onInspect?.(i)}
        >
          <sphereGeometry args={[0.055, 12, 8]} />
          <meshBasicMaterial
            color={w.contact ? "#59e398" : "#ff755d"}
            depthTest={false}
          />
        </mesh>
      ))}
    </>
  );
}
function Camera({ sample }: { sample: ContactSample }) {
  const control = useRef<React.ComponentRef<typeof OrbitControls>>(null);
  const previous = useRef(new THREE.Vector3(...toRender(sample.position)));
  useFrame(({ camera }) => {
    const target = new THREE.Vector3(...toRender(sample.position));
    camera.position.add(target.clone().sub(previous.current));
    if (control.current) {
      control.current.target.copy(target);
      control.current.update();
    }
    previous.current.copy(target);
  });
  return (
    <OrbitControls ref={control} makeDefault maxDistance={70} minDistance={3} />
  );
}
class AssetError extends Component<
  { children: ReactNode; onError: (message: string) => void },
  { error: string }
> {
  state = { error: "" };
  static getDerivedStateFromError(error: Error) {
    return { error: error.message };
  }
  componentDidCatch(error: Error) {
    this.props.onError(`Car asset unavailable: ${error.message}`);
  }
  render() {
    return this.state.error ? null : this.props.children;
  }
}
export default function ContactLab() {
  const [scene, setScene] = useState("crest-40"),
    [recording, setRecording] = useState<ContactRecording>(),
    [manifest, setManifest] = useState<CarManifest>(),
    [error, setError] = useState(""),
    [time, setTime] = useState(0),
    [playing, setPlaying] = useState(false),
    [meshReady, setMeshReady] = useState(false);
  const onReady = useMemo(() => () => setMeshReady(true), []);
  useEffect(() => {
    const abort = new AbortController();
    setRecording(undefined);
    setPlaying(false);
    setTime(0);
    setError("");
    setMeshReady(false);
    fetch(`/demo/contact-3d/${scene}.json`, { signal: abort.signal })
      .then((r) => {
        if (!r.ok)
          throw new Error(
            "Generate contact replays with run_contact_validation.py",
          );
        return r.json();
      })
      .then((v) => setRecording(parseContactRecording(v)))
      .catch((e) => {
        if (!abort.signal.aborted) setError(String(e));
      });
    return () => abort.abort();
  }, [scene]);
  useEffect(() => {
    fetch("/assets/mclaren-p1.json")
      .then((r) => r.json())
      .then((v) => setManifest(parseCarManifest(v)))
      .catch((e) => setError(String(e)));
  }, []);
  const duration = recording?.samples.at(-1)?.time ?? 0;
  useEffect(() => {
    if (!playing) return;
    let frame = 0,
      last = performance.now();
    const tick = (now: number) => {
      const dt = Math.min((now - last) / 1000, 0.05);
      last = now;
      setTime((t) => Math.min(duration, t + dt));
      frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(frame);
  }, [playing, duration]);
  useEffect(() => {
    if (duration && time >= duration) setPlaying(false);
  }, [duration, time]);
  const sample = useMemo(
    () => (recording ? sampleContact(recording, time) : undefined),
    [recording, time],
  );
  return (
    <main className="contact-lab">
      <header>
        <a href="/">◀ ApexLab</a>
        <strong>3D CONTACT VALIDATION</strong>
        <span>Phase A · synthetic test intervals</span>
        <select
          aria-label="Contact scene"
          value={scene}
          onChange={(e) => setScene(e.target.value)}
        >
          <option value="crest-40">Crest · 40 m/s</option>
          <option value="crest-12">Crest · 12 m/s</option>
          <option value="drop">Drop · 0.5 m</option>
          <option value="curb">Left curb</option>
          <option value="grass">Grass coast</option>
        </select>
      </header>
      {error && <p role="alert">{error}</p>}
      {sample && recording ? (
        <>
          <section className="contact-stage">
            <Canvas
              shadows
              camera={{ position: [sample.position[0] - 9, 5, 9], fov: 48 }}
              gl={{ toneMapping: THREE.ACESFilmicToneMapping }}
            >
              <color attach="background" args={["#253441"]} />
              <ambientLight intensity={1.3} />
              <directionalLight
                position={[sample.position[0] - 15, 35, 15]}
                intensity={3}
                castShadow
                shadow-mapSize={[2048, 2048]}
                shadow-camera-left={-50}
                shadow-camera-right={50}
                shadow-camera-top={30}
                shadow-camera-bottom={-30}
                shadow-bias={-0.0002}
              />
              <Surface recording={recording} />
              {manifest && (
                <AssetError onError={setError}>
                  <Suspense fallback={null}>
                    <Car
                      sample={sample}
                      manifest={manifest}
                      recording={recording}
                      onReady={onReady}
                    />
                  </Suspense>
                </AssetError>
              )}
              <Contacts sample={sample} />
              <Camera key={scene} sample={sample} />
            </Canvas>
            <div className="contact-hud">
              <b>
                {sample.wheels.filter((w) => w.contact).length} / 4 CONTACTS
              </b>
              <span>
                {(Math.hypot(...sample.velocity) * 3.6).toFixed(1)} km/h
              </span>
              <span>
                World vertical acceleration{" "}
                {(sample.acceleration[2] / 9.80665).toFixed(2)} g
              </span>
              <small>
                Arrows: road force on vehicle · 5 kN/m, capped at 3 m
              </small>
              <small>
                {meshReady
                  ? "P1 mesh ready · recorded contact pose"
                  : "Loading P1 mesh…"}
              </small>
            </div>
          </section>
          <div className="contact-playback">
            <button
              onClick={() => {
                if (time >= duration) setTime(0);
                setPlaying(!playing);
              }}
            >
              {playing ? "Pause" : "Play"}
            </button>
            <button
              onClick={() => {
                setTime(0);
                setPlaying(false);
              }}
            >
              Reset
            </button>
            <input
              aria-label="Contact replay time"
              type="range"
              min={0}
              max={duration}
              step={0.001}
              value={time}
              onChange={(e) => {
                setTime(Number(e.target.value));
                setPlaying(false);
              }}
            />
            <output>
              {time.toFixed(3)} / {duration.toFixed(3)} s
            </output>
          </div>
          <section className="contact-readouts">
            <div>
              <h2>Recorded wheel state</h2>
              <table>
                <thead>
                  <tr>
                    <th>Wheel</th>
                    <th>Contact</th>
                    <th>Fz</th>
                    <th>Fx / Fy</th>
                    <th>Utilization</th>
                    <th>Spring compression</th>
                    <th>Damper velocity</th>
                  </tr>
                </thead>
                <tbody>
                  {sample.wheels.map((w, i) => (
                    <tr key={i}>
                      <th>{wheelNames[i]}</th>
                      <td className={w.contact ? "contact-on" : "contact-off"}>
                        {w.contact ? "Contacting" : "Airborne"}
                      </td>
                      <td>{(w.fz / 1000).toFixed(2)} kN</td>
                      <td>
                        {w.fx.toFixed(0)} / {w.fy.toFixed(0)} N
                      </td>
                      <td>{(w.utilization * 100).toFixed(1)}%</td>
                      <td>{(w.compression * 1000).toFixed(1)} mm</td>
                      <td>{w.damperVelocity.toFixed(3)} m/s</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
            <div>
              <h2>Experiment</h2>
              <p>
                Full airborne duration{" "}
                <b>{recording.summary.all_airborne_duration_s.toFixed(3)} s</b>
              </p>
              <p>
                Peak total normal force{" "}
                <b>
                  {(recording.summary.peak_total_normal_force_n / 1000).toFixed(
                    1,
                  )}{" "}
                  kN
                </b>
              </p>
              <p>
                Native physics step <b>{recording.summary.dt_s * 1000} ms</b>
              </p>
              <small>
                Estimated P1-based mechanical fixture. Aero disabled for these
                benches. Recorded body and unsprung motion; kinematic visual
                wheel spin. No lap-time or calibrated road-car claim.
              </small>
            </div>
          </section>
        </>
      ) : (
        !error && <p>Loading contact recording…</p>
      )}
    </main>
  );
}
