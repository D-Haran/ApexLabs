import { Profiler, useEffect, useMemo, useRef, useState } from "react";
import { DriveClock } from "./clock";
import { measure, recordCost } from "./performance";
import type { ChaseMode } from "./camera";
import DriveScene from "./DriveScene";
import {
  ghostAt,
  sideslip,
  inputAxis,
  forceComponents,
  spatialDelta,
  type DriveMetadata,
  type DriveSample,
  type Ghost,
} from "./contract";
import {
  parseCarManifest,
  type CarManifest,
  wheelNames,
} from "../three/carManifest";
import "./drive.css";
const API = "http://127.0.0.1:8765";
const clientId = crypto.randomUUID();
let mutations: Promise<unknown> = Promise.resolve();
function api(path: string, body?: unknown): Promise<any> {
  if (!body) return request(path);
  const result = mutations.then(() => request(path, body));
  mutations = result.catch(() => {});
  return result;
}
async function request(path: string, body?: unknown) {
  const r = await fetch(API + path, {
    method: body ? "POST" : "GET",
    headers: body ? { "Content-Type": "application/json" } : undefined,
    body: body
      ? JSON.stringify({ ...(body as Record<string, unknown>), clientId })
      : undefined,
    signal: AbortSignal.timeout(path === "/command" ? 2000 : 15000),
  });
  const j = await r.json();
  if (!r.ok) throw Error(j.error || r.statusText);
  return j;
}
const fmt = (v: number, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : "—");
export default function DriveLab() {
  const [meta, setMeta] = useState<DriveMetadata>(),
    [liveSample, setSample] = useState<DriveSample>(),
    [manifest, setManifest] = useState<CarManifest>(),
    [ghost, setGhost] = useState<Ghost>();
  const [vehicle, setVehicle] = useState("mclaren-p1"),
    [track, setTrack] = useState("spa-francorchamps"),
    [error, setError] = useState(""),
    [connected, setConnected] = useState(false);
  const [forces, setForces] = useState(false),
    [reference, setReference] = useState(false),
    [overview, setOverview] = useState(false),
    [ghostMode, setGhostMode] = useState<"time" | "s">("time");
  const [assisted, setAssisted] = useState(true),
    [chaseMode, setChaseMode] = useState<ChaseMode>("Standard"),
    [shaping, setShaping] = useState({
      inputRise: 1.5,
      inputReturn: 2.5,
      throttleRise: 1.5,
      throttleRelease: 3,
      brakeRise: 3,
      brakeRelease: 5,
    }),
    [deadzone, setDeadzone] = useState(0.08),
    [steeringRate, setSteeringRate] = useState(1.4),
    [saturation, setSaturation] = useState(0.65),
    [deployment, setDeployment] = useState(1),
    [drs, setDrs] = useState(false),
    [aeroMode, setAeroMode] = useState(1);
  const [extra, setExtra] = useState({
    policy: true,
    velocity: false,
    aero: false,
    cg: false,
    contacts: false,
    road: false,
    loads: false,
    boundaries: true,
    player: true,
  });
  const [bestDelta, setBestDelta] = useState<number>(),
    [sectorDelta, setSectorDelta] = useState<{
      sector: number;
      delta: number;
    }>();
  const previousProgress = useRef(0);
  const [view, setView] = useState<
      "drive" | "reference" | "optimized" | "compare"
    >("drive"),
    [referenceLap, setReferenceLap] = useState<Ghost>();
  const [selectedTire, setSelectedTire] = useState<number>(),
    [forceFrame, setForceFrame] = useState<"Tire Local" | "Body" | "World">(
      "Tire Local",
    );
  const [objective, setObjective] = useState("fastest"),
    [weights, setWeights] = useState([0.7, 0, 0, 0, 0.1, 0.1, 0.1]),
    [optimization, setOptimization] = useState<Record<string, unknown>>({}),
    [improve, setImprove] = useState(false);
  const [history, setHistory] = useState<DriveSample[]>([]),
    [provenance, setProvenance] = useState(false);
  const publishInput = useRef(() => {});
  const keys = useRef(new Set<string>()),
    input = useRef({
      assisted,
      shaping,
      deadzone,
      steeringRate,
      saturation,
      deployment,
      drs,
      aeroMode,
    });
  input.current = {
    assisted,
    shaping,
    deadzone,
    steeringRate,
    saturation,
    deployment,
    drs,
    aeroMode,
  };
  const running = useRef(false);
  const [commandPending, setCommandPending] = useState(false);
  const historyTime = useRef(-1);
  const pendingObjective = useRef<string | undefined>(undefined);
  const clock = useMemo(() => new DriveClock(), []);
  const [, refresh] = useState(0);
  const replaySource = view === "reference" ? referenceLap : ghost;
  const replayTime = clock.simulationTime;
  const replayPlaying = clock.playbackState === "playing";
  const sample = clock.currentSample ?? liveSample;
  const setReplayTime = (time: number) => {
    clock.seek(time);
    clock.read();
    refresh((v) => v + 1);
  };
  const setReplayPlaying = (play: boolean) => {
    if (play) clock.play();
    else clock.pause();
    clock.read();
    refresh((v) => v + 1);
  };
  useEffect(() => {
    clock.load(view === "drive" ? undefined : replaySource);
    clock.read();
    refresh((v) => v + 1);
  }, [clock, view, replaySource]);
  useEffect(() => {
    const timer = setInterval(() => {
      refresh((v) => v + 1);
    }, 100);
    return () => clearInterval(timer);
  }, []);

  useEffect(() => {
    if (!connected) return;
    const timer = setInterval(() => {
      void api("/optimization")
        .then(async (status) => {
          setOptimization(status);
          if (status.running || !pendingObjective.current) return;
          const completed = pendingObjective.current;
          pendingObjective.current = undefined;
          if (status.status !== "accepted") return;
          await command({ running: false });
          const g: Ghost = await api(`/ghost?objective=${completed}`);
          const r: Ghost = await api(`/reference?objective=${completed}`);
          if (g.accepted && g.vehicle === vehicle && g.track === track) {
            setGhost(g);
            setReferenceLap(r);
            setReplayTime(0);
            setView("optimized");
          }
        })
        .catch(() => {});
    }, 1500);
    return () => clearInterval(timer);
  }, [connected, vehicle, track]);
  async function optimize() {
    try {
      setOptimization(await api("/optimize", { objective, weights, nodes: 4 }));
      pendingObjective.current = objective;
    } catch (e) {
      setError(String(e));
    }
  }

  useEffect(() => {
    let gone = false;
    setConnected(false);
    setSample(undefined);
    clock.clearLive();
    setGhost(undefined);
    setReferenceLap(undefined);
    setView("drive");
    setReplayTime(0);
    setHistory([]);
    historyTime.current = -1;
    setError("");
    pendingObjective.current = undefined;
    (async () => {
      await api("/configure", { vehicle, track });
      const m: DriveMetadata = await api("/metadata");
      const manifestResponse = await fetch(
        `/assets/${m.family === "p1" ? "mclaren-p1" : "mclaren-f1-2022"}.json`,
      );
      const visual = parseCarManifest(await manifestResponse.json());
      const s: DriveSample = await api("/state");
      if (gone) return;
      setMeta(m);
      setManifest(visual);
      clock.push(s);
      clock.read();
      setSample(s);
      running.current = false;
      try {
        const g: Ghost = await api("/ghost");
        const r: Ghost = await api("/reference");
        if (!gone && g.accepted && g.model === "dynamic_contact") {
          setGhost(g);
          setReferenceLap(r);
        }
      } catch {
        /* Explicit unavailable state below. */
      }
      if (!gone) setConnected(true);
    })().catch((e) => {
      if (!gone)
        setError(
          `Native server unavailable: ${e.message}. Run python3 tools/dynamics/server.py`,
        );
    });
    return () => {
      gone = true;
      running.current = false;
      void api("/command", {
        running: false,
        throttle: 0,
        brake: 0,
        steering: 0,
      }).catch(() => {});
    };
  }, [vehicle, track]);
  useEffect(() => {
    const down = (e: KeyboardEvent) => {
      if ((e.target as HTMLElement).closest("input,select,textarea")) return;
      if (
        [
          "KeyW",
          "KeyS",
          "KeyA",
          "KeyD",
          "ArrowUp",
          "ArrowDown",
          "ArrowLeft",
          "ArrowRight",
        ].includes(e.code)
      ) {
        e.preventDefault();
        keys.current.add(e.code);
        publishInput.current();
      }
    };
    const up = (e: KeyboardEvent) => {
      keys.current.delete(e.code);
      publishInput.current();
    };
    const clear = () => {
      keys.current.clear();
      void api("/command", {
        throttle: 0,
        brake: 0,
        steering: 0,
        steerKey: 0,
      }).catch(() => {});
    };
    window.addEventListener("keydown", down);
    window.addEventListener("keyup", up);
    window.addEventListener("blur", clear);
    return () => {
      window.removeEventListener("keydown", down);
      window.removeEventListener("keyup", up);
      window.removeEventListener("blur", clear);
    };
  }, []);
  useEffect(() => {
    if (!connected) return;
    let busy = false,
      gone = false,
      dirty = false;
    const sendInput = async () => {
      if (busy) {
        dirty = true;
        return;
      }
      dirty = false;
      busy = true;
      const c = input.current,
        k = keys.current,
        pad = Array.from(navigator.getGamepads?.() ?? []).find(Boolean);
      let steering =
        (k.has("KeyA") || k.has("ArrowLeft") ? 1 : 0) -
        (k.has("KeyD") || k.has("ArrowRight") ? 1 : 0);
      let throttle = k.has("KeyW") || k.has("ArrowUp") ? 1 : 0,
        brake = k.has("KeyS") || k.has("ArrowDown") ? 1 : 0;
      const analog = pad && steering === 0 && throttle === 0 && brake === 0;
      if (analog) {
        steering = -inputAxis(pad.axes[0] ?? 0, c.deadzone);
        throttle = Math.max(throttle, pad.buttons[7]?.value ?? 0);
        brake = Math.max(brake, pad.buttons[6]?.value ?? 0);
      }
      try {
        const s: DriveSample = await api("/command", {
          steering: steering * c.saturation,
          throttle,
          brake,
          assisted: false,
          keyboardFriendly: c.assisted && !analog,
          steerKey: steering,
          ...c.shaping,
          steeringRate: c.steeringRate,
          saturation: c.saturation,
          deployment: c.deployment,
          drs: c.drs,
          aeroMode: c.aeroMode,
        });
        if (!gone) {
          clock.push(s);
          if (s.error) setError(s.error ?? "");
          if (s.running && s.time - (historyTime.current ?? -1) >= 0.1) {
            historyTime.current = s.time;
            setHistory((h) =>
              h.at(-1)?.time === s.time ? h : [...h.slice(-1999), s],
            );
          }
        }
      } catch (e) {
        if (!gone) {
          setError(String(e));
          if (String(e).includes("another browser")) setConnected(false);
          running.current = false;
        }
      } finally {
        busy = false;
        if (dirty && !gone) void sendInput();
      }
    };
    publishInput.current = () => {
      void sendInput();
    };
    const timer = setInterval(() => {
      if (!busy) void sendInput();
    }, 40);
    return () => {
      gone = true;
      publishInput.current = () => {};
      clearInterval(timer);
    };
  }, [connected]);
  async function command(c: Record<string, unknown>) {
    setCommandPending(true);
    try {
      const s: DriveSample = await api("/command", c);
      clock.push(s);
      clock.read();
      setSample(s);
      running.current = !!s.running;
      if (c.reset) {
        setHistory([]);
        historyTime.current = -1;
      }
    } catch (e) {
      setError(String(e));
    } finally {
      setCommandPending(false);
    }
  }
  const distance =
    sample && meta ? ((sample.s % meta.length) + meta.length) % meta.length : 0;
  const comparingReference = view === "compare" || view === "optimized";
  const delta = spatialDelta(
    sample,
    comparingReference ? referenceLap : ghost,
    distance,
  );
  function seekDistance(s: number) {
    if (view === "drive" || !replaySource) return;
    setReplayPlaying(false);
    setReplayTime(ghostAt(replaySource.samples, s, "s")?.time ?? 0);
  }
  useEffect(() => {
    if (view !== "drive" || !sample || !meta) return;
    if (sample.time === 0) {
      setBestDelta(undefined);
      setSectorDelta(undefined);
      previousProgress.current = 0;
      return;
    }
    if (delta !== undefined) {
      setBestDelta((d) => (d === undefined ? delta : Math.min(d, delta)));
      const sector = meta.sectors.findIndex(
        (f) =>
          previousProgress.current < f * meta.length &&
          distance >= f * meta.length,
      );
      if (sector >= 0) setSectorDelta({ sector: sector + 1, delta });
    }
    previousProgress.current = distance;
  }, [sample?.time, delta, view, meta]);
  return (
    <Profiler
      id="DriveLab"
      onRender={(_, __, duration) => recordCost("React rendering", duration)}
    >
      <main className="drive-lab">
        <header>
          <a className="drive-brand" href="/">
            ◢ <b>ApexLab</b>
          </a>
          <b className="active">DRIVE</b>
          <button
            onClick={() => {
              void command({ running: false });
              if (ghost) setView("optimized");
            }}
          >
            ANALYZE
          </button>
          <button onClick={() => setImprove(!improve)}>IMPROVE</button>
          <span className="drive-nav">
            <button
              onClick={() =>
                document
                  .querySelector<HTMLSelectElement>('[aria-label="Circuit"]')
                  ?.focus()
              }
            >
              Track
            </button>
            <button
              onClick={() => {
                setImprove(false);
                document
                  .querySelector<HTMLInputElement>(
                    '[aria-label="Steering rate"]',
                  )
                  ?.focus();
              }}
            >
              Car Setup
            </button>
            <button
              onClick={() =>
                document
                  .querySelector(".drive-bottom")
                  ?.scrollIntoView({ behavior: "smooth" })
              }
            >
              Telemetry
            </button>
            <button
              onClick={() => {
                if (ghost) {
                  void command({ running: false });
                  setView("optimized");
                }
              }}
            >
              Analysis
            </button>
            <button
              disabled={!ghost || !referenceLap}
              onClick={() => {
                void command({ running: false });
                setView("compare");
              }}
            >
              Compare
            </button>
          </span>
          <select
            aria-label="Circuit"
            value={track}
            onChange={(e) => setTrack(e.target.value)}
          >
            {[
              ["spa-francorchamps", "Spa-Francorchamps"],
              ["monza", "Monza"],
              ["silverstone", "Silverstone"],
              ["suzuka", "Suzuka"],
              ["technical_test_circuit", "Technical Test Circuit"],
            ].map(([v, l]) => (
              <option key={v} value={v}>
                {l}
              </option>
            ))}
          </select>
          <button
            aria-label="Data provenance"
            onClick={() => setProvenance(!provenance)}
          >
            ⓘ
          </button>
        </header>
        {error && (
          <div role="alert" className="drive-error">
            {error}
          </div>
        )}
        {provenance && (
          <aside className="drive-provenance">
            Approximate contact dynamics · OSM circuit geometry ·
            terrain-derived elevation · estimated aero, thermal and wear maps ·
            calibrated P1 performance anchors. Fuel acts at the sprung CG. No
            ABS, traction control or yaw correction. Keyboard Friendly shapes
            commands only. Wheel spin is a visual approximation.{" "}
            <a href="/?contact">Contact validation lab</a>
            <details>
              <summary>Parameter classes and geometry sources</summary>
              <pre>{JSON.stringify(meta?.provenance, null, 2)}</pre>
            </details>
          </aside>
        )}
        <div className="drive-main">
          <aside className="drive-panel drive-session">
            <h2>SESSION</h2>
            <label>
              Car
              <select
                aria-label="Vehicle"
                value={vehicle}
                onChange={(e) => setVehicle(e.target.value)}
              >
                <option value="mclaren-p1">McLaren P1 — approximate</option>
                <option value="mcl36">MCL36-inspired — approximate</option>
              </select>
            </label>
            <label>
              Driver mode
              <select
                aria-label="Input mode"
                value={assisted ? "assisted" : "authentic"}
                onChange={(e) => setAssisted(e.target.value === "assisted")}
              >
                <option value="authentic">Raw / Research</option>
                <option value="assisted">Keyboard Friendly</option>
              </select>
            </label>
            <p className="muted">No ABS · No traction control</p>
            <hr />
            <label>
              Session view
              <select
                aria-label="Session view"
                value={view}
                onChange={(e) => {
                  setView(e.target.value as typeof view);
                  setReplayTime(0);
                  setReplayPlaying(false);
                  void command({ running: false });
                }}
              >
                <option value="drive">Live driving</option>
                <option value="reference" disabled={!referenceLap}>
                  Reference replay
                </option>
                <option value="optimized" disabled={!ghost}>
                  Optimized policy replay
                </option>
                <option value="compare" disabled={!ghost || !referenceLap}>
                  Compare
                </option>
              </select>
            </label>
            <h2>{view === "drive" ? "LIVE SESSION" : "RECORDED SESSION"}</h2>
            <p>
              Lap <b>{(sample?.lap ?? 0) + 1}</b>
            </p>
            <p>
              Elapsed <b>{fmt(sample?.lapElapsed ?? 0, 3)} s</b>
            </p>
            <p>
              Progress <b>{fmt(distance, 0)} m</b>
            </p>
            <h2>DELTA</h2>
            <strong className="drive-delta">
              {delta === undefined
                ? "—"
                : `${delta >= 0 ? "+" : ""}${fmt(delta, 3)}`}
            </strong>
            <small>
              {ghost
                ? comparingReference
                  ? "vs reference lap"
                  : "vs validated optimized policy"
                : "No validated dynamic-model ghost available"}
            </small>
            {view === "drive" && bestDelta !== undefined && (
              <small>Best delta {fmt(bestDelta, 3)} s</small>
            )}
            {view === "drive" && sectorDelta && (
              <small>
                S{sectorDelta.sector} cumulative delta{" "}
                {fmt(sectorDelta.delta, 3)} s
              </small>
            )}
            {ghost && referenceLap && (
              <small>
                Reference {fmt(referenceLap.lap_time_s, 3)} s ·{" "}
                {ghost.objective_name ?? "fastest"} policy{" "}
                {fmt(ghost.lap_time_s, 3)} s
              </small>
            )}
            <label>
              Ghost mode
              <select
                aria-label="Ghost mode"
                value={ghostMode}
                onChange={(e) => setGhostMode(e.target.value as "time" | "s")}
              >
                <option value="time">Time Ghost</option>
                <option value="s">Distance Reference</option>
              </select>
            </label>
            <hr />
            <div className="drive-actions">
              <button
                onClick={() =>
                  view === "drive"
                    ? void command({ running: !running.current })
                    : setReplayPlaying(!replayPlaying)
                }
                disabled={!connected || commandPending}
              >
                {view === "drive"
                  ? sample?.running
                    ? "Pause driving"
                    : "Start driving"
                  : replayPlaying
                    ? "Pause replay"
                    : "Play replay"}
              </button>
              <button
                onClick={() =>
                  view !== "drive"
                    ? setReplayTime(replaySource?.samples[0]?.time ?? 0)
                    : void command({
                        reset: true,
                        running: false,
                        throttle: 0,
                        brake: 0,
                        steering: 0,
                      })
                }
              >
                Reset session
              </button>
              {view !== "drive" && (
                <button
                  onClick={() => {
                    clock.step();
                    clock.read();
                    refresh((v) => v + 1);
                  }}
                >
                  Frame step
                </button>
              )}
            </div>
            <small>
              {!connected
                ? "Loading native session and replay data…"
                : sample?.performance
                  ? `500 Hz native · ${fmt(sample.performance.stepBatchMeanMs, 2)} ms / 10 ms batch`
                  : "Waiting for native server"}
            </small>
          </aside>
          <section className="drive-viewport">
            {sample && meta && manifest && (
              <DriveScene
                clock={clock}
                chaseMode={chaseMode}
                ghostSource={
                  view === "compare"
                    ? referenceLap
                    : view === "drive"
                      ? ghost
                      : undefined
                }
                ghostMode={view === "compare" ? "s" : ghostMode}
                onInspect={setSelectedTire}
                sample={sample}
                meta={meta}
                manifest={manifest}
                forces={forces}
                reference={reference}
                extra={extra}
                policyPath={ghost?.samples}
                overview={overview}
                history={view === "drive" ? history : []}
              />
            )}
            <div className="drive-hud-top">
              <small>
                LAP {(sample?.lap ?? 0) + 1} ·{" "}
                {assisted ? "KEYBOARD FRIENDLY" : "RAW / RESEARCH"}
              </small>
              <strong>{fmt(sample?.lapElapsed ?? 0, 3)}</strong>
              <span>{meta?.track ?? "Connecting to native simulator"}</span>
            </div>
            {view !== "drive" && replaySource && (
              <input
                className="drive-replay-seek"
                aria-label="Replay time"
                type="range"
                min={replaySource.samples[0]?.time ?? 0}
                max={replaySource.samples.at(-1)?.time ?? 0}
                step="any"
                value={replayTime}
                onChange={(e) => {
                  setReplayPlaying(false);
                  setReplayTime(+e.target.value);
                }}
              />
            )}
            <div className="drive-speed">
              <strong>
                {fmt(Math.hypot(...(sample?.velocity ?? [0, 0, 0])) * 3.6, 0)}
              </strong>
              <span>km/h</span>
              <b>{sample?.gear ?? "—"}</b>
              <small>{fmt(sample?.rpm ?? 0, 0)} rpm</small>
            </div>
            <div className="drive-overlays">
              <label>
                <input
                  type="checkbox"
                  checked={forces}
                  onChange={(e) => setForces(e.target.checked)}
                />
                Tire forces
              </label>
              <label>
                <input
                  type="checkbox"
                  checked={reference}
                  onChange={(e) => setReference(e.target.checked)}
                />
                Reference path
              </label>
              <details className="drive-overlay-menu">
                <summary>Overlays</summary>
                {Object.entries(extra).map(([key, on]) => (
                  <label key={key}>
                    <input
                      type="checkbox"
                      checked={on}
                      onChange={(e) =>
                        setExtra((o) => ({ ...o, [key]: e.target.checked }))
                      }
                    />
                    {
                      {
                        policy: "Policy trajectory",
                        velocity: "Velocity vector",
                        aero: "Aero force",
                        cg: "CG",
                        contacts: "Suspension/contact points",
                        road: "Road frame",
                        loads: "Tire loads",
                        boundaries: "Track boundaries",
                        player: "Player trajectory",
                      }[key]
                    }
                  </label>
                ))}
                <label>
                  <input type="checkbox" disabled />
                  Optimized path — Not available: free-path optimizer incomplete
                </label>
              </details>
              <select
                aria-label="Chase response"
                value={chaseMode}
                onChange={(e) => setChaseMode(e.target.value as ChaseMode)}
              >
                {["Tight", "Standard", "Cinematic"].map((v) => (
                  <option key={v}>{v}</option>
                ))}
              </select>
              <button onClick={() => setOverview(!overview)}>
                {overview ? "Chase camera" : "Orbit camera"}
              </button>
            </div>
            {forces && (
              <small className="drive-force-caption">
                Road force on vehicle · world arrows at contact patches · 5 kN/m
              </small>
            )}
          </section>
          <aside className="drive-right">
            <section className="drive-panel drive-map">
              <h2>TRACK MAP</h2>
              {meta && (
                <Map
                  meta={meta}
                  sample={sample}
                  ghost={ghost}
                  referenceLap={referenceLap}
                  onSeek={seekDistance}
                />
              )}
              <small>
                Gray: reference · Blue: current · Green: policy · Gold: sector
                divisions
              </small>
            </section>
            <div className="drive-status">
              <section className="drive-panel">
                <h2>CAR STATUS</h2>
                <div className="drive-tires">
                  {sample?.wheels.map((w, i) => (
                    <div key={i}>
                      <button
                        onClick={() => setSelectedTire(i)}
                        aria-label={`Inspect ${wheelNames[i]} tire`}
                      >
                        {wheelNames[i]}　{fmt(w.tread, 0)}°C
                      </button>
                      <meter min={0} max={1} value={w.utilization} />
                      <small>
                        {fmt(w.fz / 1000, 2)} kN · {fmt(w.wear * 100, 2)}% wear
                      </small>
                      <small>{w.contact ? "Contact" : "AIRBORNE"}</small>
                    </div>
                  ))}
                </div>
                <label>
                  Fuel <b>{fmt(sample?.fuel ?? 0, 2)} kg</b>
                  <meter
                    min={0}
                    max={meta?.values.initial_fuel_kg ?? 30}
                    value={sample?.fuel ?? 0}
                  />
                </label>
                <label>
                  Battery <b>{fmt((sample?.battery ?? 0) / 1e6, 2)} MJ</b>
                  <meter
                    min={0}
                    max={meta?.values.battery_capacity_j ?? 1}
                    value={sample?.battery ?? 0}
                  />
                </label>
              </section>
              <section className="drive-panel">
                <h2>AERO / LOAD (EST.)</h2>
                <p>
                  Downforce <b>{fmt((sample?.downforce ?? 0) / 1000, 2)} kN</b>
                </p>
                <p>
                  Drag <b>{fmt((sample?.drag ?? 0) / 1000, 2)} kN</b>
                </p>
                <p>
                  Front balance{" "}
                  <b>{fmt((sample?.frontAeroFraction ?? 0) * 100, 0)}%</b>
                </p>
                <p>
                  CdA / ClA{" "}
                  <b>
                    {fmt(sample?.cda ?? 0, 2)} / {fmt(sample?.cla ?? 0, 2)}
                  </b>
                </p>
                <hr />
                <h2>G-FORCES</h2>
                <div className="drive-g">
                  <svg viewBox="0 0 100 100">
                    <circle cx="50" cy="50" r="44" />
                    <circle cx="50" cy="50" r="22" />
                    <path d="M6 50H94M50 6V94" />
                    <circle
                      className="drive-g-dot"
                      cx={50 + (sample?.bodyAcceleration[1] ?? 0) * 1.5}
                      cy={50 - (sample?.bodyAcceleration[0] ?? 0) * 1.5}
                      r="4"
                    />
                  </svg>
                  <small>
                    Long {fmt((sample?.bodyAcceleration[0] ?? 0) / 9.80665, 2)}{" "}
                    g
                    <br />
                    Lat {fmt((sample?.bodyAcceleration[1] ?? 0) / 9.80665, 2)} g
                    <br />
                    Vert {fmt(
                      (sample?.bodyAcceleration[2] ?? 0) / 9.80665,
                      2,
                    )}{" "}
                    g
                  </small>
                </div>
              </section>
            </div>
          </aside>
        </div>
        <div className="drive-bottom">
          <section className="drive-panel">
            <h2>
              TELEMETRY{" "}
              <small>
                {view === "drive"
                  ? `Last ${fmt((history.at(-1)?.time ?? 0) - (history[0]?.time ?? 0), 0)} s`
                  : "Lap distance"}{" "}
                · native snapshots
              </small>
            </h2>
            {(["Speed", "Throttle", "Brake", "Steering"] as const).map(
              (name, i) => (
                <Trace
                  key={name}
                  name={name}
                  history={
                    view === "drive" ? history : (replaySource?.samples ?? [])
                  }
                  comparison={
                    view === "compare" ? referenceLap?.samples : undefined
                  }
                  cursor={view === "drive" ? undefined : replayTime}
                  onSeek={seekDistance}
                  channel={i}
                />
              ),
            )}
          </section>
          <section className="drive-panel">
            <h2>
              SUSPENSION / LOAD{" "}
              {selectedTire !== undefined && (
                <button onClick={() => setSelectedTire(undefined)}>
                  Close tire
                </button>
              )}
            </h2>
            {sample && (
              <details>
                <summary>Driving diagnostic</summary>
                <p>
                  Speed {fmt(Math.hypot(...sample.velocity), 2)} m/s · steer
                  command {fmt(sample.steerCommand ?? 0, 3)} · request{" "}
                  {fmt(sample.steeringRequest ?? sample.steering, 4)} rad
                </p>
                <p>
                  Road-wheel δ {fmt((sample.steering * 180) / Math.PI, 2)}°
                  (wheel equivalent{" "}
                  {fmt((sample.steering * 15 * 180) / Math.PI, 1)}° at
                  illustrative 15:1) · δ̇{" "}
                  {fmt(sample.steeringRateActual ?? 0, 3)} rad/s · yaw{" "}
                  {fmt(sample.omega[2], 3)} rad/s
                </p>
                <p>
                  β {fmt((sideslip(sample) * 180) / Math.PI, 2)}° · front − rear
                  utilization{" "}
                  {fmt(
                    (sample.wheels[0].utilization +
                      sample.wheels[1].utilization -
                      sample.wheels[2].utilization -
                      sample.wheels[3].utilization) /
                      2,
                    3,
                  )}
                </p>
                <p>
                  Throttle request {fmt(sample.throttle, 3)} · brake request{" "}
                  {fmt(sample.brake, 3)}
                </p>
                {sample.wheels.map((w, i) => (
                  <p key={i}>
                    {wheelNames[i]} α {fmt((w.slip * 180) / Math.PI, 2)}° ·
                    utilization {fmt(w.utilization, 3)}
                  </p>
                ))}
                <small>
                  Utilization balance is a diagnostic proxy, not an understeer
                  gradient. Positive means front more utilized.
                </small>
              </details>
            )}
            {sample && selectedTire !== undefined ? (
              <>
                <h2>{wheelNames[selectedTire]} · Road force on vehicle</h2>
                <select
                  aria-label="Force frame"
                  value={forceFrame}
                  onChange={(e) =>
                    setForceFrame(e.target.value as typeof forceFrame)
                  }
                >
                  {["Tire Local", "Body", "World"].map((v) => (
                    <option key={v}>{v}</option>
                  ))}
                </select>
                <p>
                  {forceComponents(sample, selectedTire, forceFrame)
                    .map((v) => fmt(v, 0))
                    .join(" / ")}{" "}
                  N
                </p>
                <small>
                  Local Fx/Fy/Fz:{" "}
                  {forceComponents(sample, selectedTire, "Tire Local")
                    .map((v) => fmt(v, 0))
                    .join(" / ")}{" "}
                  N<br />
                  Body Fx/Fy/Fz:{" "}
                  {forceComponents(sample, selectedTire, "Body")
                    .map((v) => fmt(v, 0))
                    .join(" / ")}{" "}
                  N<br />
                  World vector:{" "}
                  {forceComponents(sample, selectedTire, "World")
                    .map((v) => fmt(v, 0))
                    .join(" / ")}{" "}
                  N<br />
                  Steering:{" "}
                  {fmt(
                    ((selectedTire < 2 ? sample.steering : 0) * 180) / Math.PI,
                    2,
                  )}
                  °
                </small>
                <p>
                  Slip angle{" "}
                  <b>
                    {fmt((sample.wheels[selectedTire].slip * 180) / Math.PI, 2)}
                    °
                  </b>
                </p>
                <p>
                  Tread / carcass{" "}
                  <b>
                    {fmt(sample.wheels[selectedTire].tread, 1)} /{" "}
                    {fmt(sample.wheels[selectedTire].carcass, 1)}°C
                  </b>
                </p>
                <TireHistory
                  rows={
                    view === "drive" ? history : (replaySource?.samples ?? [])
                  }
                  sample={sample}
                  wheel={selectedTire}
                />
                <small>
                  3D arrows remain in world coordinates regardless of label
                  frame.
                </small>
              </>
            ) : (
              <>
                <table>
                  <thead>
                    <tr>
                      <th>Wheel</th>
                      <th>Travel</th>
                      <th>Damper</th>
                      <th>Fx / Fy</th>
                    </tr>
                  </thead>
                  <tbody>
                    {sample?.wheels.map((w, i) => (
                      <tr key={i}>
                        <td>{wheelNames[i]}</td>
                        <td>{fmt(w.compression * 1000, 1)} mm</td>
                        <td>{fmt(w.damperVelocity, 2)} m/s</td>
                        <td>
                          {fmt(w.fx, 0)} / {fmt(w.fy, 0)} N
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
                <p>
                  Front ride height{" "}
                  <b>{fmt((sample?.frontHeight ?? 0) * 1000, 1)} mm</b>
                </p>
                <p>
                  Rear ride height{" "}
                  <b>{fmt((sample?.rearHeight ?? 0) * 1000, 1)} mm</b>
                </p>
                <small>
                  Estimated suspension and floor map. Vertical g is inertial
                  body acceleration, not accelerometer specific force.
                </small>
              </>
            )}
          </section>
          <section className="drive-panel">
            <h2>
              <button onClick={() => setImprove(!improve)}>
                {improve ? "OPTIMIZATION" : "SETUP / DRIVER INPUT"}
              </button>
            </h2>
            {improve ? (
              <>
                <label>
                  Objective
                  <select
                    aria-label="Optimization objective"
                    value={objective}
                    onChange={(e) => setObjective(e.target.value)}
                  >
                    {[
                      ["fastest", "Fastest Lap"],
                      ["shortest", "Shortest Path"],
                      ["clearance", "Maximum Clearance"],
                      ["neutral", "Neutral Handling"],
                      ["platform", "Platform Stability"],
                      ["fuel", "Minimum Fuel"],
                      ["wear", "Tire Preservation"],
                      ["custom", "Custom"],
                    ].map(([v, l]) => (
                      <option value={v} key={v}>
                        {l}
                      </option>
                    ))}
                  </select>
                </label>
                {objective === "custom" && (
                  <div className="drive-weights">
                    {[
                      "Time",
                      "Path",
                      "Clearance",
                      "Balance",
                      "Platform",
                      "Fuel",
                      "Wear",
                    ].map((label, i) => (
                      <label key={label}>
                        {label}
                        <input
                          aria-label={`${label} weight`}
                          type="number"
                          min={0}
                          step={0.1}
                          value={weights[i]}
                          onChange={(e) =>
                            setWeights((w) =>
                              w.map((v, k) =>
                                k === i ? Math.max(0, +e.target.value) : v,
                              ),
                            )
                          }
                        />
                      </label>
                    ))}
                  </div>
                )}
                <small>
                  Native single shooting · periodic path/speed policy · body
                  half-width + 0.25 m margin · maximum time 115% of reference.
                  Local solution within the selected policy family.
                </small>
                <button
                  onClick={() => void optimize()}
                  disabled={optimization.running === true}
                >
                  Optimize
                </button>
                <details>
                  <summary>Solver details</summary>
                  <p>Algorithm: SciPy PRIMA COBYLA · no gradients</p>
                  <p>
                    Variables: 3 lateral-offset + 3 speed-scale nodes. Fixed
                    start node.
                  </p>
                  <p>
                    Each uncached evaluation simulates a full candidate lap (or
                    stops on failure).
                  </p>
                  <p>
                    Constraints: clearance, completion, mechanical closure, time
                    ceiling, variable bounds.
                  </p>
                  <p>
                    Workers: {String(optimization.worker_count ?? 1)} ·
                    sequential adaptive trust region
                  </p>
                  <p>
                    Function evaluations:{" "}
                    {String(optimization.evaluations ?? 0)} · cache hits:{" "}
                    {String(optimization.cache_hits ?? 0)}
                  </p>
                  <p>
                    Best objective: {String(optimization.best_objective ?? "—")}{" "}
                    · improvement: {String(optimization.improvement ?? "—")}
                  </p>
                  <p>
                    Constraint violation:{" "}
                    {String(optimization.constraint_violation ?? "—")}
                  </p>
                  <p>
                    Elapsed: {fmt(Number(optimization.wall_s ?? 0))} s · mean
                    evaluation:{" "}
                    {fmt(Number(optimization.mean_evaluation_s ?? 0), 3)} s
                  </p>
                  <small>
                    Convergence ETA unavailable. Evaluations are not iterations.
                  </small>
                </details>
                <p role="status">
                  {String(optimization.status ?? "Ready")}{" "}
                  {optimization.best_objective != null
                    ? `· Current best ${Number(optimization.best_objective).toFixed(5)}`
                    : ""}
                </p>
                <small>Only replay-validated results can become ghosts.</small>
              </>
            ) : (
              <>
                <small>
                  W / ↑ throttle · S / ↓ brake
                  <br />A D / ← → steering · Gamepad: stick + triggers
                </small>
                <details>
                  <summary>Keyboard command mapping</summary>
                  <p>
                    Full-key angle = min(max angle, atan(5 × wheelbase /
                    speed²)). 5 m/s² is an input precision estimate; no yaw
                    correction. Analog bypasses this map.
                  </p>
                  {Object.entries(shaping).map(([name, value]) => (
                    <label key={name}>
                      {name} (/s)
                      <input
                        aria-label={name}
                        type="number"
                        min={0.1}
                        max={10}
                        step={0.1}
                        value={value}
                        onChange={(e) => {
                          const v = +e.target.value;
                          if (v >= 0.1 && v <= 10)
                            setShaping((old) => ({ ...old, [name]: v }));
                        }}
                      />
                    </label>
                  ))}
                </details>
                <label>
                  Steering rate{" "}
                  <input
                    aria-label="Steering rate"
                    type="range"
                    min={0.1}
                    max={5}
                    step={0.1}
                    value={steeringRate}
                    onChange={(e) => setSteeringRate(+e.target.value)}
                  />
                  <b>{steeringRate} rad/s</b>
                </label>
                <label>
                  Steering saturation{" "}
                  <input
                    aria-label="Steering saturation"
                    type="range"
                    min={0.05}
                    max={0.65}
                    step={0.01}
                    value={saturation}
                    onChange={(e) => setSaturation(+e.target.value)}
                  />
                </label>
                <label>
                  Gamepad deadzone{" "}
                  <input
                    aria-label="Gamepad deadzone"
                    type="range"
                    min={0}
                    max={0.3}
                    step={0.01}
                    value={deadzone}
                    onChange={(e) => setDeadzone(+e.target.value)}
                  />
                </label>
                <label>
                  Hybrid deployment{" "}
                  <input
                    aria-label="Hybrid deployment"
                    type="range"
                    min={0}
                    max={1}
                    step={0.05}
                    value={deployment}
                    onChange={(e) => setDeployment(+e.target.value)}
                  />
                </label>
                {vehicle === "mcl36" ? (
                  <label>
                    <input
                      type="checkbox"
                      checked={drs}
                      onChange={(e) => setDrs(e.target.checked)}
                    />
                    DRS request · closes under braking
                  </label>
                ) : (
                  <label>
                    Active aero
                    <select
                      aria-label="Active aero"
                      value={aeroMode}
                      onChange={(e) => setAeroMode(+e.target.value)}
                    >
                      <option value={0}>Road</option>
                      <option value={1}>Race / high downforce</option>
                    </select>
                  </label>
                )}
              </>
            )}
          </section>
        </div>
      </main>
    </Profiler>
  );
}
function Trace({
  name,
  history,
  channel,
  comparison,
  cursor,
  onSeek,
}: {
  name: string;
  history: DriveSample[];
  channel: number;
  comparison?: DriveSample[];
  cursor?: number;
  onSeek: (s: number) => void;
}) {
  const spatial = cursor !== undefined;
  const [points, comparisonPoints] = useMemo(
    () =>
      measure("charts", () => {
        const scale = [360, 1, 1, 0.65][channel];
        const build = (rows?: DriveSample[]) =>
          rows
            ?.filter((_, i) => i % 3 === 0)
            .map((row, i, all) => {
              const x = spatial
                ? ((row.s - (history[0]?.s ?? 0)) /
                    Math.max(
                      0.001,
                      (history.at(-1)?.s ?? 1) - (history[0]?.s ?? 0),
                    )) *
                  500
                : (i / Math.max(1, all.length - 1)) * 500;
              const value = [
                Math.hypot(...row.velocity) * 3.6,
                row.throttle,
                row.brake,
                row.steering,
              ][channel];
              return `${x},${channel === 3 ? 21 - (value / scale) * 19 : 40 - (value / scale) * 38}`;
            })
            .join(" ");
        return [build(history), build(comparison)];
      }),
    [history, comparison, channel, spatial],
  );
  const row = measure("telemetry selectors", () =>
    cursor === undefined ? history.at(-1) : ghostAt(history, cursor, "time"),
  );
  const cursorS = row?.s ?? 0;
  return (
    <div className="drive-trace">
      <small>{name}</small>
      <svg
        viewBox="0 0 500 42"
        preserveAspectRatio="none"
        onClick={(e) => {
          if (cursor === undefined || !history.length) return;
          const rect = e.currentTarget.getBoundingClientRect();
          onSeek(
            history[0].s +
              Math.max(0, Math.min(1, (e.clientX - rect.left) / rect.width)) *
                (history.at(-1)!.s - history[0].s),
          );
        }}
      >
        {comparisonPoints && (
          <polyline
            stroke="#a9b5bc"
            strokeDasharray="4 3"
            points={comparisonPoints}
          />
        )}{" "}
        {cursor !== undefined && (
          <line
            stroke="#fff"
            strokeDasharray="3 2"
            x1={
              ((cursorS - (history[0]?.s ?? 0)) /
                ((history.at(-1)?.s ?? 1) - (history[0]?.s ?? 0))) *
              500
            }
            x2={
              ((cursorS - (history[0]?.s ?? 0)) /
                ((history.at(-1)?.s ?? 1) - (history[0]?.s ?? 0))) *
              500
            }
            y1={0}
            y2={42}
          />
        )}
        <path
          className="drive-grid"
          d="M0 10H500M0 30H500M100 0V42M200 0V42M300 0V42M400 0V42"
        />
        <polyline
          stroke={["#60b4f3", "#50d894", "#f45a51", "#edc961"][channel]}
          points={points}
        />
      </svg>
      <b>
        {fmt(
          (() => {
            const row =
              cursor === undefined
                ? history.at(-1)
                : ghostAt(history, cursor, "time");
            return row
              ? [
                  Math.hypot(...row.velocity) * 3.6,
                  row.throttle,
                  row.brake,
                  row.steering,
                ][channel]
              : 0;
          })(),
          channel ? 2 : 0,
        )}
      </b>
    </div>
  );
}
function Map({
  meta,
  sample,
  ghost,
  referenceLap,
  onSeek,
}: {
  meta: DriveMetadata;
  sample?: DriveSample;
  ghost?: Ghost;
  referenceLap?: Ghost;
  onSeek: (s: number) => void;
}) {
  const { xs, ys, x, y, w, h, trackPoints, referencePoints, policyPoints } =
    useMemo(
      () =>
        measure("telemetry selectors", () => {
          const xs = meta.geometry.map((p) => p.x_m),
            ys = meta.geometry.map((p) => p.y_m),
            x = Math.min(...xs),
            y = Math.min(...ys);
          return {
            xs,
            ys,
            x,
            y,
            w: Math.max(...xs) - x,
            h: Math.max(...ys) - y,
            trackPoints: meta.geometry
              .map((p) => `${p.x_m},${-p.y_m}`)
              .join(" "),
            referencePoints: referenceLap?.samples
              .map((p) => `${p.position[0]},${-p.position[1]}`)
              .join(" "),
            policyPoints: ghost?.samples
              .map((p) => `${p.position[0]},${-p.position[1]}`)
              .join(" "),
          };
        }),
      [meta, ghost, referenceLap],
    );
  return (
    <svg
      viewBox={`${x - 80} ${-y - h - 80} ${w + 160} ${h + 160}`}
      onClick={(e) => {
        const matrix = e.currentTarget.getScreenCTM();
        if (!matrix) return;
        const point = new DOMPoint(e.clientX, e.clientY).matrixTransform(
          matrix.inverse(),
        );
        const rows = ghost?.samples ?? referenceLap?.samples;
        if (!rows?.length) return;
        const nearest = rows.reduce((best, row) =>
          Math.hypot(row.position[0] - point.x, row.position[1] + point.y) <
          Math.hypot(best.position[0] - point.x, best.position[1] + point.y)
            ? row
            : best,
        );
        onSeek(nearest.s);
      }}
    >
      <polyline
        fill="none"
        stroke="#687b88"
        strokeWidth={14}
        points={trackPoints}
      />
      {referenceLap && (
        <polyline
          fill="none"
          stroke="#a9b5bc"
          strokeWidth={8}
          strokeDasharray="25 18"
          points={referencePoints}
        />
      )}
      {meta.sectors.map((f) => {
        const p =
          meta.geometry[
            Math.min(
              meta.geometry.length - 1,
              Math.round(f * (meta.geometry.length - 1)),
            )
          ];
        return <circle key={f} cx={p.x_m} cy={-p.y_m} r={22} fill="#d8b569" />;
      })}
      {ghost && (
        <polyline
          fill="none"
          stroke="#59dda0"
          strokeWidth={8}
          points={policyPoints}
        />
      )}
      <circle fill="#eee" cx={xs[0]} cy={-ys[0]} r={22} />
      {sample && (
        <circle
          fill="#55baff"
          cx={sample.position[0]}
          cy={-sample.position[1]}
          r={35}
        />
      )}
    </svg>
  );
}

function TireHistory({
  rows,
  sample,
  wheel,
}: {
  rows: DriveSample[];
  sample: DriveSample;
  wheel: number;
}) {
  const tire = sample.wheels[wheel],
    force = Math.hypot(tire.fx, tire.fy),
    scale = force > 0 ? tire.utilization / force : 0;
  const visible = rows
    .filter((r, i) => r.time <= sample.time && i % 8 === 0)
    .slice(-160);
  const channels = [
    {
      label: "Tread °C",
      color: "#ffb15a",
      value: (r: DriveSample) => r.wheels[wheel].tread,
    },
    {
      label: "Wear %",
      color: "#e7c5ff",
      value: (r: DriveSample) => 100 * r.wheels[wheel].wear,
    },
    {
      label: "Fy kN",
      color: "#60b4f3",
      value: (r: DriveSample) => r.wheels[wheel].fy / 1000,
    },
  ];
  return (
    <div className="drive-tire-history">
      <svg viewBox="0 0 100 100" aria-label="Actual tire friction utilization">
        <circle cx={50} cy={50} r={40} fill="none" stroke="#6b8493" />
        <path d="M10 50H90M50 10V90" stroke="#36515f" />
        <circle
          cx={50 + 40 * tire.fy * scale}
          cy={50 - 40 * tire.fx * scale}
          r={4}
          fill={tire.utilization > 0.98 ? "#f45a51" : "#50d894"}
        />
        <text x={50} y={98} textAnchor="middle" fill="#abc0cc" fontSize={10}>
          {fmt(tire.utilization * 100, 0)}% · Fy / Fx
        </text>
      </svg>
      <div>
        {channels.map((c) => {
          const values = visible.map(c.value),
            lo = Math.min(...values, 0),
            hi = Math.max(...values, lo + 0.001);
          return (
            <div key={c.label}>
              <small>
                {c.label} · {fmt(c.value(sample), 2)}
              </small>
              <svg viewBox="0 0 180 22" preserveAspectRatio="none">
                <polyline
                  fill="none"
                  stroke={c.color}
                  points={values
                    .map(
                      (v, i) =>
                        `${(i / Math.max(1, values.length - 1)) * 180},${21 - ((v - lo) / (hi - lo)) * 20}`,
                    )
                    .join(" ")}
                />
              </svg>
            </div>
          );
        })}
      </div>
    </div>
  );
}
