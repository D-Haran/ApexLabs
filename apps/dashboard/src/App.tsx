import {
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
  Component,
  type ReactNode,
} from "react";
import {
  Activity,
  ArrowUpRight,
  ChevronRight,
  FolderOpen,
  Layers,
  ScanLine,
  Settings2,
} from "lucide-react";
import type { Session, Lap } from "./telemetry/contract";
import { loadSession } from "./telemetry/client";
import { replay } from "./state/replay";
import {
  events,
  optimizationEvents,
  summary,
  lapRows,
  compatible,
} from "./analysis/metrics";
import { fmt, kmh, g, lapTime } from "./data/units";
import {
  Scene,
  defaultOverlays,
  type CameraMode,
  type Overlays,
} from "./three/Scene";
import { recordAssetReady } from "./state/performance";
import { parseCarManifest } from "./three/carManifest";
import { ChassisReadout } from "./components/ChassisReadout";
import type { AssetAdapter } from "./three/Vehicle";
import { TrackMap } from "./components/TrackMap";
import { TirePanel } from "./components/TirePanel";
import { Telemetry } from "./components/Telemetry";
import { ReplayControls, LiveReadout, Inspector } from "./components/Controls";
import { RegionAnalysis, ComparisonReadout } from "./components/Analysis";
import { Performance } from "./components/Performance";
import { AeroLoadPanel, GForcePanel } from "./components/DynamicsPanels";
import { OptimizationPanel } from "./components/OptimizationPanel";
class SceneBoundary extends Component<
  { children: ReactNode },
  { error: string | null }
> {
  state = { error: null };
  static getDerivedStateFromError(e: Error) {
    return { error: e.message };
  }
  render() {
    return this.state.error ? (
      <div className="scene-error">
        3D view unavailable: {this.state.error}. Telemetry remains available.
      </div>
    ) : (
      this.props.children
    );
  }
}
export default function App() {
  const [session, setSession] = useState<Session | null>(null),
    [lap, setLap] = useState<Lap | null>(null),
    [comparison, setComparison] = useState<Session | null>(null),
    [comparisonLap, setComparisonLap] = useState<Lap | null>(null),
    [loading, setLoading] = useState("Loading recorded engineering session"),
    [error, setError] = useState("");
  const [resultMode, setResultMode] = useState<
    "Reference" | "Optimized" | "Comparison"
  >("Reference");
  const [mode, setMode] = useState<CameraMode>("Engineering"),
    [overlays, setOverlays] = useState(defaultOverlays),
    [showOverlays, setShowOverlays] = useState(false),
    [showSettings, setShowSettings] = useState(false),
    [showAllEvents, setShowAllEvents] = useState(false),
    [asset, setAsset] = useState<AssetAdapter>();
  const [assetReady, setAssetReady] = useState(false);
  const [visualModel, setVisualModel] = useState("mclaren-p1");
  const files = useRef<HTMLInputElement>(null),
    loadTarget = useRef<"A" | "B">("A"),
    generation = useRef(0);
  const open = useCallback(
    async (input: string | File[], target: "A" | "B") => {
      const id = ++generation.current;
      setLoading(`Loading lap ${target}`);
      setError("");
      replay.pause();
      try {
        const loaded = await loadSession(input);
        if (id !== generation.current) return;
        if (target === "A") {
          setSession(loaded);
          const l = loaded.manifest.laps[0];
          setLap(l);
          replay.load(loaded, l);
          setComparison(null);
          setComparisonLap(null);
          setResultMode(
            loaded.manifest.optimization ? "Optimized" : "Reference",
          );
          const peak = events(loaded, l)[0];
          if (loaded.manifest.session_kind === "validation_scene")
            replay.seek(2.2);
          else if (peak) replay.seek(peak.time);
        } else {
          if (!session) throw new Error("Load Lap A first");
          compatible(session, loaded);
          setComparison(loaded);
          setComparisonLap(loaded.manifest.laps[0]);
          setResultMode("Comparison");
        }
      } catch (e) {
        if (id === generation.current) setError((e as Error).message);
      } finally {
        if (id === generation.current) setLoading("");
      }
    },
    [session],
  );
  const openPair = useCallback(async () => {
    const id = ++generation.current;
    setLoading("Loading reference and optimized laps");
    setError("");
    replay.pause();
    try {
      const [reference, optimized] = await Promise.all([
        loadSession("/demo/baseline/session.json"),
        loadSession("/demo/technical-optimized/session.json"),
      ]);
      if (id !== generation.current) return;
      compatible(reference, optimized);
      const referenceLap = reference.manifest.laps[0];
      setSession(reference);
      setLap(referenceLap);
      setComparison(optimized);
      setComparisonLap(optimized.manifest.laps[0]);
      setResultMode("Comparison");
      replay.load(reference, referenceLap);
      const peak = events(reference, referenceLap)[0];
      if (peak) replay.seek(peak.time);
    } catch (e) {
      if (id === generation.current) setError((e as Error).message);
    } finally {
      if (id === generation.current) setLoading("");
    }
  }, []);
  useEffect(() => {
    const scene = new URLSearchParams(location.search).get("scene");
    const scenes = [
      "flat",
      "uphill",
      "downhill",
      "left",
      "right",
      "braking",
      "acceleration",
    ];
    void open(
      scene && scenes.includes(scene)
        ? `/demo/chassis-scenes/${scene}/session.json`
        : "/demo/spa-p1-sprung/session.json",
      "A",
    );
  }, []);
  useEffect(() => {
    let cancelled = false;
    setAssetReady(false);
    const assetStart = performance.now();
    if (visualModel === "placeholder") {
      setAsset(undefined);
      return;
    }
    void fetch(`/assets/${visualModel}.json`)
      .then((r) => {
        if (!r.ok) throw new Error("Visual manifest unavailable");
        return r.json();
      })
      .then((data) => {
        const manifest = parseCarManifest(data);
        if (!cancelled)
          setAsset({
            url: `/assets/${visualModel}.glb`,
            manifest,
            scale: 1,
            rotation: [0, 0, 0],
            offset: [0, 0, 0],
            onLoaded: () => {
              setAssetReady(true);
              recordAssetReady(
                manifest.assetName,
                performance.now() - assetStart,
                manifest.textureMemoryEstimateBytes,
              );
            },
            onError: (message) => {
              setAssetReady(false);
              setError(`Vehicle asset failed: ${message}`);
            },
          });
      })
      .catch((e) => {
        if (!cancelled) setError(`Vehicle visual: ${e.message}`);
      });
    return () => {
      cancelled = true;
    };
  }, [visualModel]);
  useEffect(() => {
    const hidden = () => {
      if (document.hidden) replay.pause();
    };
    document.addEventListener("visibilitychange", hidden);
    const key = (e: KeyboardEvent) => {
      if ((e.target as HTMLElement).matches("input,select,textarea,button"))
        return;
      if (e.code === "Space") {
        e.preventDefault();
        replay.getSnapshot().playing ? replay.pause() : replay.play();
      }
      if (e.code === "ArrowRight") replay.step(1);
      if (e.code === "ArrowLeft") replay.step(-1);
    };
    window.addEventListener("keydown", key);
    return () => {
      document.removeEventListener("visibilitychange", hidden);
      window.removeEventListener("keydown", key);
    };
  }, []);
  const detected = useMemo(() => {
    if (!session || !lap) return [];
    return resultMode === "Comparison" && comparison && comparisonLap
      ? optimizationEvents(session, lap, comparison, comparisonLap)
      : events(session, lap);
  }, [session, lap, comparison, comparisonLap, resultMode]);
  const stats = useMemo(
    () => (session && lap ? summary(lapRows(session, lap)) : null),
    [session, lap],
  );
  const choose = (target: "A" | "B") => {
    loadTarget.current = target;
    files.current?.click();
  };
  const selectResult = (next: typeof resultMode) => {
    if (!session || !lap) return;
    const sessionIsOptimized = Boolean(session.manifest.optimization);
    const comparisonIsOptimized = Boolean(comparison?.manifest.optimization);
    const needsSwap =
      (next === "Optimized" && !sessionIsOptimized && comparisonIsOptimized) ||
      (next !== "Optimized" &&
        sessionIsOptimized &&
        comparison &&
        !comparisonIsOptimized);
    if (needsSwap && comparison && comparisonLap) {
      const previousSession = session;
      const previousLap = lap;
      setSession(comparison);
      setLap(comparisonLap);
      setComparison(previousSession);
      setComparisonLap(previousLap);
      replay.load(comparison, comparisonLap);
    } else if (next !== "Comparison") {
      replay.load(session, lap);
    }
    setResultMode(next);
  };
  return (
    <div className="app">
      <header className="app-header">
        <a className="brand" href="/">
          <Activity size={26} strokeWidth={1.7} />
          <span>
            Apex<span>Lab</span>
          </span>
        </a>
        <nav className="mode-nav" aria-label="Workspace modes">
          <button className="active">DRIVE</button>
          <button
            onClick={() =>
              document
                .querySelector(".telemetry-panel")
                ?.scrollIntoView({ behavior: "smooth" })
            }
          >
            ANALYZE
          </button>
          <button
            onClick={() =>
              document
                .querySelector(".region-panel")
                ?.scrollIntoView({ behavior: "smooth" })
            }
          >
            IMPROVE
          </button>
        </nav>
        <div className="header-track">
          <span>CIRCUIT</span>
          <select
            aria-label="Current circuit"
            value={session?.manifest.track.name ?? ""}
            disabled={!!loading || !session}
            onChange={(event) => {
              if (event.target.value === "ApexLab Technical Test Circuit")
                void openPair();
              else {
                const sources: Record<string, string> = {
                  "Spa-Francorchamps": "/demo/spa-p1-sprung/session.json",
                  Monza: "/demo/monza-p1/session.json",
                  Silverstone: "/demo/silverstone-p1/session.json",
                  Suzuka: "/demo/suzuka-p1/session.json",
                };
                if (sources[event.target.value])
                  void open(sources[event.target.value], "A");
              }
            }}
          >
            {session?.manifest.session_kind === "validation_scene" && (
              <option>{session.manifest.track.name}</option>
            )}
            <option>Spa-Francorchamps</option>
            <option>Monza</option>
            <option>Silverstone</option>
            <option>Suzuka</option>
            <option value="ApexLab Technical Test Circuit">
              Technical Test Circuit
            </option>
          </select>
        </div>
        <button
          className="open-session"
          title="Select session.json and its four companion CSV files together"
          onClick={() => choose("A")}
        >
          <FolderOpen size={15} /> Open
        </button>
        <button
          className="settings-button"
          title="Workspace settings"
          aria-label="Workspace settings"
          onClick={() => setShowSettings((value) => !value)}
        >
          <Settings2 size={16} />
        </button>
        {showSettings && (
          <div className="settings-popover">
            <b>WORKSPACE 06</b>
            <span>SI physics · engineering display units</span>
            <span>Recorded deterministic replay</span>
          </div>
        )}
      </header>
      <input
        ref={files}
        type="file"
        multiple
        accept=".json,.csv"
        hidden
        onChange={(e) => {
          if (e.target.files?.length)
            void open(Array.from(e.target.files), loadTarget.current);
          e.target.value = "";
        }}
      />
      {error && (
        <div className="error" role="alert">
          {error}
          <button onClick={() => setError("")}>Dismiss</button>
        </div>
      )}
      {loading && (
        <div className="loading" role="status">
          {loading}…
        </div>
      )}
      {session && lap && stats ? (
        <>
          <div className="workspace-title">
            <nav className="product-nav" aria-label="Engineering views">
              {[
                ["Track", ".map-panel"],
                ["Car Setup", ".asset-panel"],
                ["Telemetry", ".telemetry-panel"],
                ["Analysis", ".region-panel"],
                ["Compare", ".analysis-strip"],
              ].map(([label, selector]) => (
                <button
                  key={label}
                  onClick={() =>
                    document
                      .querySelector(selector)
                      ?.scrollIntoView({ behavior: "smooth" })
                  }
                >
                  {label}
                </button>
              ))}
            </nav>
            <div>
              <span className="eyebrow">
                RECORDED SESSION /{" "}
                {session.manifest.track.kind === "real_imported"
                  ? "REAL IMPORTED CIRCUIT"
                  : "SYNTHETIC VALIDATION CIRCUIT"}
              </span>
              <h1>{session.manifest.track.name.replace("ApexLab ", "")}</h1>
            </div>
            <div className="session-facts">
              <span>
                <b>{fmt(session.manifest.track.length_m, 1)}</b> m circuit
              </span>
              <span>
                <b>{fmt(session.manifest.simulation.timestep_s * 1000, 0)}</b>{" "}
                ms physics
              </span>
              <span className="verified">
                <span className="status-dot playing" /> Simulator telemetry
              </span>
            </div>
          </div>
          <main className="workspace">
            <aside className="session-sidebar">
              <div className="panel-heading">
                <span>SESSION EXPLORER</span>
                <ScanLine size={14} />
              </div>
              <div className="session-selected">
                <span className="lap-tag">A</span>
                <div>
                  <b>{session.manifest.session_name}</b>
                  <span>{session.manifest.vehicle.name}</span>
                </div>
              </div>
              <label className="visual-selector">
                Visual model
                <select
                  aria-label="Visual model"
                  value={visualModel}
                  onChange={(e) => setVisualModel(e.target.value)}
                >
                  <option value="mclaren-p1">McLaren P1</option>
                  <option value="mclaren-f1-2022">McLaren F1 2022</option>
                  <option value="placeholder">Engineering placeholder</option>
                </select>
                <small>
                  Physics:{" "}
                  {session.manifest.vehicle.normal_load_model === "sprung_body"
                    ? "passive sprung-body approximation"
                    : "quasi-static planar"}
                </small>
              </label>
              <div className="result-tabs" aria-label="Lap result mode">
                <button
                  className={resultMode === "Reference" ? "active" : ""}
                  onClick={() => selectResult("Reference")}
                  disabled={
                    Boolean(session.manifest.optimization) &&
                    (!comparison || Boolean(comparison.manifest.optimization))
                  }
                >
                  Reference
                </button>
                <button
                  className={resultMode === "Optimized" ? "active" : ""}
                  disabled={
                    !session.manifest.optimization &&
                    !comparison?.manifest.optimization
                  }
                  title="Validated locally optimized racing-line replay"
                  onClick={() => selectResult("Optimized")}
                >
                  Optimized
                </button>
                <button
                  className={resultMode === "Comparison" ? "active" : ""}
                  disabled={!comparison}
                  onClick={() => selectResult("Comparison")}
                >
                  Compare
                </button>
              </div>
              <div className="lap-selector">
                <label>
                  Recorded lap{" "}
                  <select
                    aria-label="Recorded lap"
                    value={lap.number}
                    onChange={(e) => {
                      const l = session.manifest.laps.find(
                        (v) => v.number === Number(e.target.value),
                      )!;
                      setLap(l);
                      replay.load(session, l);
                    }}
                  >
                    {session.manifest.laps.map((l) => (
                      <option key={l.number} value={l.number}>
                        {l.number + 1} · timed
                      </option>
                    ))}
                  </select>
                </label>
              </div>
              <div className="lap-result">
                <span className="eyebrow">
                  {session.manifest.session_kind === "validation_scene"
                    ? "VALIDATION INTERVAL · NOT A LAP"
                    : "SIMULATOR LAP TIME"}
                </span>
                <strong>{lapTime(lap.lap_time_s)}</strong>
                <span>
                  {session.manifest.simulation.warmup_laps} settling lap · RK4
                </span>
              </div>
              <div className="sectors">
                {lap.sector_times_s.map((t, i) => (
                  <div key={i}>
                    <span>S{i + 1}</span>
                    <b>{fmt(t, 3)}</b>
                    <small>s</small>
                  </div>
                ))}
              </div>
              <dl className="summary-list">
                <div>
                  <dt>Mean speed</dt>
                  <dd>
                    {fmt(kmh(stats.mean))} <small>km/h</small>
                  </dd>
                </div>
                <div>
                  <dt>Speed range</dt>
                  <dd>
                    {fmt(kmh(stats.min), 0)}–{fmt(kmh(stats.max), 0)}{" "}
                    <small>km/h</small>
                  </dd>
                </div>
                <div>
                  <dt>Peak lateral</dt>
                  <dd>
                    {fmt(g(stats.lat), 2)} <small>g</small>
                  </dd>
                </div>
                <div>
                  <dt>Peak braking</dt>
                  <dd>
                    {fmt(g(stats.braking), 2)} <small>g</small>
                  </dd>
                </div>
                <div>
                  <dt>Peak acceleration</dt>
                  <dd>
                    {fmt(g(stats.accel), 2)} <small>g</small>
                  </dd>
                </div>
                <div>
                  <dt>Reference μ</dt>
                  <dd>{fmt(session.manifest.vehicle.mu_reference, 2)}</dd>
                </div>
              </dl>
              <div className="event-heading">
                <span className="eyebrow">LAP EVENTS</span>
                <span>{detected.length}</span>
              </div>
              <div className="event-list">
                {detected
                  .slice(0, showAllEvents ? undefined : 7)
                  .map((e, i) => (
                    <button
                      key={`${e.label}-${e.time}`}
                      className={i === 0 ? "peak-event" : ""}
                      onClick={() => {
                        replay.pause();
                        replay.seek(e.time);
                      }}
                    >
                      <div>
                        <span>{e.label}</span>
                        <ChevronRight size={12} />
                      </div>
                      <small>
                        {e.wheel && `${e.wheel} · `}
                        {fmt(e.s, 1)} m{" "}
                        <span>{fmt(e.time - lap.start_time_s, 3)} s</span>
                      </small>
                    </button>
                  ))}
              </div>
              {detected.length > 7 && (
                <button
                  className="text-button"
                  onClick={() => setShowAllEvents(!showAllEvents)}
                >
                  {showAllEvents
                    ? "Show extrema only"
                    : `Show ${detected.length - 7} saturation onsets`}
                </button>
              )}
            </aside>
            <section className="center-panel">
              <div className="viewport-toolbar">
                <div className="camera-tabs">
                  {(
                    [
                      "Chase",
                      "Orbit",
                      "Overview",
                      "Engineering",
                    ] as CameraMode[]
                  ).map((c) => (
                    <button
                      key={c}
                      className={mode === c ? "active" : ""}
                      onClick={() => setMode(c)}
                    >
                      {c}
                    </button>
                  ))}
                </div>
                <button
                  className={showOverlays ? "active" : ""}
                  onClick={() => setShowOverlays(!showOverlays)}
                >
                  <Layers size={14} /> Overlays
                </button>
              </div>
              <div className="viewport">
                <SceneBoundary>
                  <Scene
                    session={session}
                    comparison={resultMode === "Comparison" ? comparison : null}
                    mode={mode}
                    overlays={overlays}
                    asset={asset}
                  />
                </SceneBoundary>
                <LiveReadout />
                {showOverlays && (
                  <div className="overlay-menu">
                    {(Object.keys(overlays) as (keyof Overlays)[]).map((k) => (
                      <label key={k}>
                        <input
                          type="checkbox"
                          checked={overlays[k]}
                          onChange={() =>
                            setOverlays((v) => ({ ...v, [k]: !v[k] }))
                          }
                        />
                        {
                          {
                            forces: "Tire forces",
                            loads: "Normal loads",
                            axes: "Road/body axes + wheel contacts + CG",
                            velocity: "Velocity vector",
                            reference: "Reference line",
                            boundaries: "Track boundaries",
                            trajectory: "Trajectory history",
                          }[k]
                        }
                      </label>
                    ))}
                  </div>
                )}
                <div className="scene-caption">
                  <span>
                    {session.manifest.vehicle.normal_load_model ===
                    "sprung_body"
                      ? "GRADE + PASSIVE SPRUNG BODY · ESTIMATED SUSPENSION"
                      : "PLANAR FOUR-TIRE PHYSICS · ROAD-CONFORMING VISUAL"}
                  </span>
                  <span>
                    {asset
                      ? `${asset.manifest?.assetName ?? "Imported vehicle"} visual asset`
                      : "Procedural vehicle fallback"}{" "}
                    ·{" "}
                    {assetReady
                      ? "Mesh ready"
                      : asset
                        ? "Loading mesh"
                        : "Placeholder"}
                  </span>
                </div>
              </div>
              {overlays.axes && <ChassisReadout />}
              <div className="overlay-legend">
                <span>
                  <i className="force-dot" /> Force: 1 m = 1.5 kN
                </span>
                <span>
                  <i className="load-dot" /> Load: 1 m = 2.5 kN
                </span>
                <span>Velocity: 1 m = 5 m/s</span>
                <span className="legend-spacer" />
                <span>World X / Y · SI</span>
              </div>
              <ReplayControls session={session} lap={lap} />
              <div className="analysis-strip">
                <div>
                  <span className="eyebrow">SPATIAL COMPARISON</span>
                  <p>Locate the gain. Inspect the cause.</p>
                </div>
                <button
                  onClick={() => void open("/demo/high-grip/session.json", "B")}
                  disabled={!!loading}
                >
                  Compare higher grip <ArrowUpRight size={14} />
                </button>
                <button onClick={() => choose("B")}>Load lap B</button>
              </div>
              {resultMode === "Comparison" && comparison && comparisonLap && (
                <>
                  <div className="comparison-title">
                    <span className="lap-tag b">B</span>
                    <b>{comparison.manifest.session_name}</b>
                    <select
                      aria-label="Comparison lap"
                      value={comparisonLap.number}
                      onChange={(e) =>
                        setComparisonLap(
                          comparison.manifest.laps.find(
                            (l) => l.number === Number(e.target.value),
                          )!,
                        )
                      }
                    >
                      {comparison.manifest.laps.map((l) => (
                        <option key={l.number} value={l.number}>
                          Lap {l.number + 1} · {lapTime(l.lap_time_s)}
                        </option>
                      ))}
                    </select>
                    <button
                      onClick={() => {
                        setComparison(null);
                        setComparisonLap(null);
                      }}
                    >
                      Remove
                    </button>
                  </div>
                  <ComparisonReadout
                    a={session}
                    al={lap}
                    b={comparison}
                    bl={comparisonLap}
                  />
                </>
              )}
            </section>
            <aside className="engineering-sidebar">
              <TrackMap
                session={session}
                lap={lap}
                comparison={resultMode === "Comparison" ? comparison : null}
                comparisonLap={
                  resultMode === "Comparison" ? comparisonLap : null
                }
              />
              <TirePanel />
              <AeroLoadPanel session={session} />
              <GForcePanel />
            </aside>
          </main>
          <div className="lower-workspace">
            <Telemetry
              session={session}
              lap={lap}
              comparison={resultMode === "Comparison" ? comparison : null}
              comparisonLap={resultMode === "Comparison" ? comparisonLap : null}
            />
            <OptimizationPanel
              session={session}
              hasOptimized={
                Boolean(session.manifest.optimization) ||
                Boolean(comparison?.manifest.optimization)
              }
              onReference={() => selectResult("Reference")}
              onOptimized={() => selectResult("Optimized")}
              onCompare={() => selectResult("Comparison")}
            />
          </div>
          <RegionAnalysis
            key={`${session.manifest.session_name}-${lap.number}`}
            session={session}
            lap={lap}
            b={resultMode === "Comparison" ? comparison : null}
            bl={resultMode === "Comparison" ? comparisonLap : null}
          />
          <Inspector />
          <details className="asset-panel">
            <summary>Data provenance &amp; vehicle asset adapter</summary>
            <dl className="provenance-list">
              <div>
                <dt>Geometry</dt>
                <dd>{session.manifest.track.provenance}</dd>
              </div>
              <div>
                <dt>Elevation</dt>
                <dd>
                  {session.manifest.track.elevation_source ??
                    "Flat mathematical fixture"}
                </dd>
              </div>
              <div>
                <dt>Vehicle parameters</dt>
                <dd>{session.manifest.vehicle.provenance}</dd>
              </div>
              <div>
                <dt>Visual asset</dt>
                <dd>
                  {asset
                    ? `${asset.manifest?.assetName ?? "Imported"} · ${asset.manifest?.source.author ?? "User supplied"} · ${asset.manifest?.source.license ?? "See source"}`
                    : "Repository-authored procedural placeholder"}
                </dd>
              </div>
              <div>
                <dt>Model class</dt>
                <dd>
                  {session.manifest.optimization?.kind ??
                    "Feasible controller-driven reference lap"}
                </dd>
              </div>
            </dl>
            <p>
              Visual selection does not change the physical parameter model
              shown in the session panel. The supplied P1 and F1 meshes are CC
              BY 4.0; credits above and in assets/cars/README.md. Load a
              permitted, self-contained GLB or GLTF with embedded resources.
              Local +X forward, +Y up, −Z left; origin at ground below CG.
              Visual dimensions do not change physics.
            </p>
            <p>
              Physical validation scenes:{" "}
              {[
                "flat",
                "uphill",
                "downhill",
                "left",
                "right",
                "braking",
                "acceleration",
              ].map((name) => (
                <a
                  key={name}
                  href={`/?scene=${name}`}
                  style={{ marginRight: 12 }}
                >
                  {name}
                </a>
              ))}
            </p>
            <input
              aria-label="Vehicle GLB or GLTF"
              type="file"
              accept=".glb,.gltf"
              onChange={(e) => {
                const f = e.target.files?.[0];
                if (f) {
                  if (asset) URL.revokeObjectURL(asset.url);
                  setAsset({
                    url: URL.createObjectURL(f),
                    scale: 1,
                    rotation: [0, 0, 0],
                    offset: [0, 0, 0],
                  });
                }
              }}
            />
            {asset && !asset.manifest && (
              <>
                <label>
                  Scale{" "}
                  <input
                    type="number"
                    min="0.001"
                    step="0.1"
                    value={asset.scale}
                    onChange={(e) =>
                      setAsset({
                        ...asset,
                        scale: Math.max(0.001, Number(e.target.value)),
                      })
                    }
                  />
                </label>
                <label>
                  Yaw correction °{" "}
                  <input
                    type="number"
                    value={(asset.rotation[1] * 180) / Math.PI}
                    onChange={(e) =>
                      setAsset({
                        ...asset,
                        rotation: [
                          0,
                          (Number(e.target.value) * Math.PI) / 180,
                          0,
                        ],
                      })
                    }
                  />
                </label>
                <button
                  onClick={() => {
                    URL.revokeObjectURL(asset.url);
                    setAsset(undefined);
                  }}
                >
                  Use placeholder
                </button>
              </>
            )}
          </details>
          <Performance />
          <footer>
            <span>
              ApexLab <b>/</b> Vehicle Dynamics · Lap Simulation · Telemetry
              Analysis
            </span>
            <span>Recorded data · No live physics · SI internally</span>
          </footer>
        </>
      ) : (
        !loading && (
          <div className="empty-state">
            <h1>Load an ApexLab session</h1>
            <p>
              Select session.json together with its telemetry, wheel, geometry
              and spatial CSV files.
            </p>
            <button
              onClick={() => void open("/demo/baseline/session.json", "A")}
            >
              Load demonstration
            </button>
          </div>
        )
      )}
    </div>
  );
}
