import { useState } from "react";
import type { Session } from "../telemetry/contract";
import { fmt } from "../data/units";

type Tab = "Vehicle" | "Aero" | "Tires" | "Brakes" | "Optimization";

export function OptimizationPanel({
  session,
  hasOptimized,
  onReference,
  onOptimized,
  onCompare,
}: {
  session: Session;
  hasOptimized: boolean;
  onReference: () => void;
  onOptimized: () => void;
  onCompare: () => void;
}) {
  const [tab, setTab] = useState<Tab>("Optimization");
  const vehicle = session.manifest.vehicle;
  return (
    <aside className="setup-panel">
      <div className="panel-heading">
        <span>SETUP / OPTIMIZATION</span>
        <span className="muted">RECORDED CONFIG</span>
      </div>
      <div className="setup-tabs">
        {(["Vehicle", "Aero", "Tires", "Brakes", "Optimization"] as Tab[]).map(
          (name) => (
            <button
              key={name}
              className={tab === name ? "active" : ""}
              onClick={() => setTab(name)}
            >
              {name}
            </button>
          ),
        )}
      </div>
      {tab === "Vehicle" && (
        <dl className="setup-values">
          <div>
            <dt>Model</dt>
            <dd>{vehicle.name}</dd>
          </div>
          <div>
            <dt>Mass</dt>
            <dd>{fmt(vehicle.mass_kg, 0)} kg</dd>
          </div>
          <div>
            <dt>Wheelbase</dt>
            <dd>{fmt(vehicle.front_axle_m + vehicle.rear_axle_m, 3)} m</dd>
          </div>
          <div>
            <dt>Track F / R</dt>
            <dd>
              {fmt(vehicle.front_track_m, 3)} / {fmt(vehicle.rear_track_m, 3)} m
            </dd>
          </div>
        </dl>
      )}
      {tab === "Aero" &&
        (vehicle.aero ? (
          <dl className="setup-values">
            <div>
              <dt>Cd</dt>
              <dd>{fmt(vehicle.aero.drag_coefficient, 3)}</dd>
            </div>
            <div>
              <dt>Cl down</dt>
              <dd>{fmt(vehicle.aero.lift_coefficient_down, 3)}</dd>
            </div>
            <div>
              <dt>Reference area</dt>
              <dd>{fmt(vehicle.aero.reference_area_m2, 2)} m²</dd>
            </div>
            <div>
              <dt>Air density</dt>
              <dd>{fmt(vehicle.aero.air_density_kgpm3, 3)} kg/m³</dd>
            </div>
          </dl>
        ) : (
          <p className="setup-empty">
            No aero coefficients in this session manifest.
          </p>
        ))}
      {tab === "Tires" && (
        <dl className="setup-values">
          <div>
            <dt>Reference μ</dt>
            <dd>{fmt(vehicle.mu_reference, 3)}</dd>
          </div>
          <div>
            <dt>Model</dt>
            <dd>Nonlinear load-sensitive force limit</dd>
          </div>
        </dl>
      )}
      {tab === "Brakes" && (
        <p className="setup-empty">
          Brake demand is modeled; detailed hardware setup is not exposed.
        </p>
      )}
      {tab === "Optimization" && (
        <div className="optimization-actions">
          <button onClick={onReference}>Reference simulation</button>
          <button
            disabled
            title="Offline CasADi/IPOPT command is documented in the Phase D report"
          >
            Optimize fixed line
          </button>
          <button disabled={!hasOptimized} onClick={onOptimized}>
            View optimized racing line
          </button>
          <button disabled={!hasOptimized} onClick={onCompare}>
            Compare solutions
          </button>
          <p>
            Minimum-time solves run offline with CasADi/IPOPT. Displayed
            optimized laps are independent production-simulator replays.
          </p>
          <details>
            <summary>Advanced solver settings</summary>
            <dl className="setup-values">
              <div>
                <dt>Transcription</dt>
                <dd>Spatial direct collocation</dd>
              </div>
              <div>
                <dt>CG margin</dt>
                <dd>2.5 m</dd>
              </div>
              <div>
                <dt>Claim</dt>
                <dd>Local optimum</dd>
              </div>
            </dl>
          </details>
        </div>
      )}
      <p className="setup-note">
        Recorded parameters are read-only. Visual changes never alter physics.
      </p>
    </aside>
  );
}
