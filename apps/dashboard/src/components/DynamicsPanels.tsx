import { useReplay } from "../state/replay";
import type { Session } from "../telemetry/contract";
import { fmt, g } from "../data/units";

export function AeroLoadPanel({ session }: { session: Session }) {
  const { sample } = useReplay();
  const aero = session.manifest.vehicle.aero;
  if (!sample || !aero) return null;
  const dynamicPressure = 0.5 * aero.air_density_kgpm3 * sample.speed_m_s ** 2;
  const drag = dynamicPressure * aero.drag_coefficient * aero.reference_area_m2;
  const downforce =
    dynamicPressure * aero.lift_coefficient_down * aero.reference_area_m2;
  return (
    <section className="dynamics-panel aero-panel">
      <div className="panel-heading">
        <span>AERO / LOAD</span>
        <span className="muted">CONFIGURED APPROX.</span>
      </div>
      <div className="aero-values">
        <div>
          <span>DRAG</span>
          <strong>{fmt(drag / 1000, 2)}</strong>
          <small>kN</small>
        </div>
        <div>
          <span>DOWNFORCE</span>
          <strong>{fmt(downforce / 1000, 2)}</strong>
          <small>kN</small>
        </div>
      </div>
      <dl className="coefficient-list">
        <div>
          <dt>Cd</dt>
          <dd>{fmt(aero.drag_coefficient, 3)}</dd>
        </div>
        <div>
          <dt>Cl↓</dt>
          <dd>{fmt(aero.lift_coefficient_down, 3)}</dd>
        </div>
        <div>
          <dt>Reference area</dt>
          <dd>{fmt(aero.reference_area_m2, 2)} m²</dd>
        </div>
      </dl>
    </section>
  );
}

export function GForcePanel() {
  const { sample } = useReplay();
  if (!sample) return null;
  const longitudinal = g(sample.longitudinal_accel_m_s2);
  const lateral = g(sample.lateral_accel_m_s2);
  const scale = 34 / 1.5;
  const x = Math.max(-34, Math.min(34, lateral * scale));
  const y = Math.max(-34, Math.min(34, longitudinal * scale));
  return (
    <section className="dynamics-panel g-panel">
      <div className="panel-heading">
        <span>G-FORCES</span>
        <span className="muted">ACTUAL MODEL STATE</span>
      </div>
      <div className="g-content">
        <svg
          viewBox="0 0 100 100"
          aria-label="G-force plot from modeled ax and ay"
        >
          <circle cx="50" cy="50" r="34" />
          <circle cx="50" cy="50" r="17" className="minor" />
          <path d="M10 50H90M50 10V90" />
          <circle cx={50 + x} cy={50 - y} r="4" className="g-marker" />
          <text x="84" y="47">
            ay
          </text>
          <text x="53" y="14">
            ax
          </text>
        </svg>
        <dl>
          <div>
            <dt>LONG.</dt>
            <dd>{fmt(longitudinal, 2)} g</dd>
          </div>
          <div>
            <dt>LATERAL</dt>
            <dd>{fmt(lateral, 2)} g</dd>
          </div>
        </dl>
      </div>
    </section>
  );
}
