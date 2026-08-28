import { useState } from "react";
import { wheels, type Wheel } from "../telemetry/contract";
import { useReplay } from "../state/replay";
import { fmt, deg } from "../data/units";
export function TirePanel() {
  const { sample: s } = useReplay(),
    [selected, setSelected] = useState<Wheel>("rl");
  if (!s) return null;
  const cap = s[`force_capacity_${selected}_n`],
    fx = s[`fx_${selected}_n`],
    fy = s[`fy_${selected}_n`],
    util = s[`friction_utilization_${selected}`],
    color = util > 0.98 ? "#ed806a" : "#e4aa79";
  return (
    <section className="tire-panel">
      <div className="panel-heading">
        <span>CAR STATUS</span>
        <span className="muted">FOUR-TIRE MODEL</span>
      </div>
      <div className="car-schematic">
        <svg viewBox="0 0 120 190" aria-label="Top-down vehicle load schematic">
          <path
            className="car-body"
            d="M43 20C48 12 72 12 77 20L84 47V143L75 176H45L36 143V47Z"
          />
          <path className="car-glass" d="M47 38H73L77 73H43Z" />
          <path className="car-glass" d="M43 116H77L72 151H48Z" />
          <path className="car-centerline" d="M60 17V176" />
          {[
            ["fl", 24, 50],
            ["fr", 96, 50],
            ["rl", 24, 139],
            ["rr", 96, 139],
          ].map(([wheel, x, y]) => (
            <g key={String(wheel)}>
              <rect
                x={Number(x) - 7}
                y={Number(y) - 17}
                width="14"
                height="34"
                rx="2"
              />
              <text x={x} y={Number(y) + 3} textAnchor="middle">
                {String(wheel).toUpperCase()}
              </text>
            </g>
          ))}
        </svg>
        <dl>
          {wheels.map((wheel) => (
            <button
              key={wheel}
              onClick={() => setSelected(wheel)}
              className={selected === wheel ? "selected" : ""}
              aria-label={`Inspect ${wheel.toUpperCase()} tire`}
            >
              <span>{wheel.toUpperCase()}</span>
              <b>{fmt(s[`fz_${wheel}_n`] / 1000, 2)} kN</b>
              <small>
                {fmt(s[`friction_utilization_${wheel}`] * 100, 0)}% use
              </small>
            </button>
          ))}
        </dl>
      </div>
      <div className="friction">
        <svg
          viewBox="0 0 150 150"
          aria-label={`${selected.toUpperCase()} friction circle`}
        >
          <circle
            cx="75"
            cy="75"
            r="55"
            fill="none"
            stroke="#3f4d4e"
            strokeWidth="1.5"
          />
          <circle
            cx="75"
            cy="75"
            r="27.5"
            fill="none"
            stroke="#2b3638"
            strokeDasharray="2 4"
          />
          <path d="M15 75H135M75 15V135" stroke="#354143" />
          {cap > 0 && (
            <>
              <line
                x1="75"
                y1="75"
                x2={75 + (fx / cap) * 55}
                y2={75 - (fy / cap) * 55}
                stroke={color}
                strokeWidth="1.5"
              />
              <circle
                cx={75 + (fx / cap) * 55}
                cy={75 - (fy / cap) * 55}
                r="4"
                fill={color}
              />
            </>
          )}
          <text x="119" y="91">
            Fx / cap
          </text>
          <text x="80" y="16">
            Fy / cap
          </text>
        </svg>
        <div className="force-readout">
          <span className="eyebrow">
            {selected.toUpperCase()} OPERATING POINT
          </span>
          <strong style={{ color }}>
            {fmt(util * 100, 1)}
            <small>%</small>
          </strong>
          <span>
            {s[`tire_saturated_${selected}`] ? "SATURATED" : "WITHIN LIMIT"}
          </span>
          <dl>
            <dt>Capacity</dt>
            <dd>{fmt(cap / 1000, 2)} kN</dd>
            <dt>Slip angle</dt>
            <dd>{fmt(deg(s[`slip_angle_${selected}_rad`]), 2)}°</dd>
          </dl>
        </div>
      </div>
      <div className="force-values">
        <span>
          Fx{" "}
          <b>
            {fmt(fx / 1000, 2)} <small>kN</small>
          </b>
        </span>
        <span>
          Fy{" "}
          <b>
            {fmt(fy / 1000, 2)} <small>kN</small>
          </b>
        </span>
        <span>
          Fz{" "}
          <b>
            {fmt(s[`fz_${selected}_n`] / 1000, 2)} <small>kN</small>
          </b>
        </span>
      </div>
      {cap === 0 && (
        <p className="muted">
          No force capacity; normalized point unavailable.
        </p>
      )}
    </section>
  );
}
