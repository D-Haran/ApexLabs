import { useMemo, useState } from "react";
import type { Session, Lap } from "../telemetry/contract";
import { replay, useReplay } from "../state/replay";
import { seekDistance } from "../telemetry/replay";
import { utilization, lapRows } from "../analysis/metrics";
export function TrackMap({
  session,
  lap,
  comparison,
  comparisonLap,
}: {
  session: Session;
  lap: Lap;
  comparison?: Session | null;
  comparisonLap?: Lap | null;
}) {
  const { sample } = useReplay();
  const [color, setColor] = useState("Reference");
  const geo = session.geometry;
  const bounds = useMemo(() => {
    const xs = geo.map((r) => r.x_m),
      ys = geo.map((r) => -r.y_m);
    const x = Math.min(...xs) - 22,
      y = Math.min(...ys) - 22;
    return { x, y, w: Math.max(...xs) - x + 22, h: Math.max(...ys) - y + 22 };
  }, [geo]);
  const markerScale = Math.max(bounds.w, bounds.h) / 300;
  const path = useMemo(
    () => geo.map((r, i) => `${i ? "L" : "M"}${r.x_m},${-r.y_m}`).join(" "),
    [geo],
  );
  const trajectoryPath = useMemo(
    () =>
      lapRows(session, lap)
        .filter((_, index) => index % 30 === 0)
        .map(
          (row, index) =>
            `${index ? "L" : "M"}${row.position_x_m},${-row.position_y_m}`,
        )
        .join(" "),
    [session, lap],
  );
  const comparisonPath = useMemo(
    () =>
      comparison && comparisonLap
        ? lapRows(comparison, comparisonLap)
            .filter((_, index) => index % 30 === 0)
            .map(
              (row, index) =>
                `${index ? "L" : "M"}${row.position_x_m},${-row.position_y_m}`,
            )
            .join(" ")
        : "",
    [comparison, comparisonLap],
  );
  const segments = useMemo(() => {
    const rows = lapRows(session, lap).filter((_, i) => i % 40 === 0);
    return rows.slice(1).map((r, i) => {
      const v =
        color === "Speed"
          ? r.speed_m_s / 45
          : color === "Utilization"
            ? utilization(r)
            : Math.abs(r.lateral_accel_m_s2) / 12;
      return (
        <path
          key={i}
          d={`M${rows[i].position_x_m},${-rows[i].position_y_m}L${r.position_x_m},${-r.position_y_m}`}
          stroke={`hsl(${185 - v * 160} 55% 62%)`}
          strokeWidth={2}
          vectorEffect="non-scaling-stroke"
          fill="none"
        />
      );
    });
  }, [session, lap, color]);
  return (
    <section className="map-panel">
      <div className="panel-heading">
        <span>CIRCUIT POSITION</span>
        <select
          aria-label="Map coloring"
          value={color}
          onChange={(e) => setColor(e.target.value)}
        >
          {["Reference", "Speed", "Utilization", "Lateral g"].map((v) => (
            <option key={v}>{v}</option>
          ))}
        </select>
      </div>
      <svg
        role="img"
        aria-label="Interactive circuit map; click to seek"
        viewBox={`${bounds.x} ${bounds.y} ${bounds.w} ${bounds.h}`}
        onClick={(e) => {
          const svg = e.currentTarget,
            p = svg.createSVGPoint();
          p.x = e.clientX;
          p.y = e.clientY;
          const c = p.matrixTransform(svg.getScreenCTM()!.inverse());
          const closest = geo.reduce((a, b) =>
            Math.hypot(b.x_m - c.x, -b.y_m - c.y) <
            Math.hypot(a.x_m - c.x, -a.y_m - c.y)
              ? b
              : a,
          );
          replay.pause();
          replay.seek(seekDistance(session, lap, closest.s_m));
        }}
      >
        <path
          d={path}
          fill="none"
          stroke="#313a3d"
          strokeWidth="7"
          vectorEffect="non-scaling-stroke"
        />
        <path
          d={path}
          fill="none"
          stroke="#758986"
          strokeWidth="1.2"
          vectorEffect="non-scaling-stroke"
        />
        <path
          d={trajectoryPath}
          fill="none"
          stroke={session.manifest.optimization ? "#f0a66c" : "#6ba9df"}
          strokeWidth="1.4"
          vectorEffect="non-scaling-stroke"
          opacity="0.92"
        />
        {comparisonPath && (
          <path
            d={comparisonPath}
            fill="none"
            stroke={comparison?.manifest.optimization ? "#f0a66c" : "#6ba9df"}
            strokeWidth="1.4"
            vectorEffect="non-scaling-stroke"
            opacity="0.92"
          />
        )}
        {color !== "Reference" && segments}
        {session.manifest.track.sector_boundaries_fraction.map((f, i) => {
          const r = geo[Math.round(f * (geo.length - 1))];
          return (
            <g key={i}>
              <circle
                cx={r.x_m}
                cy={-r.y_m}
                r={4 * markerScale}
                fill="#bdc6c2"
              />
              <text
                x={r.x_m + 9 * markerScale}
                y={-r.y_m - 9 * markerScale}
                fill="#99a7a5"
                fontSize={13 * markerScale}
              >
                {i ===
                session.manifest.track.sector_boundaries_fraction.length - 1
                  ? "S/F"
                  : `S${i + 1}`}
              </text>
            </g>
          );
        })}
        {sample && (
          <>
            <circle
              cx={sample.position_x_m}
              cy={-sample.position_y_m}
              r={10 * markerScale}
              fill="#e9a26d"
              opacity="0.18"
            />
            <circle
              cx={sample.position_x_m}
              cy={-sample.position_y_m}
              r={4 * markerScale}
              fill="#f0b17f"
              stroke="#161c1e"
              strokeWidth={markerScale}
            />
          </>
        )}
      </svg>
      <div className="map-meta">
        <span>{(session.manifest.track.length_m / 1000).toFixed(3)} km</span>
        <span>
          <i className="map-line reference" /> Reference
          {comparisonPath && (
            <>
              <i className="map-line optimized" /> Optimized
            </>
          )}
        </span>
      </div>
      {color !== "Reference" && (
        <div className="color-key">
          {color === "Speed"
            ? "0 → 162 km/h"
            : color === "Utilization"
              ? "0 → 100% utilization"
              : "0 → 1.22 g"}
          <i />
        </div>
      )}
    </section>
  );
}
