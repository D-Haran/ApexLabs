import { useMemo, useState } from "react";
import type { Session, Lap } from "../telemetry/contract";
import { region, compare } from "../analysis/metrics";
import { fmt, kmh, signed } from "../data/units";
import { replay, useReplay } from "../state/replay";
import { lapDistance, lowerBound } from "../telemetry/replay";
export function ComparisonReadout({
  a,
  al,
  b,
  bl,
}: {
  a: Session;
  al: Lap;
  b: Session;
  bl: Lap;
}) {
  const { time } = useReplay();
  const delta = useMemo(() => compare(a, al, b, bl), [a, al, b, bl]);
  const s = lapDistance(a, al, time);
  const i = lowerBound(delta, s, (r) => r.s),
    d = delta[i];
  const covered = s >= delta[0].s && s <= delta.at(-1)!.s;
  const value =
    covered && d
      ? i === 0
        ? d.timeDelta
        : delta[i - 1].timeDelta +
          ((s - delta[i - 1].s) / (d.s - delta[i - 1].s)) *
            (d.timeDelta - delta[i - 1].timeDelta)
      : null;
  return (
    <div className="comparison-readout">
      <span>
        AT CURRENT DISTANCE <b>{fmt(s, 1)} m</b>
      </span>
      <strong className={value !== null && value < 0 ? "gain" : ""}>
        {value === null ? "Outside spatial coverage" : `${signed(value)} s`}
      </strong>
      <span>Δt = B − A · Negative = B ahead</span>
      <span>
        Recorded lap difference <b>{signed(bl.lap_time_s - al.lap_time_s)} s</b>
      </span>
    </div>
  );
}
export function RegionAnalysis({
  session,
  lap,
  b,
  bl,
}: {
  session: Session;
  lap: Lap;
  b: Session | null;
  bl: Lap | null;
}) {
  const grid = session.spatial.filter((r) => r.lap_number === lap.number);
  const [start, setStart] = useState(180),
    [end, setEnd] = useState(245);
  const compute = (s: Session, l: Lap) => {
    try {
      return { value: region(s, l, start, end), error: null };
    } catch (e) {
      return { value: null, error: (e as Error).message };
    }
  };
  const aResult = useMemo(
    () => compute(session, lap),
    [session, lap, start, end],
  );
  const bResult = useMemo(
    () => (b && bl ? compute(b, bl) : null),
    [b, bl, start, end],
  );
  const rows: [string, ReturnType<typeof compute>][] = [["A", aResult]];
  if (bResult) rows.push(["B", bResult]);
  return (
    <section className="region-panel">
      <div className="panel-heading">
        <span>REGION INSPECTION</span>
        <span>
          {grid[0].s_m}–{grid.at(-1)!.s_m} m coverage
        </span>
      </div>
      <div className="region-inputs">
        <label>
          From{" "}
          <input
            aria-label="Region start"
            type="number"
            value={start}
            onChange={(e) => setStart(Number(e.target.value))}
          />{" "}
          m
        </label>
        <span>→</span>
        <label>
          To{" "}
          <input
            aria-label="Region end"
            type="number"
            value={end}
            onChange={(e) => setEnd(Number(e.target.value))}
          />{" "}
          m
        </label>
      </div>
      <table>
        <thead>
          <tr>
            <th>Lap</th>
            <th>Entry</th>
            <th>Minimum</th>
            <th>Exit</th>
            <th>Time</th>
            <th>Peak |ay|</th>
            <th>Peak use</th>
          </tr>
        </thead>
        <tbody>
          {rows.map(([label, result]) => {
            const r = result.value;
            return (
              <tr key={label}>
                <td>{label}</td>
                {r ? (
                  <>
                    <td>
                      {fmt(kmh(r.entry))} <small>km/h</small>
                    </td>
                    <td>
                      {fmt(kmh(r.min))} <small>km/h</small>
                    </td>
                    <td>
                      {fmt(kmh(r.exit))} <small>km/h</small>
                    </td>
                    <td>
                      {fmt(r.time, 3)} <small>s</small>
                    </td>
                    <td>
                      {fmt(r.lat, 2)} <small>m/s²</small>
                    </td>
                    <td>
                      {fmt(r.util * 100)}
                      <small>%</small>
                    </td>
                  </>
                ) : (
                  <td colSpan={6}>{result.error}</td>
                )}
              </tr>
            );
          })}
        </tbody>
      </table>
      <p className="muted">
        Boundary speeds/time use exported spatial interpolation; extrema use
        enclosed recorded states.
      </p>
    </section>
  );
}
