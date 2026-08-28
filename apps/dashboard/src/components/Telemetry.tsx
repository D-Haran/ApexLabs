import { useEffect, useMemo, useRef, useState } from "react";
import uPlot from "uplot";
import "uplot/dist/uPlot.min.css";
import {
  wheels,
  type Session,
  type Lap,
  type Field,
  type Sample,
} from "../telemetry/contract";
import { replay } from "../state/replay";
import { lapDistance, seekDistance, sampleAt } from "../telemetry/replay";
import { compare, spatialAt } from "../analysis/metrics";
import { kmh, deg, g } from "../data/units";
interface Channel {
  key: Field | "delta_time" | "delta_speed";
  name: string;
  unit: string;
  color: string;
  convert: (n: number) => number;
}
const identity = (n: number) => n;
export const channels: Channel[] = [
  {
    key: "speed_m_s",
    name: "Speed",
    unit: "km/h",
    color: "#b6d2c8",
    convert: kmh,
  },
  {
    key: "throttle",
    name: "Throttle",
    unit: "%",
    color: "#9abe9b",
    convert: (n) => n * 100,
  },
  {
    key: "brake",
    name: "Brake",
    unit: "%",
    color: "#e4927d",
    convert: (n) => n * 100,
  },
  {
    key: "steering_angle_rad",
    name: "Steering",
    unit: "°",
    color: "#c7b59a",
    convert: deg,
  },
  {
    key: "lateral_accel_m_s2",
    name: "Lateral acceleration",
    unit: "g",
    color: "#93bacd",
    convert: g,
  },
  {
    key: "longitudinal_accel_m_s2",
    name: "Longitudinal acceleration",
    unit: "g",
    color: "#afa9c7",
    convert: g,
  },
  {
    key: "yaw_rate_rad_s",
    name: "Yaw rate",
    unit: "°/s",
    color: "#c2baa3",
    convert: deg,
  },
  {
    key: "lateral_error_m",
    name: "Lateral error",
    unit: "m",
    color: "#d2b393",
    convert: identity,
  },
  ...wheels.map((w) => ({
    key: `friction_utilization_${w}` as Field,
    name: `${w.toUpperCase()} utilization`,
    unit: "%",
    color: "#e4aa79",
    convert: (n: number) => n * 100,
  })),
  ...wheels.map((w) => ({
    key: `fz_${w}_n` as Field,
    name: `${w.toUpperCase()} Fz`,
    unit: "kN",
    color: "#92b7ca",
    convert: (n: number) => n / 1000,
  })),
  {
    key: "delta_time",
    name: "Time delta · B − A",
    unit: "s",
    color: "#e4aa79",
    convert: identity,
  },
  {
    key: "delta_speed",
    name: "Speed delta · B − A",
    unit: "km/h",
    color: "#8cbdce",
    convert: kmh,
  },
];
function Plot({
  channel,
  data,
  axis,
  session,
  lap,
  range,
  onRange,
}: {
  channel: Channel;
  data: uPlot.AlignedData;
  axis: "time" | "distance";
  session: Session;
  lap: Lap;
  range: [number, number] | null;
  onRange: (r: [number, number] | null) => void;
}) {
  const node = useRef<HTMLDivElement>(null),
    plot = useRef<uPlot | null>(null),
    rangeRef = useRef(range),
    panning = useRef(false);
  rangeRef.current = range;
  const [hover, setHover] = useState<string>("");
  useEffect(() => {
    if (!node.current) return;
    const root = node.current;
    const p = new uPlot(
      {
        width: root.clientWidth || 800,
        height: 110,
        padding: [10, 14, 0, 0],
        legend: { show: false },
        select: { show: true, left: 0, top: 0, width: 0, height: 0 },
        cursor: { drag: { x: true, y: false, setScale: false } },
        scales: { x: { time: false }, y: { auto: true } },
        axes: [
          {
            stroke: "#849291",
            grid: { show: false },
            ticks: { stroke: "#303c3f" },
            font: "10px ui-monospace",
            size: 24,
            values: (_, values) =>
              values.map(
                (v) => `${Math.round(v)}${axis === "distance" ? " m" : " s"}`,
              ),
          },
          {
            stroke: "#849291",
            grid: { stroke: "#253033", width: 1 },
            ticks: { show: false },
            font: "10px ui-monospace",
            size: 50,
          },
        ],
        series: [
          {},
          ...data.slice(1).map((_, i) => ({
            stroke: i ? "#e5a777" : channel.color,
            width: 1.5,
            dash: i ? [5, 3] : undefined,
          })),
        ],
        hooks: {
          setSelect: [
            (u) => {
              if (panning.current) {
                u.setSelect({ left: 0, top: 0, width: 0, height: 0 }, false);
                return;
              }
              if (u.select.width > 4) {
                onRange([
                  u.posToVal(u.select.left, "x"),
                  u.posToVal(u.select.left + u.select.width, "x"),
                ]);
                u.setSelect({ left: 0, top: 0, width: 0, height: 0 }, false);
              }
            },
          ],
          setCursor: [
            (u) => {
              const i = u.cursor.idx;
              if (i != null)
                setHover(
                  data
                    .slice(1)
                    .map(
                      (series, j) =>
                        `${j ? "B" : "A"} ${series[i]?.toFixed(2) ?? "—"} ${channel.unit}`,
                    )
                    .join("  /  "),
                );
            },
          ],
        },
      },
      data,
      root,
    );
    plot.current = p;
    if (rangeRef.current)
      p.setScale("x", { min: rangeRef.current[0], max: rangeRef.current[1] });
    const indicator = document.createElement("div");
    indicator.className = "shared-cursor";
    p.over.append(indicator);
    const update = () => {
      const { time } = replay.getSnapshot();
      const x =
        axis === "time"
          ? time - lap.start_time_s
          : lapDistance(session, lap, time);
      const left = p.valToPos(x, "x");
      indicator.style.left = `${left}px`;
      indicator.style.display =
        left < 0 || left > p.over.clientWidth ? "none" : "block";
    };
    const unsub = replay.subscribe(update);
    update();
    let down = 0,
      panStart: number | null = null,
      initial: [number, number] | null = null;
    const pointerDown = (e: PointerEvent) => {
      down = e.clientX;
      panning.current = e.shiftKey;
      if (e.shiftKey) {
        panStart = e.clientX;
        initial = [p.scales.x.min!, p.scales.x.max!];
      }
    };
    const pointerUp = (e: PointerEvent) => {
      if (panStart !== null && initial) {
        const dx =
          ((e.clientX - panStart) / p.over.clientWidth) *
          (initial[1] - initial[0]);
        onRange([initial[0] - dx, initial[1] - dx]);
        panStart = null;
        return;
      }
      if (Math.abs(e.clientX - down) < 4) {
        const x = p.posToVal(
          e.clientX - p.over.getBoundingClientRect().left,
          "x",
        );
        replay.pause();
        replay.seek(
          axis === "time"
            ? lap.start_time_s + x
            : seekDistance(session, lap, x),
        );
      }
    };
    const wheel = (e: WheelEvent) => {
      e.preventDefault();
      const min = p.scales.x.min!,
        max = p.scales.x.max!,
        center = p.posToVal(
          e.clientX - p.over.getBoundingClientRect().left,
          "x",
        ),
        factor = e.deltaY > 0 ? 1.2 : 0.8;
      onRange([
        center + (min - center) * factor,
        center + (max - center) * factor,
      ]);
    };
    p.over.addEventListener("pointerdown", pointerDown);
    p.over.addEventListener("pointerup", pointerUp);
    p.over.addEventListener("wheel", wheel, { passive: false });
    const resize = new ResizeObserver(() => {
      p.setSize({ width: root.clientWidth, height: 110 });
      update();
    });
    resize.observe(root);
    return () => {
      unsub();
      resize.disconnect();
      p.over.removeEventListener("pointerdown", pointerDown);
      p.over.removeEventListener("pointerup", pointerUp);
      p.over.removeEventListener("wheel", wheel);
      p.destroy();
      plot.current = null;
    };
  }, [channel, data, axis, session, lap, onRange]);
  useEffect(() => {
    const p = plot.current;
    if (p) {
      p.setScale(
        "x",
        range
          ? { min: range[0], max: range[1] }
          : { min: data[0][0], max: data[0].at(-1)! },
      );
      const { time } = replay.getSnapshot();
      const x =
        axis === "time"
          ? time - lap.start_time_s
          : lapDistance(session, lap, time);
      const el = p.over.querySelector<HTMLElement>(".shared-cursor");
      if (el) el.style.left = `${p.valToPos(x, "x")}px`;
    }
  }, [range, data, axis, lap, session]);
  return (
    <div className="plot-row" data-range={range ? range.join(",") : "full"}>
      <div className="plot-label">
        <b style={{ color: channel.color }}>{channel.name}</b>
        <span>{hover || channel.unit}</span>
      </div>
      <div ref={node} />
    </div>
  );
}
export function Telemetry({
  session,
  lap,
  comparison,
  comparisonLap,
}: {
  session: Session;
  lap: Lap;
  comparison: Session | null;
  comparisonLap: Lap | null;
}) {
  const [axis, setAxis] = useState<"time" | "distance">("distance"),
    [selected, setSelected] = useState<string[]>([
      "speed_m_s",
      "throttle",
      "brake",
      "steering_angle_rad",
    ]),
    [range, setRange] = useState<[number, number] | null>(null),
    [menu, setMenu] = useState(false);
  useEffect(() => {
    setRange(null);
  }, [session, lap, axis]);
  useEffect(() => {
    if (comparison) {
      setAxis("distance");
      setSelected((v) =>
        v.includes("delta_time") ? v : [...v, "delta_time", "delta_speed"],
      );
    } else setSelected((v) => v.filter((k) => !k.startsWith("delta_")));
  }, [comparison]);
  const spatial = useMemo(
    () => session.spatial.filter((r) => r.lap_number === lap.number),
    [session, lap],
  );
  const rows = useMemo(
    () =>
      axis === "time"
        ? session.samples.filter(
            (r) => r.time_s >= lap.start_time_s && r.time_s <= lap.end_time_s,
          )
        : spatial.map((r) => ({
            ...sampleAt(session, r.time_s),
            ...Object.fromEntries(
              [
                "speed_m_s",
                "throttle",
                "brake",
                "steering_angle_rad",
                "lateral_accel_m_s2",
                "lateral_error_m",
              ].map((k) => [k, r[k as keyof typeof r]]),
            ),
          })),
    [session, lap, axis, spatial],
  );
  const x = useMemo(
    () =>
      axis === "time"
        ? rows.map((r) => r.time_s - lap.start_time_s)
        : spatial.map((r) => r.s_m),
    [rows, spatial, axis, lap],
  );
  const other = useMemo(() => {
    if (!comparison || !comparisonLap || axis !== "distance") return null;
    const grid = comparison.spatial.filter(
      (r) => r.lap_number === comparisonLap.number,
    );
    return x.map((s) => {
      const r = spatialAt(grid, s);
      return r
        ? {
            ...sampleAt(comparison, r.time_s),
            ...Object.fromEntries(
              [
                "speed_m_s",
                "throttle",
                "brake",
                "steering_angle_rad",
                "lateral_accel_m_s2",
                "lateral_error_m",
              ].map((k) => [k, r[k as keyof typeof r]]),
            ),
          }
        : null;
    });
  }, [comparison, comparisonLap, axis, x]);
  const deltas = useMemo(
    () =>
      comparison && comparisonLap
        ? compare(session, lap, comparison, comparisonLap)
        : [],
    [session, lap, comparison, comparisonLap],
  );
  const plots = useMemo(
    () =>
      channels
        .filter(
          (c) =>
            selected.includes(c.key) &&
            (!c.key.startsWith("delta_") ||
              (axis === "distance" && comparison)),
        )
        .map((channel) => {
          let data: uPlot.AlignedData;
          if (channel.key === "delta_time" || channel.key === "delta_speed")
            data = [
              deltas.map((d) => d.s),
              deltas.map((d) =>
                channel.convert(
                  channel.key === "delta_time" ? d.timeDelta : d.speedDelta,
                ),
              ),
            ];
          else
            data = [
              x,
              rows.map((r) => channel.convert(r[channel.key as Field])),
              ...(other
                ? [
                    other.map((r) =>
                      r ? channel.convert(r[channel.key as Field]) : null,
                    ),
                  ]
                : []),
            ];
          return { channel, data };
        }),
    [selected, axis, comparison, deltas, x, rows, other],
  );
  return (
    <section className="telemetry-panel">
      <div className="telemetry-heading">
        <div>
          <span className="eyebrow">SYNCHRONIZED TELEMETRY</span>
          <span className="legend-a">A · {session.manifest.session_name}</span>
          {comparison && (
            <span className="legend-b">
              B · {comparison.manifest.session_name}
            </span>
          )}
        </div>
        <div className="toolbar">
          <button onClick={() => setRange(null)}>Reset view</button>
          <button
            className={menu ? "active" : ""}
            onClick={() => setMenu(!menu)}
          >
            Channels {selected.length}
          </button>
          <div className="segmented">
            <button
              className={axis === "time" ? "active" : ""}
              onClick={() => setAxis("time")}
            >
              Time
            </button>
            <button
              className={axis === "distance" ? "active" : ""}
              onClick={() => setAxis("distance")}
            >
              Distance
            </button>
          </div>
        </div>
      </div>
      {menu && (
        <div className="channel-menu">
          {channels
            .filter((c) => !c.key.startsWith("delta_") || comparison)
            .map((c) => (
              <label key={c.key}>
                <input
                  type="checkbox"
                  checked={selected.includes(c.key)}
                  onChange={() =>
                    setSelected((v) =>
                      v.includes(c.key)
                        ? v.filter((k) => k !== c.key)
                        : [...v, c.key],
                    )
                  }
                />
                {c.name} <small>{c.unit}</small>
              </label>
            ))}
        </div>
      )}
      <div className="plots">
        {plots.map(({ channel, data }) => (
          <Plot
            key={channel.key}
            channel={channel}
            data={data}
            axis={axis}
            session={session}
            lap={lap}
            range={range}
            onRange={setRange}
          />
        ))}
      </div>
      <div className="chart-help">
        Click to seek · Drag to zoom · Shift-drag to pan · Scroll to zoom
        {comparison && " · Dashed trace = Lap B · Negative Δt = B ahead"}
      </div>
    </section>
  );
}
