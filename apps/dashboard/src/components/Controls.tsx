import { Pause, Play, RotateCcw, StepBack, StepForward } from "lucide-react";
import { replay, useReplay } from "../state/replay";
import type { Session, Lap } from "../telemetry/contract";
import { fmt, kmh, deg, g, lapTime } from "../data/units";
import { lapDistance } from "../telemetry/replay";
export function ReplayControls({
  session,
  lap,
}: {
  session: Session;
  lap: Lap;
}) {
  const { time, playing, rate } = useReplay();
  return (
    <div className="replay-bar">
      <div className="transport">
        <button aria-label="Restart" onClick={replay.restart}>
          <RotateCcw size={16} />
        </button>
        <button aria-label="Previous frame" onClick={() => replay.step(-1)}>
          <StepBack size={17} />
        </button>
        <button
          className="play"
          aria-label={playing ? "Pause" : "Play"}
          onClick={playing ? replay.pause : replay.play}
        >
          {playing ? <Pause size={18} /> : <Play size={18} />}
        </button>
        <button aria-label="Next frame" onClick={() => replay.step(1)}>
          <StepForward size={17} />
        </button>
      </div>
      <span className="time-code" data-testid="cursor-time">
        {lapTime(time - lap.start_time_s)}
      </span>
      <input
        aria-label="Replay timeline"
        type="range"
        min={lap.start_time_s}
        max={lap.end_time_s}
        step="any"
        value={time}
        onChange={(e) => {
          replay.pause();
          replay.seek(Number(e.target.value));
        }}
      />
      <span className="duration">{lapTime(lap.lap_time_s)}</span>
      <select
        aria-label="Playback speed"
        value={rate}
        onChange={(e) => replay.setRate(Number(e.target.value))}
      >
        {[0.1, 0.25, 0.5, 1, 2, 4].map((v) => (
          <option key={v} value={v}>
            {v}×
          </option>
        ))}
      </select>
      <span className="distance-readout">
        {fmt(lapDistance(session, lap, time), 1)} <small>m</small>
      </span>
    </div>
  );
}
export function LiveReadout() {
  const { sample: s, playing } = useReplay();
  if (!s) return null;
  return (
    <>
      <div className="viewport-status">
        <span className={playing ? "status-dot playing" : "status-dot"} />
        {playing ? "REPLAYING" : "PAUSED"}
        <span>SCHEMA 04</span>
      </div>
      <div className="live-readout">
        <div className="live-speed">
          <strong>{fmt(kmh(s.speed_m_s), 1)}</strong>
          <span>km/h</span>
        </div>
        <div className="command">
          <label>
            THROTTLE <b>{fmt(s.throttle * 100, 0)}%</b>
          </label>
          <div className="meter">
            <i
              style={{ width: `${s.throttle * 100}%`, background: "#adc8ad" }}
            />
          </div>
        </div>
        <div className="command">
          <label>
            BRAKE <b>{fmt(s.brake * 100, 0)}%</b>
          </label>
          <div className="meter">
            <i style={{ width: `${s.brake * 100}%`, background: "#e89278" }} />
          </div>
        </div>
        <div className="live-steer">
          <span>STEER</span>
          <b>{fmt(deg(s.steering_angle_rad), 1)}°</b>
        </div>
        <div className="live-steer">
          <span>LATERAL</span>
          <b>{fmt(g(s.lateral_accel_m_s2), 2)} g</b>
        </div>
      </div>
    </>
  );
}
export function Inspector() {
  const { sample: s, time } = useReplay();
  if (!s) return null;
  const rows: [string, number, string, number][] = [
    ["Simulation time", time, "s", 3],
    ["Track position", s.track_s_m, "m", 2],
    ["Speed", s.speed_m_s, "m/s", 3],
    ["vx", s.vx_m_s, "m/s", 3],
    ["vy", s.vy_m_s, "m/s", 3],
    ["Yaw rate", deg(s.yaw_rate_rad_s), "°/s", 2],
    ["Steering", deg(s.steering_angle_rad), "°", 2],
    ["Throttle", s.throttle * 100, "%", 1],
    ["Brake", s.brake * 100, "%", 1],
    ["Lateral acceleration", s.lateral_accel_m_s2, "m/s²", 3],
    ["Longitudinal acceleration", s.longitudinal_accel_m_s2, "m/s²", 3],
    ["Lateral error", s.lateral_error_m, "m", 3],
    ["Heading error", deg(s.heading_error_rad), "°", 2],
  ];
  return (
    <details className="inspector">
      <summary>
        Current state inspector <span>SI + engineering units</span>
      </summary>
      <dl>
        {rows.map(([name, value, unit, precision]) => (
          <div key={name}>
            <dt>{name}</dt>
            <dd title={String(value)}>
              {fmt(value, precision)} <small>{unit}</small>
            </dd>
          </div>
        ))}
      </dl>
      <details>
        <summary>Raw interpolated snapshot</summary>
        <pre>{JSON.stringify(s, null, 2)}</pre>
      </details>
    </details>
  );
}
