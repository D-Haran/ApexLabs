import { useSyncExternalStore } from "react";
import { lowerBound, sampleAt } from "../telemetry/replay";
import type { Lap, Sample, Session } from "../telemetry/contract";
export interface Snapshot {
  time: number;
  playing: boolean;
  rate: number;
  sample: Sample | null;
  revision: number;
}
export class ReplayStore {
  session: Session | null = null;
  lap: Lap | null = null;
  private listeners = new Set<() => void>();
  private snapshot: Snapshot = {
    time: 0,
    playing: false,
    rate: 1,
    sample: null,
    revision: 0,
  };
  subscribe = (fn: () => void) => {
    this.listeners.add(fn);
    return () => {
      this.listeners.delete(fn);
    };
  };
  getSnapshot = () => this.snapshot;
  private emit(patch: Partial<Snapshot>) {
    this.snapshot = {
      ...this.snapshot,
      ...patch,
      revision: this.snapshot.revision + 1,
    };
    this.listeners.forEach((fn) => fn());
  }
  load(session: Session, lap = session.manifest.laps[0]) {
    this.session = session;
    this.lap = lap;
    this.emit({
      playing: false,
      time: lap.start_time_s,
      sample: sampleAt(session, lap.start_time_s),
    });
  }
  seek = (time: number) => {
    if (!this.session || !this.lap || !Number.isFinite(time)) return;
    const t = Math.max(
      this.lap.start_time_s,
      Math.min(this.lap.end_time_s, time),
    );
    this.emit({ time: t, sample: sampleAt(this.session, t) });
  };
  play = () => {
    if (!this.lap) return;
    if (this.snapshot.time >= this.lap.end_time_s)
      this.seek(this.lap.start_time_s);
    this.emit({ playing: true });
  };
  pause = () => this.emit({ playing: false });
  restart = () => {
    this.pause();
    if (this.lap) this.seek(this.lap.start_time_s);
  };
  setRate = (rate: number) => {
    if ([0.1, 0.25, 0.5, 1, 2, 4].includes(rate)) this.emit({ rate });
  };
  advance = (seconds: number) => {
    if (!this.snapshot.playing || !this.lap) return;
    this.seek(this.snapshot.time + Math.max(0, seconds) * this.snapshot.rate);
    if (this.snapshot.time >= this.lap.end_time_s) this.pause();
  };
  step = (direction: 1 | -1) => {
    if (!this.session) return;
    this.pause();
    const rows = this.session.samples,
      i = lowerBound(rows, this.snapshot.time, (r) => r.time_s);
    const index =
      direction === 1
        ? rows[i]?.time_s === this.snapshot.time
          ? i + 1
          : i
        : i - 1;
    this.seek(rows[Math.max(0, Math.min(rows.length - 1, index))].time_s);
  };
}
export const replay = new ReplayStore();
export const useReplay = () =>
  useSyncExternalStore(replay.subscribe, replay.getSnapshot);
