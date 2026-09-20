import { ghostAt, type DriveSample, type Ghost } from "./contract";

/** Wall-clock anchored, forward-only playback. No accumulated render deltas. */
export class DriveClock {
  session?: Ghost;
  currentSample?: DriveSample;
  playbackState: "paused" | "playing" = "paused";
  readonly playbackDirection = "forward";
  private rate = 1;
  private anchorTime = 0;
  private anchorWall = 0;
  private live: DriveSample[] = [];
  private receivedAt = 0;
  private frameTime = 0;
  constructor(private now = () => performance.now()) {}
  get playbackRate() {
    return this.rate;
  }
  get simulationTime() {
    return this.frameTime;
  }
  load(session?: Ghost) {
    this.session = session;
    this.playbackState = "paused";
    this.anchorTime = session?.samples[0]?.time ?? 0;
    this.frameTime = this.anchorTime;
    this.anchorWall = this.now();
  }
  private cursor(wall: number) {
    if (!this.session) return this.frameTime;
    const end = this.session.samples.at(-1)?.time ?? this.anchorTime;
    const t =
      this.anchorTime +
      (this.playbackState === "playing"
        ? (Math.max(0, wall - this.anchorWall) / 1000) * this.rate
        : 0);
    if (t >= end) {
      this.anchorTime = end;
      this.playbackState = "paused";
    }
    return Math.min(end, t);
  }
  play() {
    if (!this.session?.samples.length) return;
    const wall = this.now();
    this.anchorTime = this.cursor(wall);
    if (this.anchorTime >= this.session.samples.at(-1)!.time - 1e-9)
      this.anchorTime = this.session.samples[0].time;
    this.anchorWall = wall;
    this.playbackState = "playing";
    this.frameTime = this.anchorTime;
  }
  pause() {
    this.anchorTime = this.cursor(this.now());
    this.frameTime = this.anchorTime;
    this.playbackState = "paused";
  }
  seek(time: number) {
    if (!this.session?.samples.length || !Number.isFinite(time)) return;
    this.anchorTime = Math.max(
      this.session.samples[0].time,
      Math.min(this.session.samples.at(-1)!.time, time),
    );
    this.frameTime = this.anchorTime;
    this.anchorWall = this.now();
    this.playbackState = "paused";
  }
  setRate(rate: number) {
    if (!Number.isFinite(rate) || rate <= 0)
      throw new RangeError("Playback rate must be positive");
    const wall = this.now();
    this.anchorTime = this.cursor(wall);
    this.anchorWall = wall;
    this.rate = rate;
  }
  step() {
    this.pause();
    const rows = this.session?.samples;
    if (!rows) return;
    let lo = 0,
      hi = rows.length;
    while (lo < hi) {
      const mid = (lo + hi) >>> 1;
      if (rows[mid].time <= this.frameTime + 1e-9) lo = mid + 1;
      else hi = mid;
    }
    this.seek(rows[Math.min(lo, rows.length - 1)].time);
  }
  push(sample: DriveSample) {
    if (this.live.length && sample.time < this.live.at(-1)!.time) {
      this.live = [];
      this.frameTime = sample.time;
    }
    if (sample.time === this.live.at(-1)?.time) {
      this.live[this.live.length - 1] = sample;
      return;
    }
    this.live.push(sample);
    this.live = this.live.slice(-32);
    this.receivedAt = this.now();
  }
  read(wall = this.now()): DriveSample | undefined {
    if (this.session) {
      this.frameTime = this.cursor(wall);
      return (this.currentSample = ghostAt(
        this.session.samples,
        this.frameTime,
        "time",
      ));
    }
    const latest = this.live.at(-1);
    if (!latest) return;
    // One polling interval plus jitter margin. Never extrapolate missing physics.
    this.frameTime = latest.running
      ? Math.max(
          this.frameTime,
          this.live[0].time,
          Math.min(
            latest.time,
            latest.time - 0.06 + Math.max(0, wall - this.receivedAt) / 1000,
          ),
        )
      : latest.time;
    return (this.currentSample = ghostAt(this.live, this.frameTime, "time"));
  }
  clearLive() {
    this.live = [];
    this.frameTime = 0;
    this.currentSample = undefined;
  }
}
