export const kmh = (v: number) => v * 3.6;
export const deg = (v: number) => (v * 180) / Math.PI;
export const g = (v: number) => v / 9.80665;
export const fmt = (v: number, digits = 1) =>
  Number.isFinite(v) ? v.toFixed(digits) : "—";
export const signed = (v: number, digits = 3) =>
  (v > 0 ? "+" : "") + fmt(v, digits);
export const lapTime = (v: number) =>
  `${Math.floor(v / 60)}:${(v % 60).toFixed(3).padStart(6, "0")}`;
