/** Exact critically damped spring for a constant target over dt. */
export function spring(
  position: number,
  velocity: number,
  target: number,
  omega: number,
  dt: number,
): [number, number] {
  const offset = position - target,
    decay = Math.exp(-omega * dt),
    impulse = velocity + omega * offset;
  return [
    target + (offset + impulse * dt) * decay,
    (velocity - omega * impulse * dt) * decay,
  ];
}
export const cameraFrequency = { Tight: 18, Standard: 11, Cinematic: 5 };
export type ChaseMode = keyof typeof cameraFrequency;
