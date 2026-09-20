/** CPU timings, not GPU execution timings. Nested categories are not additive. */
const windowMs = 8000;
const samples = new Map<string, { time: number; ms: number }[]>();
export function recordCost(name: string, ms: number) {
  const now = performance.now(),
    values = samples.get(name) ?? [];
  values.push({ time: now, ms });
  while (values.length && values[0].time < now - windowMs) values.shift();
  samples.set(name, values);
}
export function measure<T>(name: string, operation: () => T): T {
  const start = performance.now();
  try {
    return operation();
  } finally {
    recordCost(name, performance.now() - start);
  }
}
export function cpuReport() {
  const now = performance.now();
  return Object.fromEntries(
    [...samples].map(([name, values]) => {
      const recent = values.filter((v) => v.time >= now - windowMs),
        sorted = recent.map((v) => v.ms).sort((a, b) => a - b);
      return [
        name,
        {
          meanMs:
            recent.reduce((a, b) => a + b.ms, 0) / Math.max(1, recent.length),
          p95Ms: sorted[Math.floor(sorted.length * 0.95)] ?? 0,
          count: recent.length,
          windowMs,
        },
      ];
    }),
  );
}
