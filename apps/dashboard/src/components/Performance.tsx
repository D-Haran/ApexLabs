import { useState } from "react";
import { measure } from "../state/performance";
export function Performance() {
  const [busy, setBusy] = useState(false),
    [metrics, setMetrics] = useState<Awaited<
      ReturnType<typeof measure>
    > | null>(null);
  return (
    <details className="performance">
      <summary>Performance measurements</summary>
      <p>
        300 rendered frame intervals in the current view, 100 selector seeks,
        one seek-to-paint. Keep this tab visible.
      </p>
      <button
        disabled={busy}
        onClick={async () => {
          setBusy(true);
          try {
            setMetrics(await measure());
          } finally {
            setBusy(false);
          }
        }}
      >
        {busy ? "Measuring 300 frames…" : "Measure current view"}
      </button>
      {metrics && (
        <>
          <dl>
            {Object.entries(metrics).map(([k, v]) => (
              <div key={k}>
                <dt>{k.replaceAll("_", " ")}</dt>
                <dd>{typeof v === "number" ? v.toFixed(3) : v}</dd>
              </div>
            ))}
          </dl>
          <a
            download="apexlab-performance.json"
            href={`data:application/json,${encodeURIComponent(JSON.stringify(metrics, null, 2))}`}
          >
            Download measurements
          </a>
        </>
      )}
    </details>
  );
}
