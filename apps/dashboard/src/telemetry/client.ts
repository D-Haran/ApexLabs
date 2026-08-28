import type { Session } from "./contract";
export function loadSession(input: string | File[]): Promise<Session> {
  const start = performance.now();
  return new Promise((resolve, reject) => {
    const worker = new Worker(
      new URL("../workers/session.worker.ts", import.meta.url),
      { type: "module" },
    );
    worker.onmessage = (e) => {
      worker.terminate();
      if (e.data.error) reject(new Error(e.data.error));
      else resolve({ ...e.data.session, load_ms: performance.now() - start });
    };
    worker.onerror = (e) => {
      worker.terminate();
      reject(new Error(e.message));
    };
    worker.postMessage(
      typeof input === "string"
        ? { url: new URL(input, location.href).href }
        : { files: input },
    );
  });
}
