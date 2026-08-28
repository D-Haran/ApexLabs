import { decodeSession, validateManifest } from "../telemetry/load";
self.onmessage = async (e: MessageEvent<{ url?: string; files?: File[] }>) => {
  const start = performance.now();
  try {
    let manifest: unknown;
    const files: Record<string, string> = {};
    if (e.data.url) {
      const fetchText = async (url: string) => {
        const r = await fetch(url);
        if (!r.ok) throw new Error(`Cannot load ${url}: ${r.status}`);
        return r.text();
      };
      manifest = JSON.parse(await fetchText(e.data.url));
      const m = validateManifest(manifest);
      await Promise.all(
        Object.values(m.files)
          .filter((name): name is string => Boolean(name))
          .map(
            async (name) =>
              (files[name] = await fetchText(new URL(name, e.data.url).href)),
          ),
      );
    } else {
      const input = e.data.files ?? [],
        names = new Set<string>();
      for (const file of input) {
        if (names.has(file.name))
          throw new Error(
            `Duplicate filename: ${file.name}. Select one session folder.`,
          );
        names.add(file.name);
      }
      const entry = input.find((f) => f.name === "session.json");
      if (!entry)
        throw new Error("Select session.json and its four companion CSV files");
      manifest = JSON.parse(await entry.text());
      const m = validateManifest(manifest);
      await Promise.all(
        Object.values(m.files)
          .filter((name): name is string => Boolean(name))
          .map(async (name) => {
            const f = input.find((f) => f.name === name);
            if (!f) throw new Error(`Missing file: ${name}`);
            files[name] = await f.text();
          }),
      );
    }
    const session = decodeSession(manifest, files);
    session.load_ms = performance.now() - start;
    self.postMessage({ session });
  } catch (error) {
    self.postMessage({
      error: error instanceof Error ? error.message : String(error),
    });
  }
};
