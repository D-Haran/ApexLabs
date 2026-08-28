# ApexLab dashboard

Run `npm ci && npm run dev`, then open http://127.0.0.1:5173.

- `npm test`: deterministic contract/replay/analysis tests with real exported fixtures.
- `npm run build`: strict TypeScript and Vite production build.
- `npm run preview`: serve that build on http://127.0.0.1:4173.
- `npx playwright install --no-shell chromium && npm run test:e2e`: UI, screenshot and performance checks.
- `npm run format`: format client and central telemetry contract.

The default launch is deterministic and paused at the detected peak-utilization event.
`public/demo/baseline` and `public/demo/high-grip` are generated data, not source-code constants.
Regenerate them from the repository root:

```sh
python3 tools/python/replay/export_session.py --output apps/dashboard/public/demo/baseline
python3 tools/python/replay/export_session.py \
  --vehicle data/generated/milestone-4/vehicle_mu_1.35.json \
  --output apps/dashboard/public/demo/high-grip --name 'Technical circuit · higher grip'
```

For local files, select `session.json`, `telemetry.csv`, `wheels.csv`, `geometry.csv`, and
`spatial.csv` together. All file work stays in a worker inside the browser.

See [the full replay contract](../../docs/architecture/milestone-5-replay.md),
[validation report](../../docs/experiments/milestone-5-results.md), and
[asset adapter notes](../../assets/README.md).
