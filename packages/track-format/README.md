# Track formats v1 and v2

The source-of-truth track document is concise JSON containing closed-loop control points,
left/right widths, provenance, and generalized sector boundaries. Geometry is reconstructed
deterministically as a periodic uniform Catmull–Rom cubic. Arc-length and projection samples are
runtime cache data and are deliberately not serialized.

Coordinates are SI metres in the same inertial X/Y frame as vehicle telemetry. Control-point
order establishes forward travel. The left normal is `(-tangent_y, tangent_x)`, so positive
track-relative lateral displacement is left of the centerline. A point-CG is on track when
`-right_width <= e_y <= left_width`. Elevation and bank are API fields fixed at zero in v1.

Version 2 adds a local WGS84 east/north/up coordinate-system record, a required elevation on each
control point, real-versus-synthetic metadata, source/course-selection diagnostics, and an
imported-versus-published length check. Horizontal geometry and elevation use the same periodic
Catmull–Rom parameterization. Arc length is accumulated in 3D; `grade` is `dz` divided by local
horizontal distance. Banking remains zero until a reliable source is supplied.

The importer writes a separate `*.render.json` sidecar for pits and facility roads. This sidecar is
not read by `PeriodicTrack` and can never change physics geometry implicitly.

See `configs/tracks/schema-v1.json` / `schema-v2.json` for the machine-readable contracts and
`configs/tracks/technical_test_circuit.json` for the generated demonstration circuit.
