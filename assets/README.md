# Visual assets

Milestone 5.5 integrates the supplied CC BY 4.0 McLaren P1 and Gulf McLaren F1 2022 meshes through
versioned semantic rigs. See [car provenance, limitations and processing commands](cars/README.md).
Processed GLBs/manifests are local runtime files under `apps/dashboard/public/assets/`.

The procedural coupe remains a fallback. Visual selection and dimensions never set vehicle physics.
Runtime coordinates are +X forward, +Y up, -Z left; wheel-derived contact planes and semantic pivots
provide grounding, steering and kinematic spin. Chassis heave/roll/pitch come only from C++ telemetry.

Asphalt/grass/gravel PBR tiles are authored deterministically by the repository, with metadata beside
them in `apps/dashboard/public/assets/surfaces/README.md`. No remote hotlinks, textures or HDR files
are required. Instanced vegetation and bounded terrain are illustrative track context; terrain away
from the circuit interpolates the existing elevation profile and is not independently surveyed.

Inter, Barlow Condensed and IBM Plex Mono are bundled through Fontsource (SIL Open Font License).
Lucide icons use the ISC license. Runtime asset dependencies are local after installation/processing.
