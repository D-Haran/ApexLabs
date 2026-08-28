# Supplied car assets and reproducible processing

Both source packages are present locally. Identification uses the `asset.extras` metadata and
included `license.txt`, not the directory name or generic Object_* nodes.

- **Mclaren P1** by **Kevin Love SketchFab / Tyler_Kevin**:
  https://sketchfab.com/3d-models/mclaren-p1-7917d2a323b24c89acb418464376c4c9
- **Gulf Mclaren F1 2022 Car** by **chwashere123**:
  https://sketchfab.com/3d-models/gulf-mclaren-f1-2022-car-dabbf81921e94065bfadbb5fd1b2ee5b

Both supplied packages declare **CC BY 4.0**: https://creativecommons.org/licenses/by/4.0/ .
These credits must accompany redistribution. No authentication/download step is needed for this
workspace. Source GLTF, scene.bin and textures remain unmodified and ignored by Git; processed
GLBs are also ignored. Manifests, processing code, and attribution remain versionable. Use Git LFS
if a distribution chooses to commit GLBs; do not add raw meshes to ordinary Git history.

From repository root (Python 3.13 used here):

```sh
uv venv .cache/car-venv
uv pip install --python .cache/car-venv/bin/python -r tools/assets/requirements.txt
npm --prefix apps/dashboard ci
.cache/car-venv/bin/python tools/assets/process_car_asset.py assets/cars/mclaren_p1/scene.gltf apps/dashboard/public/assets/mclaren-p1.glb --forward +Z --wheelbase 2.67
.cache/car-venv/bin/python tools/assets/process_car_asset.py assets/cars/gulf_mclaren_f1_2022_car/scene.gltf apps/dashboard/public/assets/mclaren-f1-2022.glb --forward +Z --wheelbase 3.6
node tools/assets/compress_car_asset.mjs apps/dashboard/public/assets/mclaren-p1.glb apps/dashboard/public/assets/mclaren-f1-2022.glb
.cache/car-venv/bin/python tools/assets/create_surface_textures.py
.cache/car-venv/bin/python -m unittest discover -s tools/assets -p test_process_car_asset.py
```

The preprocessor supports static glTF triangle packages. Missing/unsafe resource paths, unsupported
skins/animations/sparse accessors and ambiguous wheel geometry fail explicitly. It applies all
scene graph transforms, welds coincident positions, computes true triangle connectivity, and merges
concentric tire shells spatially. Four validated wheel centers define axle pairs and side labels.
Source node names have **no semantic role**. Material names identify tire/rim/disc/caliper roles.
Front steer pivots contain spin pivots for tire/rim/disc; calipers remain direct steer children.

The verified source **scene** convention is +Y up, +Z forward. P1 taillight/headlight geometry
establishes the forward sign; F1 nose/wing inspection confirms it. Units are not declared by the
authors. P1 scale is anchored to the existing published 2.67 m wheelbase; F1's **3.6 m is an explicit
engineering visual normalization assumption**, not a measured/proprietary McLaren chassis spec.
Runtime uses +X forward, +Y up, -Z left. The source axle midpoint becomes local x=0; minimum tire
heights determine the contact plane, and per-wheel contact corrections are recorded. The rig shifts
that midpoint relative to the physics CG without rescaling the car to match another physics model.

Manifests beside the GLBs contain dimensions, radii, every final semantic mapping, transforms,
source hash, triangle counts, file sizes and texture memory estimates. P1 blur/damaged-glass
alternative meshes are intentionally excluded. The body silhouette is not simplified. Textures
are embedded and capped at 1024 pixels; no runtime remote resources are needed. Meshopt geometry
compression is lossless and every decoded buffer is checked byte-for-byte. The extension follows
[the Khronos specification](https://github.com/KhronosGroup/glTF/tree/main/extensions/2.0/Vendor/EXT_meshopt_compression).
Draco, KTX2 and LODs are not required by the measured performance; KTX2 would reduce GPU texture
memory but needs an additional encoder/transcoder deployment. No arbitrary polygon target is used.

**Source limitation:** F1 `brake` contains rear low-detail blocks only; treated conservatively as
stationary brake assemblies. Front brake discs/calipers are absent from this source and are not
fabricated. P1 has separate tire/rim/disc/caliper geometry at all four wheels.
Visual spin integrates each rigid-body contact-patch longitudinal velocity / measured radius, not physical wheel dynamics,
wheelspin, ABS or TC. F1 visual selection continues to use the clearly displayed P1 approximation
unless another physical session is opened. Neither mesh supplies physical parameters.
