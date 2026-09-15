# Generated decorative foliage atlas

Runtime asset: `apps/dashboard/public/assets/environment/forest-atlas-generated.png`.
Created with the built-in image_gen tool using the imagegen skill, 2026-09-20.
It has genuine PNG alpha, preserved without image editing. This file is **not** a
Poly Haven asset and is not claimed to be a photograph or survey of Spa vegetation.
The renderer instances camera-facing vertical forest clusters in a single draw call.
They are visual-only impostors with baked lighting, not physics geometry. This is
a performance/quality approximation; near-field parallax and physical foliage
lighting are not reproduced. Local CC0 sky and surface maps remain independently
attributed in the asset manifest. The branch-card implementation is retained for
further development but is not the selected showcase foliage.

Final prompt (built-in generation, not CLI):

> Use case: photorealistic-natural. Asset type: transparent game foliage atlas for a realistic Ardennes motorsport circuit environment. Create one square high-resolution RGBA image with three full-height photorealistic European conifer trees side by side, each fully isolated on a genuinely transparent background. Three vertical equal-width cells: left mature Norway spruce with broad dense green irregular branches, center Scots pine with visible textured trunk and an irregular dark-green crown, right slender dense spruce. Trees occupy 90 percent image height from roots/trunk base at same bottom baseline, with generous clear transparent margins between cells, no overlap. Natural detailed needles, asymmetric organic silhouettes and small transparent gaps between branches. Soft even overcast daylight, muted natural healthy forest greens, moderate contrast, no black silhouettes. Orthographic front view suited to alpha-cutout tree billboard textures. No ground, no grass, no horizon, no cast shadow, no white background, no visible checkerboard, no text, no watermark. This is decorative rendering scenery, not a scientific map.

The output's cell widths are unequal, so the full atlas is used as a three-tree
cluster rather than cropping trees at cell boundaries.
