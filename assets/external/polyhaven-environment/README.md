# Local environment assets

Five texture/HDR files (3,821,179 bytes total) are downloaded by:

```sh
python3 tools/assets/fetch_environment.py
```

Sources: [Poly Haven pine tree 01](https://polyhaven.com/a/pine_tree_01) and
[Kloofendal partly cloudy sky](https://polyhaven.com/a/kloofendal_48d_partly_cloudy_puresky).
All are [CC0](https://polyhaven.com/license). The manifest stores original URLs,
byte counts and publisher MD5 checksums, which the importer verifies. Runtime
files are local; the application does not hotlink remote resources.

The source pine glTF contains three LOD0 meshes totaling 17,182,252 triangles and
a 948,849,556-byte geometry buffer. That source mesh was inspected but not downloaded
or put into Git. Instead the renderer instances authored 384-triangle branch-card
crowns and textured trunks using the source needle alpha/diffuse/normal atlas. This
is an explicit environment approximation, not the original scanned tree geometry.
There are no cone crowns in the new dynamic showcase. The older replay environment
retains its fallback geometry unless detailed rendering is requested.

The HDR is 1k, selected to keep environment loading bounded. Needle/bark maps are
1k; leaves use cutout alpha and double-sided PBR materials. Tree placement is
illustrative and excludes the road. It is not a survey of Spa's vegetation.
