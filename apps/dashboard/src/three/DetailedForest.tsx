import { useTexture } from "@react-three/drei";
import { useEffect, useMemo, useRef } from "react";
import * as THREE from "three";
import { mergeGeometries } from "three/examples/jsm/utils/BufferGeometryUtils.js";
import { forestPlacements } from "./environmentGeometry";
import type { EnvironmentSession } from "./TrackEnvironment";
/** Authored branch-card geometry with CC0 Poly Haven needle/bark maps. */
export default function DetailedForest({
  session,
}: {
  session: EnvironmentSession;
}) {
  const leaves = useRef<THREE.InstancedMesh>(null),
    trunks = useRef<THREE.InstancedMesh>(null);
  const [diff, alpha, normal, bark] = useTexture([
    "/assets/environment/twig_diff.jpg",
    "/assets/environment/twig_alpha.png",
    "/assets/environment/twig_nor_gl.jpg",
    "/assets/environment/bark_diff.jpg",
  ]);
  const placements = useMemo(
    () => forestPlacements(session.geometry).filter((_, i) => i % 2 === 0),
    [session],
  );
  const geometry = useMemo(() => {
    const cards: THREE.BufferGeometry[] = [];
    for (let layer = 0; layer < 12; layer++)
      for (let branch = 0; branch < 8; branch++) {
        const y = 0.2 + layer * 0.063,
          angle = (branch * Math.PI) / 4 + layer * 2.399,
          reach = (1 - y) * 0.9;
        for (let cross = 0; cross < 2; cross++) {
          const g = new THREE.PlaneGeometry(0.24, 0.52);
          const uv = g.getAttribute("uv");
          for (let k = 0; k < uv.count; k++)
            uv.setXY(k, uv.getX(k) * 0.235, 0.555 + uv.getY(k) * 0.435);
          g.rotateX(-Math.PI / 2 + cross * 0.8);
          g.rotateZ(-0.3);
          g.rotateY(angle);
          g.scale(
            (1 - y) * 1.5 + 0.2,
            (1 - y) * 1.2 + 0.4,
            (1 - y) * 1.5 + 0.2,
          );
          g.translate(Math.cos(angle) * reach, y, Math.sin(angle) * reach);
          cards.push(g);
        }
      }
    const result = mergeGeometries(cards);
    cards.forEach((g) => g.dispose());
    return result;
  }, []);
  useEffect(() => () => geometry.dispose(), [geometry]);
  useEffect(() => {
    diff.colorSpace = bark.colorSpace = THREE.SRGBColorSpace;
    for (const t of [diff, alpha, normal, bark]) {
      t.anisotropy = 8;
      t.needsUpdate = true;
    }
    const m = new THREE.Matrix4(),
      q = new THREE.Quaternion(),
      up = new THREE.Vector3(0, 1, 0);
    placements.forEach((p, i) => {
      q.setFromAxisAngle(up, p.rotation);
      m.compose(p.position, q, p.scale);
      leaves.current?.setMatrixAt(i, m);
      m.compose(
        p.position.clone().add(new THREE.Vector3(0, p.scale.y * 0.48, 0)),
        q,
        new THREE.Vector3(0.22, p.scale.y * 0.96, 0.22),
      );
      trunks.current?.setMatrixAt(i, m);
      leaves.current?.setColorAt(
        i,
        new THREE.Color().setHSL(
          0.21 + (i % 7) * 0.004,
          0.15,
          0.52 + (i % 5) * 0.025,
        ),
      );
    });
    for (const mesh of [leaves.current, trunks.current])
      if (mesh) {
        mesh.instanceMatrix.needsUpdate = true;
        if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
        mesh.computeBoundingSphere();
      }
  }, [placements, diff, alpha, normal, bark]);
  return (
    <group>
      <instancedMesh
        ref={trunks}
        args={[undefined, undefined, placements.length]}
        castShadow
        receiveShadow
      >
        <cylinderGeometry args={[0.22, 1, 1, 10]} />
        <meshStandardMaterial map={bark} roughness={1} />
      </instancedMesh>
      <instancedMesh
        ref={leaves}
        args={[geometry, undefined, placements.length]}
        castShadow
        receiveShadow
      >
        <meshStandardMaterial
          map={diff}
          alphaMap={alpha}
          alphaTest={0.4}
          normalMap={normal}
          normalScale={new THREE.Vector2(0.25, 0.25)}
          roughness={0.88}
          side={THREE.DoubleSide}
        />
      </instancedMesh>
    </group>
  );
}
