import { useTexture } from "@react-three/drei";
import { useEffect, useMemo, useRef } from "react";
import * as THREE from "three";
import { forestPlacements } from "./environmentGeometry";
import type { EnvironmentSession } from "./TrackEnvironment";
/** Generated decorative forest impostors; neither surveyed objects nor collision geometry. */
export default function ForestBillboards({
  session,
}: {
  session: EnvironmentSession;
}) {
  const mesh = useRef<THREE.InstancedMesh>(null),
    texture = useTexture("/assets/environment/forest-atlas-generated.png");
  const positions = useMemo(
    () => forestPlacements(session.geometry).filter((_, i) => i % 3 === 0),
    [session],
  );
  const geometry = useMemo(
    () => new THREE.PlaneGeometry(1, 1).translate(0, 0.5, 0),
    [],
  );
  const material = useMemo(() => {
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.anisotropy = 8;
    const m = new THREE.MeshBasicMaterial({
      map: texture,
      alphaTest: 0.3,
      side: THREE.DoubleSide,
      color: "#b6c3b1",
      fog: true,
    });
    m.onBeforeCompile = (shader) => {
      shader.vertexShader = shader.vertexShader.replace(
        "#include <project_vertex>",
        `
   vec3 center = (modelMatrix * instanceMatrix * vec4(0.0,0.0,0.0,1.0)).xyz;
   vec3 right = normalize(vec3(viewMatrix[0][0],0.0,viewMatrix[2][0]));
   vec2 size = vec2(length(instanceMatrix[0].xyz), length(instanceMatrix[1].xyz));
   vec4 mvPosition = viewMatrix * vec4(center + right * position.x * size.x + vec3(0.0,position.y * size.y,0.0),1.0);
   gl_Position = projectionMatrix * mvPosition;
  `,
      );
    };
    return m;
  }, [texture]);
  useEffect(
    () => () => {
      geometry.dispose();
      material.dispose();
    },
    [geometry, material],
  );
  useEffect(() => {
    const matrix = new THREE.Matrix4();
    positions.forEach((p, i) => {
      const h = p.scale.y;
      matrix.compose(
        p.position.clone().add(new THREE.Vector3(0, -h * 0.022, 0)),
        new THREE.Quaternion(),
        new THREE.Vector3(h, h, 1),
      );
      mesh.current?.setMatrixAt(i, matrix);
    });
    if (mesh.current) {
      mesh.current.instanceMatrix.needsUpdate = true;
      mesh.current.computeBoundingSphere();
    }
  }, [positions]);
  return (
    <instancedMesh ref={mesh} args={[geometry, material, positions.length]} />
  );
}
