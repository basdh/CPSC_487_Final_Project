import * as THREE from 'three';

/**
 * Fit perspective camera to scene contents (excluding optional helpers).
 */
export function frameCameraToScene(camera, controls, scene, options = {}) {
  const box = new THREE.Box3();
  const tmp = new THREE.Box3();

  scene.traverse((o) => {
    if (!o.isMesh) return;
    /** Skip huge floor plane / sky so the camera stays near the robot. */
    if (o.userData?.excludeFromFrame) return;
    const g = o.geometry;
    if (!g) return;
    if (!g.boundingBox) g.computeBoundingBox();
    tmp.copy(g.boundingBox).applyMatrix4(o.matrixWorld);
    box.union(tmp);
  });

  if (!isFinite(box.min.x) || box.isEmpty()) {
    box.setFromCenterAndSize(new THREE.Vector3(0, 1, 0), new THREE.Vector3(4, 4, 4));
  }

  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z, 0.5);
  const dist = maxDim * (options.distanceFactor ?? 1.35);

  const dir = new THREE.Vector3(0.55, 0.42, 1).normalize();
  camera.position.copy(center.clone().addScaledVector(dir, dist));
  camera.near = Math.max(0.01, dist / 200);
  camera.far = Math.max(500, dist * 50);
  camera.updateProjectionMatrix();

  controls.target.copy(center);
  controls.update();
}

/**
 * Thick RGB arrows at origin (always visible; complements AxesHelper).
 */
/**
 * Aim camera at one object (e.g. GLB root) — avoids huge floor planes skewing the view.
 */
export function frameCameraToObject(camera, controls, object, margin = 1.85) {
  const box = new THREE.Box3().setFromObject(object);
  if (!isFinite(box.min.x) || box.isEmpty()) return;
  const c = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const half = Math.max(size.x, size.y, size.z) * 0.5;
  const dist = Math.max(half * margin, 0.35);
  camera.position.set(c.x + dist * 0.75, c.y + dist * 0.42, c.z + dist * 0.9);
  camera.near = 0.02;
  camera.far = Math.max(250, dist * 50);
  camera.updateProjectionMatrix();
  controls.target.copy(c);
  controls.update();
}

export function makeOriginArrows(scene, length = 2.2) {
  const g = new THREE.Group();
  g.name = 'origin_arrows';
  const x = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 0.01, 0),
    length,
    0xff3333,
    length * 0.12,
    length * 0.08
  );
  const y = new THREE.ArrowHelper(
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(0, 0.01, 0),
    length,
    0x33ff66,
    length * 0.12,
    length * 0.08
  );
  const z = new THREE.ArrowHelper(
    new THREE.Vector3(0, 0, 1),
    new THREE.Vector3(0, 0.01, 0),
    length,
    0x3388ff,
    length * 0.12,
    length * 0.08
  );
  g.add(x, y, z);
  scene.add(g);
  return g;
}
