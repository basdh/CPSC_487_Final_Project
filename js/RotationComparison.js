import * as THREE from 'three';

/**
 * Side-by-side rotation interpolation comparison (SO(3) ideas).
 *
 * “Euler interpolation is component-wise and can produce unintuitive rotation paths.”
 * “Quaternion SLERP interpolates on S^3 and better respects SO(3) geometry.”
 *
 * Conceptually related to a * exp(t log(a^{-1} b)); quaternions implement the geodesic
 * interpolation via THREE.Quaternion.slerpQuaternions(q0, q1, u).
 */
export class RotationComparison {
  constructor(scene) {
    this.scene = scene;
    this.group = new THREE.Group();
    this.group.position.set(-3.2, 1.35, 0);
    scene.add(this.group);

    const mkCube = () => {
      const g = new THREE.BoxGeometry(0.35, 0.35, 0.35);
      const m = new THREE.MeshStandardMaterial({
        color: 0xaaaaaa,
        metalness: 0.2,
        roughness: 0.45,
      });
      const mesh = new THREE.Mesh(g, m);
      mesh.castShadow = true;
      return mesh;
    };

    this.cubeEuler = mkCube();
    this.cubeQuat = mkCube();
    this.cubeEuler.position.set(-0.55, 0, 0);
    this.cubeQuat.position.set(0.55, 0, 0);

    const axesE = new THREE.AxesHelper(0.45);
    const axesQ = new THREE.AxesHelper(0.45);
    this.cubeEuler.add(axesE);
    this.cubeQuat.add(axesQ);

    this.group.add(this.cubeEuler);
    this.group.add(this.cubeQuat);

    this.qStart = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 0, 0));
    this.qEnd = new THREE.Quaternion().setFromEuler(
      new THREE.Euler(1.1, 0.9, -0.7, 'XYZ')
    );
    this.eulerStart = new THREE.Euler(0, 0, 0, 'XYZ');
    this.eulerEnd = new THREE.Euler(1.1, 0.9, -0.7, 'XYZ');

    /** Ghost meshes */
    this.ghostEuler = [];
    this.ghostQuat = [];
    const ghostMat = new THREE.MeshStandardMaterial({
      color: 0x888888,
      transparent: true,
      opacity: 0.22,
      depthWrite: false,
    });
    /** Stagger ghosts in depth so they do not z-fight with the main cubes (was causing full-scene flicker). */
    for (let i = 0; i < 7; i++) {
      const ge = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.22, 0.22), ghostMat.clone());
      const gq = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.22, 0.22), ghostMat.clone());
      ge.visible = false;
      gq.visible = false;
      ge.renderOrder = 1;
      gq.renderOrder = 1;
      if (ge.material) ge.material.depthWrite = false;
      if (gq.material) gq.material.depthWrite = false;
      this.group.add(ge);
      this.group.add(gq);
      const dz = 0.052 * (i + 1);
      ge.position.set(this.cubeEuler.position.x, this.cubeEuler.position.y, dz);
      gq.position.set(this.cubeQuat.position.x, this.cubeQuat.position.y, dz);
      this.ghostEuler.push(ge);
      this.ghostQuat.push(gq);
    }

    this.showGhosts = true;
    this.showTrail = false;
    this.time = 0;

    /** Keep demo geometry out of scene picks / camera framing — avoids raycast + orbit weirdness when toggled on. */
    this.group.traverse((o) => {
      o.userData.excludeFromPick = true;
      o.userData.excludeFromFrame = true;
    });
  }

  update(dt, showComparison, showGhosts, showTrail) {
    if (!showComparison) {
      this.group.visible = false;
      return;
    }
    this.group.visible = true;
    this.showGhosts = showGhosts;
    this.showTrail = showTrail;
    this.time += dt * 0.35;
    const u = (Math.sin(this.time) * 0.5 + 0.5) % 1;

    // Euler: naive component-wise interpolation of Euler angles
    const e = new THREE.Euler(
      THREE.MathUtils.lerp(this.eulerStart.x, this.eulerEnd.x, u),
      THREE.MathUtils.lerp(this.eulerStart.y, this.eulerEnd.y, u),
      THREE.MathUtils.lerp(this.eulerStart.z, this.eulerEnd.z, u),
      'XYZ'
    );
    this.cubeEuler.quaternion.setFromEuler(e);

    // Quaternion SLERP on the unit sphere S^3
    THREE.Quaternion.slerpQuaternions(this.qStart, this.qEnd, u, this.cubeQuat.quaternion);

    // Ghost poses along the path
    for (let i = 0; i < this.ghostEuler.length; i++) {
      const s = (i + 1) / (this.ghostEuler.length + 1);
      const ue = new THREE.Euler(
        THREE.MathUtils.lerp(this.eulerStart.x, this.eulerEnd.x, s),
        THREE.MathUtils.lerp(this.eulerStart.y, this.eulerEnd.y, s),
        THREE.MathUtils.lerp(this.eulerStart.z, this.eulerEnd.z, s),
        'XYZ'
      );
      this.ghostEuler[i].quaternion.setFromEuler(ue);
      this.ghostEuler[i].visible = showGhosts;

      const qq = new THREE.Quaternion().slerpQuaternions(this.qStart, this.qEnd, s);
      this.ghostQuat[i].quaternion.copy(qq);
      this.ghostQuat[i].visible = showGhosts;
    }
  }
}
