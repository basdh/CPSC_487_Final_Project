import * as THREE from 'three';

/**
 * A chain of spheres connected by fixed-length segments (FABRIK-style reach).
 * Mirrors pointer modes: autonomous sway, reach toward plane hit, avoid pointer.
 */
export class SphereChain {
  /**
   * @param {THREE.Scene} scene
   * @param {object} opts
   */
  constructor(scene, opts = {}) {
    this.scene = scene;
    this.n = opts.segments ?? 24;
    this.restLen = opts.restLength ?? 0.11;
    this.origin = opts.origin ?? new THREE.Vector3(-2.4, 0.35, 0.45);
    /** World-space positions p[0]..p[n] */
    this.p = [];
    for (let i = 0; i <= this.n; i++) {
      const t = i / this.n;
      this.p.push(
        new THREE.Vector3(
          this.origin.x + t * 0.5,
          this.origin.y + t * 1.4,
          this.origin.z + t * 0.2
        )
      );
    }

    this.group = new THREE.Group();
    this.group.name = 'sphere_chain';
    scene.add(this.group);

    const matBall = new THREE.MeshStandardMaterial({
      color: 0xa78bfa,
      metalness: 0.15,
      roughness: 0.45,
      emissive: 0x1e1b4b,
      emissiveIntensity: 0.35,
    });
    this.spheres = [];
    const r = 0.055;
    for (let i = 0; i <= this.n; i++) {
      const m = new THREE.Mesh(new THREE.SphereGeometry(r, 14, 14), matBall.clone());
      m.userData.chainIndex = i;
      m.userData.pickable = true;
      this.group.add(m);
      this.spheres.push(m);
    }
    this.spheres[0].material.color.setHex(0xf472b6);
    this.spheres[this.n].material.color.setHex(0xfbbf24);

    const lineGeom = new THREE.BufferGeometry();
    const lineMat = new THREE.LineBasicMaterial({
      color: 0xc4b5fd,
      transparent: true,
      opacity: 0.85,
    });
    this.line = new THREE.Line(lineGeom, lineMat);
    this.group.add(this.line);

    this.tmp = new THREE.Vector3();
    /** Selected mesh highlight */
    this.selectedIndex = -1;
  }

  setSelected(i) {
    this.selectedIndex = i;
    this.spheres.forEach((s, k) => {
      const base = k === 0 ? 0xf472b6 : k === this.n ? 0xfbbf24 : 0xa78bfa;
      s.material.color.setHex(base);
      s.material.emissiveIntensity = k === i ? 0.9 : 0.35;
    });
    if (i >= 0 && this.spheres[i]) {
      this.spheres[i].material.color.setHex(0xffffff);
    }
  }

  updateLine() {
    const arr = new Float32Array((this.n + 1) * 3);
    for (let i = 0; i <= this.n; i++) {
      arr[i * 3] = this.p[i].x;
      arr[i * 3 + 1] = this.p[i].y;
      arr[i * 3 + 2] = this.p[i].z;
    }
    this.line.geometry.setAttribute('position', new THREE.BufferAttribute(arr, 3));
    this.line.geometry.setDrawRange(0, this.n + 1);
  }

  syncMeshes() {
    for (let i = 0; i <= this.n; i++) {
      this.spheres[i].position.copy(this.p[i]);
    }
    this.updateLine();
  }

  /**
   * FABRIK: satisfy segment lengths between fixed root and target tip.
   */
  fabrik(targetTip, iterations = 14) {
    const L = this.restLen;
    const n = this.n;
    const root = this.origin;

    for (let it = 0; it < iterations; it++) {
      this.p[n].copy(targetTip);
      for (let i = n - 1; i >= 0; i--) {
        this.tmp.subVectors(this.p[i], this.p[i + 1]);
        if (this.tmp.lengthSq() < 1e-12) this.tmp.set(1, 0, 0);
        this.tmp.normalize().multiplyScalar(L);
        this.p[i].copy(this.p[i + 1]).add(this.tmp);
      }
      this.p[0].copy(root);
      for (let i = 1; i <= n; i++) {
        this.tmp.subVectors(this.p[i], this.p[i - 1]);
        if (this.tmp.lengthSq() < 1e-12) this.tmp.set(1, 0, 0);
        this.tmp.normalize().multiplyScalar(L);
        this.p[i].copy(this.p[i - 1]).add(this.tmp);
      }
    }
    this.syncMeshes();
  }

  /**
   * @param {'No Reaction'|'Reach Pointer'|'Avoid Pointer'} mode
   * @param {THREE.Vector3} pointerWorld
   * @param {number} u — spline phase [0,1)
   * @param {number} dt
   * @param {object} avoidOpts — { safeRadius, strength }
   */
  update(mode, pointerWorld, u, dt, avoidOpts = {}) {
    const tip = new THREE.Vector3();
    const base = this.origin;

    if (mode === 'No Reaction') {
      const ang = u * Math.PI * 2;
      tip.set(
        base.x + Math.cos(ang) * 1.1 + 0.4,
        base.y + 0.85 + Math.sin(ang * 2) * 0.25,
        base.z + Math.sin(ang) * 0.45
      );
    } else if (mode === 'Reach Pointer') {
      tip.copy(pointerWorld);
    } else {
      /** Push tip outward from the pointer (same spirit as softplus avoid on the arm). */
      const rs = avoidOpts.safeRadius ?? 0.55;
      const away = new THREE.Vector3().subVectors(base, pointerWorld);
      if (away.lengthSq() < 1e-6) away.set(1, 0.4, 0);
      away.normalize();
      tip.copy(pointerWorld).addScaledVector(away, rs + 0.5);
    }

    this.fabrik(tip);
  }

  getPickables() {
    return this.spheres;
  }
}
