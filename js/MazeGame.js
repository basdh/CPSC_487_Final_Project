import * as THREE from 'three';
import { OrbitControls } from 'https://unpkg.com/three@0.160.0/examples/jsm/controls/OrbitControls.js';
import { RobotArm } from './RobotArm.js';
import { OptimizationController } from './OptimizationController.js';
import { stripProceduralMeshes } from './XArmGLTF.js';
import {
  loadKinematicGLTF,
  getKinematicBinding,
  DEFAULT_GLTF_JOINT_AXES,
} from './XArmKinematicGLTF.js';
import {
  generateVerifiedMaze,
  hasPathThroughMaze,
  createRng,
  mazeToWallRects,
  cellCenterWorld,
} from './mazeGenerator.js';

function circleRectOverlap(px, pz, r, rect) {
  const qx = THREE.MathUtils.clamp(px, rect.minX, rect.maxX);
  const qz = THREE.MathUtils.clamp(pz, rect.minZ, rect.maxZ);
  const dx = px - qx;
  const dz = pz - qz;
  return dx * dx + dz * dz < r * r;
}

/**
 * Maze mini-game: perfect grid maze (recursive-backtracker / randomized DFS); xArm GLB arms,
 * same IK as lab, reach toward the invisible player position when you get close.
 */
export class MazeGame {
  /**
   * @param {{
   *   renderer: THREE.WebGLRenderer;
   *   container: HTMLElement;
   *   onExit: () => void;
   *   getMainControls?: () => { enabled: boolean } | null | undefined;
   *   modelUrl?: string;
   * }} opts
   */
  constructor(opts) {
    this.renderer = opts.renderer;
    this.container = opts.container;
    this.onExit = opts.onExit;
    this.getMainControls = opts.getMainControls || (() => null);
    this.modelUrl = opts.modelUrl || './assets/xarm.glb';

    this.isActive = false;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0a0f18);

    this.camera = new THREE.PerspectiveCamera(48, 1, 0.08, 140);
    this.camera.position.set(0, 12, 10);

    this.cols = 8;
    this.rows = 8;
    this.cellSize = 1.12;
    this.playerRadius = 0.16;
    this.wallThickness = 0.048;
    this.captureRadius = 0.21;
    /** Arms only run IK when player is within this horizontal distance (m). */
    this.armActivationRadius = 2.45;
    /** Lift soles slightly above the floor mesh after bbox snap (m). */
    this.armFloorClearance = 0.015;
    /** Loaded once; clones placed per arm (never added to scene). */
    this.armTemplateKin = null;

    /** @type {{ robot: RobotArm; opt: OptimizationController; baseXZ: THREE.Vector2 }[]} */
    this.arms = [];
    /** @type {{ minX:number,maxX:number,minZ:number,maxZ:number }[]} */
    this.wallRects = [];
    this.level = 0;
    this.lastSeed = 1;

    this.playerPos = new THREE.Vector3();
    this.exitWorld = new THREE.Vector3();
    this.startWorld = new THREE.Vector3();

    this.keys = { up: false, down: false, left: false, right: false };

    /** @type {THREE.Group | null} */
    this.mazeGroup = null;
    /** @type {THREE.Mesh | null} */
    this.playerMesh = null;
    /** @type {THREE.Mesh | null} */
    this.exitMesh = null;

    this.staticReady = false;

    /** @type {OrbitControls | null} */
    this.orbitControls = null;

    /** True while async level build runs — blocks re-entrant win/lose (exit world pos is level-invariant). */
    this._transitioning = false;

    /** Low-pass filtered aiming point for arms (lags the player for sluggish motion). */
    this._armIkTarget = new THREE.Vector3();

    this._boundKeyDown = (e) => this._onKey(e, true);
    this._boundKeyUp = (e) => this._onKey(e, false);
  }

  _setupOrbitControls() {
    if (this.orbitControls) {
      this.orbitControls.dispose();
      this.orbitControls = null;
    }
    const oc = new OrbitControls(this.camera, this.renderer.domElement);
    oc.enableDamping = true;
    oc.dampingFactor = 0.07;
    oc.minDistance = 6.5;
    oc.maxDistance = 32;
    oc.minPolarAngle = 0.18;
    oc.maxPolarAngle = Math.PI / 2 - 0.06;
    oc.enablePan = false;
    oc.enableRotate = false;
    oc.rotateSpeed = 0;
    oc.zoomSpeed = 0.85;
    this.orbitControls = oc;
  }

  /**
   * Fixed viewpoint from the maze **north** (+Z): centered on the north rim, looking south.
   * With grid (0,0)=start at min x,z and exit at max x,z, this puts **start top-left** and
   * **exit bottom-right** on screen. Zoom only changes distance (no orbit rotation).
   */
  _syncFixedCamera(instant) {
    if (!this.orbitControls) return;
    const halfH = (this.rows * this.cellSize) / 2;
    const target = new THREE.Vector3(0, 0.14, 0);
    if (instant) {
      this.orbitControls.target.copy(target);
      const northStandoff = 9.2;
      this.camera.position.set(0, 12.4, halfH + northStandoff);
      this.camera.lookAt(target);
    }
    this.orbitControls.update();
  }

  _buildStaticScene() {
    const ambient = new THREE.AmbientLight(0xffffff, 0.55);
    const hemi = new THREE.HemisphereLight(0xbcdcff, 0x1a1f28, 0.9);
    const dir = new THREE.DirectionalLight(0xffffff, 1.05);
    dir.position.set(8, 20, 12);
    dir.castShadow = true;
    this.scene.add(ambient);
    this.scene.add(hemi);
    this.scene.add(dir);

    const floorSz = 46;
    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(floorSz, floorSz),
      new THREE.MeshStandardMaterial({
        color: 0x1e293b,
        roughness: 0.85,
        metalness: 0.05,
      })
    );
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = 0.01;
    floor.receiveShadow = true;
    this.scene.add(floor);
  }

  _onKey(e, down) {
    if (!this.isActive) return;
    const c = e.code;
    if (c === 'ArrowUp') {
      this.keys.up = down;
      if (down) e.preventDefault();
    }
    if (c === 'ArrowDown') {
      this.keys.down = down;
      if (down) e.preventDefault();
    }
    if (c === 'ArrowLeft') {
      this.keys.left = down;
      if (down) e.preventDefault();
    }
    if (c === 'ArrowRight') {
      this.keys.right = down;
      if (down) e.preventDefault();
    }
  }

  _setHud(html) {
    const el = document.getElementById('game-hud');
    if (el) el.innerHTML = html;
  }

  _showOverlay(show) {
    const el = document.getElementById('game-overlay');
    if (el) {
      el.style.display = show ? 'flex' : 'none';
      if (show) el.style.flexDirection = 'column';
    }
  }

  async enter() {
    this.isActive = true;
    this.lastSeed = (Date.now() % 100000) + 17;
    this.level = 0;
    if (!this.staticReady) {
      this._buildStaticScene();
      this.staticReady = true;
    }

    const mc = this.getMainControls();
    if (mc) mc.enabled = false;

    this._setupOrbitControls();
    await this._buildLevel();

    if (!this.armTemplateKin) {
      this._setHud(
        '<span style="color:#f87171">Could not load xArm GLB.</span> Add <code>assets/xarm.glb</code> or set <strong>glTF URL</strong> in the lab, then open the maze again.'
      );
    } else {
      this._setHud(
        this._hudText(
          'Reach the <span style="color:#4ade80">green exit</span>. Walls stop you but do not reset the level. Arms wake when you get close. More arms each level.'
        )
      );
    }
    this._showOverlay(true);

    document.querySelectorAll('.lil-gui').forEach((el) => {
      el.style.display = 'none';
    });
    const ms = document.getElementById('model-status');
    if (ms) ms.style.display = 'none';
    const ax = document.getElementById('axes-legend');
    if (ax) ax.style.display = 'none';

    window.addEventListener('keydown', this._boundKeyDown);
    window.addEventListener('keyup', this._boundKeyUp);
    this.onResize();
  }

  exit() {
    if (!this.isActive) return;
    this.isActive = false;
    window.removeEventListener('keydown', this._boundKeyDown);
    window.removeEventListener('keyup', this._boundKeyUp);
    this._clearDynamicContent();

    if (this.orbitControls) {
      this.orbitControls.dispose();
      this.orbitControls = null;
    }
    const mc = this.getMainControls();
    if (mc) mc.enabled = true;

    this._showOverlay(false);

    document.querySelectorAll('.lil-gui').forEach((el) => {
      el.style.display = '';
    });
    const ms = document.getElementById('model-status');
    if (ms) ms.style.display = '';
    const ax = document.getElementById('axes-legend');
    if (ax) ax.style.display = '';

    this.onExit();
  }

  _disposeRobot(robot) {
    if (robot.gltfRoot) {
      this.scene.remove(robot.gltfRoot);
      robot.gltfRoot.traverse((o) => {
        if (o.geometry) o.geometry.dispose?.();
        if (o.material) {
          if (Array.isArray(o.material)) o.material.forEach((m) => m.dispose?.());
          else o.material.dispose?.();
        }
      });
      robot.detachGltfKinematic();
      robot.proceduralGripTip = null;
    }
    for (const link of robot.links) {
      this.scene.remove(link.mesh);
      link.mesh.traverse((o) => {
        if (o.geometry) o.geometry.dispose?.();
        if (o.material) {
          if (Array.isArray(o.material)) o.material.forEach((m) => m.dispose?.());
          else o.material.dispose?.();
        }
      });
    }
  }

  _clearDynamicContent() {
    for (const a of this.arms) {
      this._disposeRobot(a.robot);
    }
    this.arms.length = 0;
    if (this.mazeGroup) {
      this.scene.remove(this.mazeGroup);
      this.mazeGroup.traverse((o) => {
        if (o.isMesh) {
          o.geometry?.dispose();
          if (o.material) {
            if (Array.isArray(o.material)) o.material.forEach((m) => m.dispose?.());
            else o.material.dispose();
          }
        }
      });
      this.mazeGroup = null;
    }
    if (this.exitMesh) {
      this.scene.remove(this.exitMesh);
      this.exitMesh.geometry.dispose();
      this.exitMesh.material.dispose();
      this.exitMesh = null;
    }
    if (this.playerMesh) {
      this.scene.remove(this.playerMesh);
      this.playerMesh.geometry.dispose();
      this.playerMesh.material.dispose();
      this.playerMesh = null;
    }
  }

  _hudText(msg) {
    return `
      <div class="game-hud-title">Maze challenge</div>
      <div class="game-hud-level">Level <strong>${this.level}</strong></div>
      <p class="game-hud-body">${msg}</p>
      <p class="game-hud-keys">Arrows: move (world axes) · Scroll: zoom only · <kbd>Esc</kbd>: lab</p>
    `;
  }

  /** Random interior cells (not start/end); spreads bases when possible, up to {@link MazeGame#armCount} slots. */
  _pickArmPlacements(rng) {
    const cands = [];
    for (let z = 0; z < this.rows; z++) {
      for (let x = 0; x < this.cols; x++) {
        if (x === 0 && z === 0) continue;
        if (x === this.cols - 1 && z === this.rows - 1) continue;
        cands.push([x, z]);
      }
    }
    for (let i = cands.length - 1; i > 0; i--) {
      const j = Math.floor(rng() * (i + 1));
      [cands[i], cands[j]] = [cands[j], cands[i]];
    }
    const halfW = (this.cols * this.cellSize) / 2;
    const halfH = (this.rows * this.cellSize) / 2;
    const out = [];
    const used = new Set();
    const minD = 0.4 * this.cellSize;
    for (const [cx, cz] of cands) {
      if (out.length >= this.armCount) break;
      const wx = (cx + 0.5) * this.cellSize - halfW + (rng() - 0.5) * 0.26;
      const wz = (cz + 0.5) * this.cellSize - halfH + (rng() - 0.5) * 0.26;
      if (out.every((p) => Math.hypot(wx - p.x, wz - p.z) >= minD)) {
        out.push({ x: wx, z: wz });
        used.add(`${cx},${cz}`);
      }
    }
    for (const [cx, cz] of cands) {
      if (out.length >= this.armCount) break;
      if (used.has(`${cx},${cz}`)) continue;
      out.push({
        x: (cx + 0.5) * this.cellSize - halfW,
        z: (cz + 0.5) * this.cellSize - halfH,
      });
      used.add(`${cx},${cz}`);
    }
    return out;
  }

  /**
   * Approximate full-arm collision by testing player sphere against each visible mesh sphere.
   * This avoids end-effector-only misses without requiring expensive mesh-distance queries.
   */
  _armTouchesPlayer(arm, playerRadius) {
    const tmpPos = new THREE.Vector3();
    const tmpScale = new THREE.Vector3();
    for (const c of arm.colliders) {
      c.mesh.getWorldPosition(tmpPos);
      c.mesh.getWorldScale(tmpScale);
      const s = Math.max(tmpScale.x, tmpScale.y, tmpScale.z);
      const r = c.radius * s;
      if (tmpPos.distanceTo(this.playerPos) <= r + playerRadius) return true;
    }
    return false;
  }

  _easeRobotToRest(robot, opt, dt) {
    const rest = robot.thetaRest || [0, 0, 0, 0];
    const k = 1 - Math.exp(-dt * 1.35);
    const next = robot.theta.map((t, i) => THREE.MathUtils.lerp(t, rest[i] ?? 0, k));
    robot.setJointAngles(next);
    robot.setGripperGoal(0);
    robot.forwardKinematics();
    robot.updateMeshes();
    // Keep optimizer state continuous; do not hard-resync each frame or it can appear to snap.
    opt.thetaPrev = next.slice();
  }

  async _buildLevel() {
    this._clearDynamicContent();
    this.wallRects = [];

    const sc0 = cellCenterWorld(0, 0, this.cols, this.rows, this.cellSize);
    this.startWorld.set(sc0.x, sc0.y, sc0.z);
    this.playerPos.set(sc0.x, sc0.y, sc0.z);
    this._armIkTarget.set(sc0.x, sc0.y + 0.02, sc0.z);
    const ec0 = cellCenterWorld(this.cols - 1, this.rows - 1, this.cols, this.rows, this.cellSize);
    this.exitWorld.set(ec0.x, 0.14, ec0.z);

    const rng = createRng(this.lastSeed);
    const maxArmSlots = this.cols * this.rows - 2;
    this.armCount = Math.min(this.level + 1, maxArmSlots);

    const maze = generateVerifiedMaze(this.cols, this.rows, this.lastSeed);
    console.assert(
      hasPathThroughMaze(maze, 0, 0, this.cols - 1, this.rows - 1) === true,
      'hasPathThroughMaze(maze, 0, 0, cols - 1, rows - 1) must be true before rendering'
    );
    this.wallRects = mazeToWallRects(maze, this.cellSize, this.wallThickness);

    const armPlacements = this._pickArmPlacements(rng);

    if (!this.armTemplateKin) {
      const kin = await loadKinematicGLTF(this.modelUrl, {
        targetHeight: 0.76,
        ctrlNames: ['ctrl_1', 'ctrl_2', 'ctrl_3', 'ctrl_4'],
        endEffectorHints: ['grabber', 'polySurface3', 'tcp', 'tool0', 'EE', 'end_effector'],
      });
      if (kin) this.armTemplateKin = kin;
      else console.error('[MazeGame] GLB load failed:', this.modelUrl);
    }

    this.mazeGroup = new THREE.Group();
    this.scene.add(this.mazeGroup);

    const wallH = 0.42;
    const wallMat = new THREE.MeshStandardMaterial({
      color: 0x7f1d1d,
      roughness: 0.55,
      metalness: 0.1,
    });

    for (const r of this.wallRects) {
      const wx = (r.minX + r.maxX) / 2;
      const wz = (r.minZ + r.maxZ) / 2;
      const dx = Math.max(0.02, r.maxX - r.minX);
      const dz = Math.max(0.02, r.maxZ - r.minZ);
      const g = new THREE.BoxGeometry(dx, wallH, dz);
      const m = new THREE.Mesh(g, wallMat);
      m.position.set(wx, wallH / 2, wz);
      m.castShadow = true;
      m.receiveShadow = true;
      this.mazeGroup.add(m);
    }

    this.exitMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(0.32, 0.32, 0.12, 24),
      new THREE.MeshStandardMaterial({
        color: 0x22c55e,
        emissive: 0x0d4a2a,
        emissiveIntensity: 0.5,
        metalness: 0.2,
        roughness: 0.4,
      })
    );
    this.exitMesh.position.copy(this.exitWorld);
    this.exitMesh.position.y = 0.08;
    this.scene.add(this.exitMesh);

    this.playerMesh = new THREE.Mesh(
      new THREE.SphereGeometry(this.playerRadius, 20, 20),
      new THREE.MeshStandardMaterial({
        color: 0x38bdf8,
        emissive: 0x0c4a6e,
        emissiveIntensity: 0.4,
        metalness: 0.15,
        roughness: 0.4,
      })
    );
    this.playerMesh.position.copy(this.playerPos);
    this.playerMesh.castShadow = true;
    this.scene.add(this.playerMesh);

    const tpl = this.armTemplateKin;
    if (tpl) {
      const axes = [...DEFAULT_GLTF_JOINT_AXES];
      for (const place of armPlacements) {
        const { x, z } = place;
        const root = tpl.root.clone(true);
        const { ctrlNodes, eeObject } = getKinematicBinding(root);
        const bp = tpl.root.position;
        root.position.copy(bp);
        root.position.x += x;
        root.position.z += z;
        const yaw =
          Math.atan2(-x, -z) + (rng() - 0.5) * 0.4;
        root.rotation.y = yaw;
        root.scale.copy(tpl.root.scale).multiplyScalar(0.92);
        root.updateMatrixWorld(true);
        const bbox = new THREE.Box3().setFromObject(root);
        root.position.y += -bbox.min.y + this.armFloorClearance;
        this.scene.add(root);

        const robot = new RobotArm();
        robot.sceneRef = this.scene;
        robot.buildRobot(this.scene, { meshStyle: 'bones' });
        stripProceduralMeshes(robot);
        robot.attachGltfKinematic(root, ctrlNodes, eeObject, axes);
        robot.setJointAngles([0, 0, 0, 0]);
        robot.forwardKinematics();
        robot.updateMeshes();

        const opt = new OptimizationController(robot);
        opt.syncFilteredFromRobot();
        opt.thetaPrev = robot.theta.slice();
        opt.optimizationEnabled = true;
        opt.learningRate = 0.046;
        opt.gradientEpsilon = 0.034;
        opt.useCentralDifferences = true;
        opt.gradientClip = 22;
        opt.jointOutputSmoothing = 0.21;
        opt.targetWeight = 6.1;
        opt.smoothnessWeight = 0.36;
        opt.restPoseWeight = 0.05;
        opt.jointLimitsEnabled = false;

        const baseXZ = new THREE.Vector2(root.position.x, root.position.z);
        /** @type {{ mesh: THREE.Mesh, radius: number }[]} */
        const colliders = [];
        root.traverse((o) => {
          if (!o.isMesh || !o.geometry) return;
          if (!o.geometry.boundingSphere) o.geometry.computeBoundingSphere();
          const bs = o.geometry.boundingSphere;
          if (!bs) return;
          colliders.push({ mesh: o, radius: bs.radius });
        });
        this.arms.push({
          robot,
          opt,
          baseXZ,
          colliders,
          wasInRange: false,
          engage: 0,
          baseLearningRate: opt.learningRate,
        });
      }
    }

    this._syncFixedCamera(true);
  }

  async _lose() {
    if (this._transitioning) return;
    this._transitioning = true;
    try {
      this.level = 0;
      this.lastSeed = (Date.now() % 200000) + Math.floor(Math.random() * 1e6);
      await this._buildLevel();
      this._setHud(
        this._hudText(
          '<span style="color:#f87171">Arm touch!</span> Level reset to <strong>0</strong> — new maze.'
        )
      );
    } finally {
      this._transitioning = false;
    }
  }

  async _win() {
    if (this._transitioning) return;
    this._transitioning = true;
    try {
      this.level += 1;
      this.lastSeed = this.level * 486187 + (Date.now() % 100000);
      await this._buildLevel();
      this._setHud(
        this._hudText(
          '<span style="color:#4ade80">Exit cleared!</span> Now at <strong>level ' +
            this.level +
            '</strong> — new random maze.'
        )
      );
    } finally {
      this._transitioning = false;
    }
  }

  /**
   * Axes match the north-edge camera: on screen, **up** is −world Z (toward start / top of board),
   * **down** is +Z (toward exit), **left** is −X, **right** is +X.
   */
  _worldMoveAxes() {
    const forward = new THREE.Vector3(0, 0, -1);
    const right = new THREE.Vector3(1, 0, 0);
    return { forward, right };
  }

  update(dt) {
    if (!this.isActive) return;

    if (this._transitioning) {
      if (this.orbitControls) this.orbitControls.update();
      return;
    }

    const speed = 3.4;
    const { forward, right } = this._worldMoveAxes();
    const move = new THREE.Vector3();
    if (this.keys.up) move.add(forward);
    if (this.keys.down) move.sub(forward);
    if (this.keys.right) move.add(right);
    if (this.keys.left) move.sub(right);
    if (move.lengthSq() > 1e-8) {
      move.normalize().multiplyScalar(speed * dt);
      const nx = this.playerPos.x + move.x;
      const nz = this.playerPos.z + move.z;
      if (!this._hitsWall(nx, nz)) {
        this.playerPos.x = nx;
        this.playerPos.z = nz;
      }
    }

    if (this.playerMesh) this.playerMesh.position.copy(this.playerPos);

    const rawAim = new THREE.Vector3(this.playerPos.x, this.playerPos.y + 0.02, this.playerPos.z);
    const aimSmooth = 1 - Math.exp(-dt * 3.8);
    this._armIkTarget.lerp(rawAim, aimSmooth);
    const target = this._armIkTarget;

    for (const arm of this.arms) {
      const { robot, opt, baseXZ } = arm;
      const dPlan = Math.hypot(
        this.playerPos.x - baseXZ.x,
        this.playerPos.z - baseXZ.y
      );
      const inRange = dPlan <= this.armActivationRadius;
      if (!inRange) {
        arm.wasInRange = false;
        arm.engage = 0;
        this._easeRobotToRest(robot, opt, dt);
        continue;
      }

      if (!arm.wasInRange) {
        // Prevent first-frame activation jump: start optimizer from the current eased pose.
        opt.syncFilteredFromRobot();
        opt.thetaPrev = robot.theta.slice();
        arm.engage = 0;
        arm.wasInRange = true;
      }

      // Smoothly ramp tracking authority after entering activation radius.
      arm.engage = Math.min(1, arm.engage + dt * 3.2);
      const lrPrev = opt.learningRate;
      opt.learningRate = arm.baseLearningRate * (0.22 + 0.78 * arm.engage);
      opt.thetaPrev = robot.theta.slice();
      opt.step(robot.theta, target, robot.thetaRest, false);
      opt.learningRate = lrPrev;
      robot.setGripperGoal(1);
      robot.updateMeshes();
      const pR = this.playerRadius * 0.96;
      if (this._armTouchesPlayer(arm, pR)) {
        void this._lose();
        return;
      }
    }

    const distExit = new THREE.Vector2(this.playerPos.x - this.exitWorld.x, this.playerPos.z - this.exitWorld.z).length();
    if (distExit < this.cellSize * 0.38) {
      void this._win();
    }

    if (this.orbitControls) this.orbitControls.update();
  }

  _hitsWall(px, pz) {
    const r = this.playerRadius * 0.92;
    for (const rect of this.wallRects) {
      if (circleRectOverlap(px, pz, r, rect)) return true;
    }
    return false;
  }

  render() {
    this.renderer.render(this.scene, this.camera);
  }

  onResize() {
    if (!this.renderer || !this.camera) return;
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    this.camera.aspect = w / Math.max(h, 1);
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }
}
