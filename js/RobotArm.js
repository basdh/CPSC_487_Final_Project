import * as THREE from 'three';
import { Link } from './Link.js';
import { Joint } from './Joint.js';

/** @param {'boxes'|'bones'} style */
function makeArmSegmentGroup(L, linkThickness, color, mkMat, style) {
  const g = new THREE.Group();
  if (style === 'bones') {
    const rS = linkThickness * 1.35;
    const rC = linkThickness * 0.55;
    const mat = mkMat(color);
    const sph = new THREE.Mesh(new THREE.SphereGeometry(rS, 16, 16), mat);
    sph.castShadow = true;
    const cyl = new THREE.Mesh(new THREE.CylinderGeometry(rC, rC, L, 14), mat);
    cyl.rotation.z = Math.PI / 2;
    cyl.position.set(L / 2, 0, 0);
    cyl.castShadow = true;
    g.add(sph, cyl);
  } else {
    const armGeo = new THREE.BoxGeometry(L, linkThickness, linkThickness);
    const mesh = new THREE.Mesh(armGeo, mkMat(color));
    mesh.castShadow = true;
    mesh.position.set(L / 2, 0, 0);
    g.add(mesh);
  }
  return g;
}

/**
 * RobotArm — serial chain with explicit forward kinematics.
 *
 * “The robot’s joint angles are the configuration variables. Forward kinematics maps
 * these variables to world-space poses for every link.” (Unit 4.1)
 *
 * “Each link pose is represented as a homogeneous transform combining rotation and translation.” (SE(3))
 */
export class RobotArm {
  constructor() {
    /** @type {Link[]} */
    this.links = [];
    /** @type {Joint[]} */
    this.joints = [];
    /** @type {Link | null} */
    this.rootLink = null;
    /** @type {Link | null} */
    this.endEffectorLink = null;
    /** World-space offset from end-effector link frame origin to tool tip (along +X). */
    this.endEffectorTipOffset = new THREE.Vector3();
    /** Joint angles [theta1..theta4] — matches joints order */
    this.theta = [0, 0, 0, 0];
    /** Rest pose for optimization regularization */
    this.thetaRest = [0.2, 0.5, -0.6, 0.1];
    /** RGB axis helpers per link (optional visibility) */
    this.frameHelpers = [];
    this.sceneRef = null;
    /** When set, joint angles drive GLTF ctrl_1…4 nodes instead of moving procedural link groups. */
    this.useGltfKinematic = false;
    /** @type {THREE.Object3D | null} */
    this.gltfRoot = null;
    /** @type {(THREE.Object3D | null)[]} */
    this.gltfCtrlNodes = [null, null, null, null];
    /** Rest pose from file (per ctrl node) */
    /** @type {THREE.Euler[]} */
    this.gltfRestEuler = [];
    /** @type {('x'|'y'|'z')[]} */
    this.gltfJointAxes = ['y', 'z', 'z', 'z'];
    /** @type {THREE.Object3D | null} */
    this.gltfEEObject = null;
    /** Optional GLB gripper pieces (first pass: two meshes under EE). */
    /** @type {THREE.Object3D | null} */
    this.gripJawA = null;
    /** @type {THREE.Object3D | null} */
    this.gripJawB = null;
    /** @type {THREE.Euler | null} */
    this._gripRestA = null;
    /** @type {THREE.Euler | null} */
    this._gripRestB = null;
    /** Smoothed 0 = open, 1 = closed */
    this.gripperVisual = 0;
    /** Target grip closure (set from IK / game logic). */
    this.gripperGoal = 0;
    /** @type {THREE.Mesh | null} */
    this.proceduralGripTip = null;
  }

  /**
   * After loading xarm.glb: drive the nested ctrl_i chain (see XArmKinematicGLTF.js).
   * @param {THREE.Object3D} gltfRoot
   * @param {(THREE.Object3D | null)[]} ctrlNodes — ctrl_1 … ctrl_4
   * @param {THREE.Object3D | null} eeObject — e.g. grabber (end-effector region)
   * @param {('x'|'y'|'z')[] | undefined} jointAxes
   */
  attachGltfKinematic(gltfRoot, ctrlNodes, eeObject, jointAxes) {
    this.gltfRoot = gltfRoot;
    this.gltfCtrlNodes = ctrlNodes;
    this.gltfEEObject = eeObject;
    this.gltfJointAxes = jointAxes || ['y', 'z', 'z', 'z'];
    this.gltfRestEuler = ctrlNodes.map((n) =>
      n
        ? new THREE.Euler(n.rotation.x, n.rotation.y, n.rotation.z, n.rotation.order)
        : new THREE.Euler()
    );
    this.useGltfKinematic = true;
    this.thetaRest = [0, 0, 0, 0];
    this.setJointAngles([0, 0, 0, 0]);
    this._setupGripperFromEE(eeObject);
  }

  /** Remove GLB from scene graph and fall back to procedural FK visuals. Caller should scene.remove(gltfRoot) first if attached. */
  detachGltfKinematic() {
    this.useGltfKinematic = false;
    this.gltfRoot = null;
    this.gltfCtrlNodes = [null, null, null, null];
    this.gltfEEObject = null;
    this.gltfRestEuler = [];
    this.gripJawA = null;
    this.gripJawB = null;
    this._gripRestA = null;
    this._gripRestB = null;
    this.gripperVisual = 0;
    this.gripperGoal = 0;
  }

  /**
   * @param {number} g 0 = open, 1 = closed (smoothed internally in updateMeshes).
   */
  setGripperGoal(g) {
    this.gripperGoal = THREE.MathUtils.clamp(g, 0, 1);
  }

  /** Heuristic: two jaw meshes under end-effector, or one mesh for squeeze. */
  _setupGripperFromEE(ee) {
    this.gripJawA = null;
    this.gripJawB = null;
    this._gripRestA = null;
    this._gripRestB = null;
    if (!ee) return;

    const byHint = [null, null];
    const hint = (name) => {
      const n = (name || '').toLowerCase();
      return /left|_l$|^l_|jaw.?l|finger.?l|pad.?l/.test(n) ? 0
        : /right|_r$|^r_|jaw.?r|finger.?r|pad.?r/.test(n) ? 1
        : -1;
    };
    ee.traverse((o) => {
      if (!o.isMesh) return;
      const h = hint(o.name);
      if (h === 0 || h === 1) byHint[h] = o;
    });
    if (byHint[0] && byHint[1]) {
      this.gripJawA = byHint[0];
      this.gripJawB = byHint[1];
    } else {
      const meshes = [];
      ee.traverse((o) => {
        if (o.isMesh) meshes.push(o);
      });
      if (meshes.length >= 2) {
        this.gripJawA = meshes[0];
        this.gripJawB = meshes[1];
      } else if (meshes.length === 1) {
        this.gripJawA = meshes[0];
      }
    }

    if (this.gripJawA) {
      this._gripRestA = new THREE.Euler(
        this.gripJawA.rotation.x,
        this.gripJawA.rotation.y,
        this.gripJawA.rotation.z,
        this.gripJawA.rotation.order
      );
    }
    if (this.gripJawB) {
      this._gripRestB = new THREE.Euler(
        this.gripJawB.rotation.x,
        this.gripJawB.rotation.y,
        this.gripJawB.rotation.z,
        this.gripJawB.rotation.order
      );
    }
  }

  _applyGripperVisual(t) {
    const close = THREE.MathUtils.clamp(t, 0, 1);
    const spread = 0.42;

    if (this.gripJawA && this._gripRestA) {
      this.gripJawA.rotation.order = this._gripRestA.order;
      this.gripJawA.rotation.copy(this._gripRestA);
      this.gripJawA.rotation.z += close * spread;
    }
    if (this.gripJawB && this._gripRestB) {
      this.gripJawB.rotation.order = this._gripRestB.order;
      this.gripJawB.rotation.copy(this._gripRestB);
      this.gripJawB.rotation.z -= close * spread;
    } else if (this.gripJawA && !this.gripJawB && this._gripRestA) {
      const s = THREE.MathUtils.lerp(1, 0.9, close);
      this.gripJawA.scale.setScalar(s);
    }

    if (this.proceduralGripTip) {
      const u = THREE.MathUtils.lerp(1, 0.72, close);
      this.proceduralGripTip.scale.set(1, u, 1);
    }
  }

  applyGltfKinematicRotations() {
    for (let i = 0; i < 4; i++) {
      const n = this.gltfCtrlNodes[i];
      if (!n) continue;
      const base = this.gltfRestEuler[i];
      const ax = this.gltfJointAxes[i] || 'z';
      n.rotation.order = base.order;
      n.rotation.copy(base);
      if (ax === 'x') n.rotation.x += this.theta[i];
      else if (ax === 'y') n.rotation.y += this.theta[i];
      else n.rotation.z += this.theta[i];
    }
  }

  /**
   * Procedural arm: base → shoulder → elbow → wrist → end-effector.
   * @param {{ meshStyle?: 'boxes' | 'bones' }} [options] — bones = spheres + cylinders (default reactive look).
   */
  buildRobot(scene, options = {}) {
    this.sceneRef = scene;
    const meshStyle = options.meshStyle || 'bones';
    const colors = [0x3b82f6, 0x22c55e, 0xeab308, 0xf97316, 0xec4899];

    const mkMat = (hex) =>
      new THREE.MeshStandardMaterial({
        color: hex,
        metalness: 0.25,
        roughness: 0.55,
      });

    // --- Geometry constants (clarity over realism) ---
    const baseH = 0.35;
    const baseR = 0.16;
    const L1 = 1.05;
    const L2 = 0.9;
    const L3 = 0.35;
    const L4 = 0.18;
    const linkThickness = 0.07;

    // Base link: cylinder along Y, origin at bottom center
    const baseGeo = new THREE.CylinderGeometry(baseR, baseR * 1.1, baseH, 20);
    const baseMesh = new THREE.Mesh(baseGeo, mkMat(colors[0]));
    baseMesh.castShadow = true;
    baseMesh.position.set(0, baseH / 2, 0);
    const baseGroup = new THREE.Group();
    baseGroup.add(baseMesh);

    const link0 = new Link('base', baseGroup);
    link0.localMatrix.identity();
    scene.add(baseGroup);
    this.links.push(link0);
    this.rootLink = link0;

    // Link 1: upper arm along +X from joint
    const g1 = makeArmSegmentGroup(L1, linkThickness, colors[1], mkMat, meshStyle);
    const link1 = new Link('shoulder', g1);

    // Joint 1: base yaw about Y, joint at top of base
    const T0 = new THREE.Matrix4().makeTranslation(0, baseH, 0);
    const axisY = new THREE.Vector3(0, 1, 0);
    const j0 = new Joint('base_yaw', link0, link1, axisY, T0);
    this.joints.push(j0);
    scene.add(g1);

    // Link 2: forearm
    const g2 = makeArmSegmentGroup(L2, linkThickness * 0.95, colors[2], mkMat, meshStyle);
    const link2 = new Link('elbow', g2);

    const T1 = new THREE.Matrix4().makeTranslation(L1, 0, 0);
    const axisZ = new THREE.Vector3(0, 0, 1);
    const j1 = new Joint('shoulder_pitch', link1, link2, axisZ, T1);
    this.joints.push(j1);
    scene.add(g2);

    // Link 3: wrist segment
    const g3 = makeArmSegmentGroup(L3, linkThickness * 0.9, colors[3], mkMat, meshStyle);
    const link3 = new Link('wrist', g3);

    const T2 = new THREE.Matrix4().makeTranslation(L2, 0, 0);
    const j2 = new Joint('elbow', link2, link3, axisZ, T2);
    this.joints.push(j2);
    scene.add(g3);

    // Link 4: tip + end-effector marker
    const g4 = new THREE.Group();
    if (meshStyle === 'bones') {
      const mat = mkMat(colors[4]);
      const rS = linkThickness * 1.2;
      const rC = linkThickness * 0.5;
      const jnt = new THREE.Mesh(new THREE.SphereGeometry(rS, 14, 14), mat);
      jnt.castShadow = true;
      const cyl = new THREE.Mesh(new THREE.CylinderGeometry(rC, rC, L4, 12), mat);
      cyl.rotation.z = Math.PI / 2;
      cyl.position.set(L4 / 2, 0, 0);
      cyl.castShadow = true;
      const tipMesh = new THREE.Mesh(new THREE.SphereGeometry(0.056, 14, 14), mkMat(0xffffff));
      tipMesh.position.set(L4 + 0.06, 0, 0);
      tipMesh.castShadow = true;
      this.proceduralGripTip = tipMesh;
      g4.add(jnt, cyl, tipMesh);
    } else {
      const arm4Geo = new THREE.BoxGeometry(L4, linkThickness * 0.85, linkThickness * 0.85);
      const arm4Mesh = new THREE.Mesh(arm4Geo, mkMat(colors[4]));
      arm4Mesh.position.set(L4 / 2, 0, 0);
      arm4Mesh.castShadow = true;
      const tipMesh = new THREE.Mesh(new THREE.SphereGeometry(0.055, 16, 16), mkMat(0xffffff));
      tipMesh.position.set(L4 + 0.06, 0, 0);
      this.proceduralGripTip = tipMesh;
      g4.add(arm4Mesh);
      g4.add(tipMesh);
    }
    /** Extra RGB frame at the tip for wrist-orientation visualization (Euler vs quaternion along spline). */
    this.eeOrientationGroup = new THREE.Group();
    this.eeOrientationGroup.position.set(L4 + 0.06, 0, 0);
    const eeAxes = new THREE.AxesHelper(0.28);
    this.eeOrientationGroup.add(eeAxes);
    g4.add(this.eeOrientationGroup);
    const link4 = new Link('end_effector', g4);

    const T3 = new THREE.Matrix4().makeTranslation(L3, 0, 0);
    const j3 = new Joint('wrist_roll', link3, link4, axisZ, T3);
    this.joints.push(j3);
    scene.add(g4);

    this.links.push(link1, link2, link3, link4);
    this.endEffectorLink = link4;
    this.endEffectorTipOffset.set(L4 + 0.06, 0, 0);

    for (const link of this.links) {
      link.mesh.traverse((o) => {
        if (o.isMesh) o.userData.pickable = true;
      });
    }

    const limits = [
      [-2.7, 2.7],
      [-0.1, 2.35],
      [-2.65, 0.45],
      [-2.85, 2.85],
    ];
    this.joints.forEach((j, i) => {
      j.minTheta = limits[i][0];
      j.maxTheta = limits[i][1];
    });

    this.addCoordinateFrames();
    return this;
  }

  setJointAngles(thetaArr) {
    for (let i = 0; i < this.joints.length; i++) {
      this.theta[i] = thetaArr[i] ?? 0;
      this.joints[i].theta = this.theta[i];
    }
  }

  /**
   * Forward kinematics: propagate worldMatrix along the serial chain.
   * world_child = world_parent * T_constant * T_variable(theta)
   */
  forwardKinematics() {
    this.rootLink.worldMatrix.copy(this.rootLink.localMatrix);

    let current = this.rootLink;
    for (let i = 0; i < this.joints.length; i++) {
      const joint = this.joints[i];
      joint.theta = this.theta[i];
      const jointM = joint.getJointTransform(this.theta[i]);
      const child = joint.childLink;
      child.worldMatrix.multiplyMatrices(current.worldMatrix, jointM);
      current = child;
    }
  }

  getEndEffectorPosition() {
    if (this.gltfEEObject) {
      const box = new THREE.Box3().setFromObject(this.gltfEEObject);
      return box.getCenter(new THREE.Vector3());
    }
    const p = this.endEffectorTipOffset.clone();
    return p.applyMatrix4(this.endEffectorLink.worldMatrix);
  }

  getEndEffectorQuaternion() {
    if (this.gltfEEObject) {
      const q = new THREE.Quaternion();
      this.gltfEEObject.getWorldQuaternion(q);
      return q.normalize();
    }
    const q = new THREE.Quaternion();
    const pos = new THREE.Vector3();
    const scl = new THREE.Vector3();
    this.endEffectorLink.worldMatrix.decompose(pos, q, scl);
    return q.normalize();
  }

  updateMeshes() {
    if (this.useGltfKinematic) {
      this.applyGltfKinematicRotations();
      const k = 0.14;
      this.gripperVisual = THREE.MathUtils.lerp(this.gripperVisual, this.gripperGoal, k);
      this._applyGripperVisual(this.gripperVisual);
      if (this.sceneRef) this.sceneRef.updateMatrixWorld(true);
      return;
    }
    const k = 0.14;
    this.gripperVisual = THREE.MathUtils.lerp(this.gripperVisual, this.gripperGoal, k);
    this._applyGripperVisual(this.gripperVisual);
    for (const link of this.links) {
      const m = link.mesh;
      m.matrixAutoUpdate = false;
      m.matrix.copy(link.worldMatrix);
      m.matrixWorldNeedsUpdate = true;
    }
  }

  addCoordinateFrames() {
    const axisLen = 0.22;
    for (const link of this.links) {
      const ah = new THREE.AxesHelper(axisLen);
      ah.visible = false;
      link.mesh.add(ah);
      this.frameHelpers.push(ah);
    }
  }

  setFramesVisible(on) {
    for (const h of this.frameHelpers) h.visible = on;
  }

  /** World origins of each child link frame after joint i (serial chain). */
  getJointWorldPositions() {
    const positions = [];
    let W = this.rootLink.worldMatrix.clone();
    for (let i = 0; i < this.joints.length; i++) {
      const jointM = this.joints[i].getJointTransform(this.theta[i]);
      W.multiply(jointM);
      positions.push(new THREE.Vector3().setFromMatrixPosition(W));
    }
    return positions;
  }

  /**
   * Revolute axis of joint i in world coordinates (unit vector).
   */
  getJointAxisWorld(jointIdx) {
    const joint = this.joints[jointIdx];
    let W = this.rootLink.worldMatrix.clone();
    for (let i = 0; i < jointIdx; i++) {
      W.multiply(this.joints[i].getJointTransform(this.theta[i]));
    }
    W.multiply(joint.T_constant);
    const q = new THREE.Quaternion();
    W.decompose(new THREE.Vector3(), q, new THREE.Vector3());
    return joint.axis.clone().applyQuaternion(q).normalize();
  }
}
