import * as THREE from 'three';
import { OrbitControls } from 'https://unpkg.com/three@0.160.0/examples/jsm/controls/OrbitControls.js';
import GUI from 'https://unpkg.com/lil-gui@0.19.1/dist/lil-gui.esm.js';
import { RobotArm } from './RobotArm.js';
import { SplineController } from './SplineController.js';
import { OptimizationController } from './OptimizationController.js';
import { PointerController } from './PointerController.js';
import { RotationComparison } from './RotationComparison.js';
import { VisualizationController } from './VisualizationController.js';
import { stripProceduralMeshes } from './XArmGLTF.js';
import { loadKinematicGLTF } from './XArmKinematicGLTF.js';
import { frameCameraToObject, makeOriginArrows } from './sceneUtils.js';
import { MazeGame } from './MazeGame.js';

const params = {
  pointerMode: 'No Reaction',
  /** Display: load GLB vs procedural FK arm only */
  geometryMode: 'GLB file',
  meshStyle: 'bones',
  /** Relative URL or same-origin path to any .glb with ctrl_1…4-style joints */
  modelUrl: './assets/xarm.glb',
  ctrlNodesCsv: 'ctrl_1,ctrl_2,ctrl_3,ctrl_4',
  eeHintsCsv: 'grabber,polySurface3,tcp',
  /** Per-joint rotation axis for ctrl_1…4 (matches ctrl names order): x, y, or z in each node's local rest frame */
  jointAxesCsv: 'y,z,z,z',
  /** Raycast: GLB only vs whole scene */
  pickScope: 'GLB model',
  showPointerTarget: true,
  showAvoidanceZone: true,
  optimizationEnabled: true,
  learningRate: 0.065,
  gradientEpsilon: 0.035,
  useCentralDifferences: true,
  gradientClip: 18,
  jointOutputSmoothing: 0.32,
  /** Larger = pointer target follows mouse faster (still smoothed) */
  pointerSmoothing: 14,
  /** Keyboard (W/S) vertical move speed in m/s for the 3D target */
  pointerVerticalSpeed: 1.15,
  /** Arrow keys: nudge target on the interaction plane (camera-aligned XZ) */
  pointerPlanarSpeed: 2.6,
  /** Min / max extra Y on top of the floor plane hit (meters) */
  pointerVerticalMin: -0.35,
  pointerVerticalMax: 2.2,
  /** Reach mode: end-effector within this distance (m) to target → grip “contact” */
  gripContactDistance: 0.07,
  targetWeight: 4.0,
  smoothnessWeight: 0.42,
  restPoseWeight: 0.1,
  avoidanceWeight: 1.2,
  safeRadius: 0.55,
  jointLimitsEnabled: false,
  splineMode: 'Hermite',
  animationSpeed: 0.12,
  showSplineCurve: false,
  showGhostPoses: false,
  showTangents: false,
  showAcceleration: false,
  rotationInterpolationMode: 'Quaternion SLERP',
  showOrientationGhosts: false,
  showEndEffectorTrail: false,
  showErrorLine: true,
  showObjectiveText: true,
  showGradientArrows: true,
  showSplinePanel: false,
  showWorldAxes: true,
  showFrames: false,
  showOriginArrows: true,
};

let splineU = 0;
let prevWristQuat = new THREE.Quaternion();
const clock = new THREE.Clock();

let robot;
let spline;
let optimizer;
let pointer;
let viz;
let rotCompare;
/** @type {THREE.Mesh[]} */
let ghostMeshes = [];
let worldAxes;
let interactionPlane;
let pointerTargetMesh;
let avoidZoneMesh;
let renderer;
let scene;
let camera;
let controls;
let canvas;
/** @type {THREE.Group | null} */
let originArrowsGroup = null;
const pickRaycaster = new THREE.Raycaster();
/** Smoothed 3D target for optimization (reduces high-frequency jitter from mouse) */
const smoothedPointer = new THREE.Vector3();
let currentGltfRoot = null;

/** W/S = pointer height only */
const keyPointerVertical = { up: false, down: false };
/** Arrow keys = pointer movement on floor plane (lab mode only) */
const keyPointerPlanar = { up: false, down: false, left: false, right: false };

/** @type {MazeGame | null} */
let mazeGame = null;

const container = document.getElementById('app');

function getCameraPlanarAxes(cam) {
  const forward = new THREE.Vector3();
  cam.getWorldDirection(forward);
  forward.y = 0;
  if (forward.lengthSq() < 1e-8) forward.set(0, 0, -1);
  else forward.normalize();
  const right = new THREE.Vector3().crossVectors(forward, new THREE.Vector3(0, 1, 0)).normalize();
  return { forward, right };
}

function startMazeChallenge() {
  if (!renderer || !container) return;
  if (!mazeGame) {
    mazeGame = new MazeGame({
      renderer,
      container,
      onExit: () => {},
      getMainControls: () => controls,
      modelUrl: params.modelUrl,
    });
  } else if (mazeGame.modelUrl !== params.modelUrl) {
    mazeGame.modelUrl = params.modelUrl;
    mazeGame.armTemplateKin = null;
  }
  mazeGame.enter().catch(console.error);
}

function setModelStatus(msg) {
  const el = document.getElementById('model-status');
  if (el) el.textContent = msg;
}

function parseCfgCsv(s) {
  return String(s || '')
    .split(',')
    .map((x) => x.trim())
    .filter(Boolean);
}

/** @returns {('x'|'y'|'z')[]} */
function parseJointAxesCsv(s) {
  const parts = String(s || '')
    .split(',')
    .map((x) => x.trim().toLowerCase())
    .filter(Boolean);
  const out = [];
  for (const p of parts) {
    if (p === 'x' || p === 'y' || p === 'z') out.push(p);
  }
  while (out.length < 4) out.push('z');
  return out.slice(0, 4);
}

function placeMainArmAtOrigin(root, floorY = 0, clearance = 0.01) {
  root.updateMatrixWorld(true);
  const box0 = new THREE.Box3().setFromObject(root);
  const cx = (box0.min.x + box0.max.x) * 0.5;
  const cz = (box0.min.z + box0.max.z) * 0.5;
  root.position.x -= cx;
  root.position.z -= cz;

  root.updateMatrixWorld(true);
  const box1 = new THREE.Box3().setFromObject(root);
  root.position.y += floorY - box1.min.y + clearance;
  root.updateMatrixWorld(true);
}

/**
 * Load / replace GLB (same kinematic conventions: named ctrl nodes + optional EE object).
 */
async function applyGltfFromParams() {
  if (!scene || !robot) return;
  const ctrlNames = parseCfgCsv(params.ctrlNodesCsv);
  const eeHints = parseCfgCsv(params.eeHintsCsv);
  const kin = await loadKinematicGLTF(params.modelUrl, {
    ctrlNames: ctrlNames.length ? ctrlNames : undefined,
    endEffectorHints: eeHints.length ? eeHints : undefined,
  });
  if (!kin) {
    setModelStatus(`Failed to load: ${params.modelUrl}`);
    return;
  }
  if (currentGltfRoot) {
    scene.remove(currentGltfRoot);
    robot.detachGltfKinematic();
    currentGltfRoot = null;
  }
  currentGltfRoot = kin.root;
  scene.add(kin.root);
  placeMainArmAtOrigin(kin.root, 0, 0.01);
  stripProceduralMeshes(robot);
  robot.proceduralGripTip = null;
  const jointAxes = parseJointAxesCsv(params.jointAxesCsv);
  robot.attachGltfKinematic(kin.root, kin.ctrlNodes, kin.eeObject, jointAxes);
  robot.setJointAngles([0, 0, 0, 0]);
  robot.forwardKinematics();
  robot.updateMeshes();
  frameCameraToObject(camera, controls, kin.root);
  if (pointer) smoothedPointer.copy(pointer.targetWorld);
  if (optimizer) optimizer.syncFilteredFromRobot();
  if (typeof spline !== 'undefined' && spline) spline.refreshKeyQuaternions();
  prevWristQuat.copy(robot.getEndEffectorQuaternion());
  setModelStatus(`Loaded GLB: ${params.modelUrl}`);
}

async function bootstrap() {
  canvas = document.createElement('canvas');
  container.style.position = 'relative';
  container.appendChild(canvas);

  try {
    renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
  } catch (e) {
    setModelStatus('WebGL unavailable — try another browser.');
    throw e;
  }

  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMap.enabled = true;
  if (THREE.SRGBColorSpace !== undefined) {
    renderer.outputColorSpace = THREE.SRGBColorSpace;
  }
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.15;

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b1220);

  camera = new THREE.PerspectiveCamera(
    50,
    window.innerWidth / window.innerHeight,
    0.08,
    250
  );
  camera.position.set(1.35, 1.05, 2.05);

  controls = new OrbitControls(camera, canvas);
  controls.target.set(0, 0.42, 0);
  controls.enableDamping = true;
  controls.dampingFactor = 0.06;
  controls.update();

  scene.add(new THREE.AmbientLight(0xffffff, 0.48));
  scene.add(new THREE.HemisphereLight(0xc8d8ff, 0x12141f, 1.15));
  const dir = new THREE.DirectionalLight(0xffffff, 1.35);
  dir.position.set(6, 14, 8);
  dir.castShadow = true;
  scene.add(dir);

  worldAxes = new THREE.AxesHelper(2.4);
  worldAxes.position.y = 0.002;
  scene.add(worldAxes);

  const grid = new THREE.GridHelper(28, 28, 0x475569, 0x1e293b);
  grid.position.y = 0;
  scene.add(grid);

  const planeGeom = new THREE.PlaneGeometry(48, 48);
  const planeMat = new THREE.MeshBasicMaterial({
    color: 0x334155,
    transparent: true,
    opacity: 0.28,
    side: THREE.DoubleSide,
    depthWrite: false,
  });
  interactionPlane = new THREE.Mesh(planeGeom, planeMat);
  interactionPlane.rotation.x = -Math.PI / 2;
  interactionPlane.position.y = 0.09;
  interactionPlane.renderOrder = -1;
  interactionPlane.userData.excludeFromFrame = true;
  interactionPlane.userData.excludeFromPick = true;
  scene.add(interactionPlane);

  pointer = new PointerController(camera, canvas, interactionPlane);
  pointer.pointerNdc.set(0, 0.15);
  pointer.raycastToPlane();
  smoothedPointer.copy(pointer.targetWorld);

  robot = new RobotArm();
  robot.buildRobot(scene, {
    meshStyle: params.meshStyle === 'boxes' ? 'boxes' : 'bones',
  });
  robot.setJointAngles([0, 0, 0, 0]);
  robot.forwardKinematics();
  robot.updateMeshes();

  optimizer = new OptimizationController(robot);
  optimizer.syncFilteredFromRobot();
  optimizer.thetaPrev = robot.theta.slice();

  if (params.geometryMode === 'GLB file') {
    await applyGltfFromParams();
    if (!currentGltfRoot) {
      robot.setJointAngles([0.35, 0.45, -0.55, 0.1]);
      robot.forwardKinematics();
      robot.updateMeshes();
      frameCameraToObject(camera, controls, robot.rootLink.mesh);
      setModelStatus('GLB failed — procedural FK arm only. Fix modelUrl / path.');
    }
  } else {
    robot.setJointAngles([0.35, 0.45, -0.55, 0.1]);
    robot.forwardKinematics();
    robot.updateMeshes();
    frameCameraToObject(camera, controls, robot.rootLink.mesh);
    setModelStatus('Procedural reactive arm (bones). Switch geometry to GLB file + Reload to use a model.');
  }

  optimizer.syncFilteredFromRobot();
  optimizer.thetaPrev = robot.theta.slice();

  originArrowsGroup = makeOriginArrows(scene, 2.2);

  spline = new SplineController(robot);

  pointerTargetMesh = new THREE.Mesh(
    new THREE.SphereGeometry(0.065, 24, 24),
    new THREE.MeshStandardMaterial({
      color: 0x38bdf8,
      emissive: 0x38bdf8,
      emissiveIntensity: 0.95,
      metalness: 0.12,
      roughness: 0.35,
    })
  );
  pointerTargetMesh.userData.excludeFromFrame = true;
  pointerTargetMesh.userData.excludeFromPick = true;
  scene.add(pointerTargetMesh);

  avoidZoneMesh = new THREE.Mesh(
    new THREE.SphereGeometry(1, 32, 32),
    new THREE.MeshBasicMaterial({
      color: 0xff3333,
      wireframe: true,
      transparent: true,
      opacity: 0.35,
    })
  );
  avoidZoneMesh.visible = false;
  avoidZoneMesh.userData.excludeFromFrame = true;
  avoidZoneMesh.userData.excludeFromPick = true;
  scene.add(avoidZoneMesh);

  viz = new VisualizationController(scene, robot, camera);
  viz.attachObjectiveHud(container);

  rotCompare = new RotationComparison(scene);
  prevWristQuat.copy(robot.getEndEffectorQuaternion());

  const ghostGroup = new THREE.Group();
  scene.add(ghostGroup);
  const ghostMat = new THREE.MeshStandardMaterial({
    color: 0x94a3b8,
    transparent: true,
    opacity: 0.18,
    depthWrite: false,
  });
  ghostMeshes = [];
  for (let g = 0; g < 9; g++) {
    const m = new THREE.Mesh(new THREE.SphereGeometry(0.05, 12, 12), ghostMat.clone());
    m.visible = false;
    ghostGroup.add(m);
    ghostMeshes.push(m);
  }

  setupGui();

  optimizer.updateObjectiveValue(robot.theta, smoothedPointer, robot.thetaRest, false);

  window.addEventListener('resize', onResize);
  canvas.addEventListener('mousemove', (e) => pointer.updateMouse(e));
  canvas.addEventListener('click', onCanvasClick);

  window.addEventListener('keydown', onKeyPointerVertical);
  window.addEventListener('keyup', onKeyPointerVertical);
  window.addEventListener('keydown', onKeyPointerPlanar);
  window.addEventListener('keyup', onKeyPointerPlanar);
  window.addEventListener('keydown', onEscapeMaze);

  animate();
}

function onEscapeMaze(event) {
  if (event.code !== 'Escape' || !mazeGame?.isActive) return;
  mazeGame.exit();
  event.preventDefault();
}

function onKeyPointerVertical(event) {
  if (mazeGame?.isActive) return;
  const down = event.type === 'keydown';
  const k = event.code;
  if (k === 'KeyW') {
    keyPointerVertical.up = down;
    if (down) event.preventDefault();
  }
  if (k === 'KeyS') {
    keyPointerVertical.down = down;
    if (down) event.preventDefault();
  }
}

function onKeyPointerPlanar(event) {
  if (mazeGame?.isActive) return;
  const down = event.type === 'keydown';
  const k = event.code;
  if (k === 'ArrowUp') {
    keyPointerPlanar.up = down;
    if (down) event.preventDefault();
  }
  if (k === 'ArrowDown') {
    keyPointerPlanar.down = down;
    if (down) event.preventDefault();
  }
  if (k === 'ArrowLeft') {
    keyPointerPlanar.left = down;
    if (down) event.preventDefault();
  }
  if (k === 'ArrowRight') {
    keyPointerPlanar.right = down;
    if (down) event.preventDefault();
  }
}

function onCanvasClick(event) {
  if (!camera || !canvas || !scene || !robot) return;
  const rect = canvas.getBoundingClientRect();
  const ndc = new THREE.Vector2(
    ((event.clientX - rect.left) / rect.width) * 2 - 1,
    -((event.clientY - rect.top) / rect.height) * 2 + 1
  );
  pickRaycaster.setFromCamera(ndc, camera);
  const pickList = [];
  const root =
    params.pickScope === 'GLB model' && robot.gltfRoot ? robot.gltfRoot : scene;
  root.traverse((o) => {
    if (o.isMesh && !o.userData.excludeFromPick) pickList.push(o);
  });
  const hits = pickRaycaster.intersectObjects(pickList, false);
  if (hits.length === 0) {
    setModelStatus(
      params.pickScope === 'GLB model'
        ? 'Pick: miss — click an xArm mesh (or set pick scope to Everything).'
        : 'Pick: miss.'
    );
    return;
  }
  const hit = hits[0];
  const name = hit.object.name || '(unnamed)';
  console.log('[Pick]', name, hit.object);
  setModelStatus(`Pick: "${name}" (see console)`);
}

function onResize() {
  if (!renderer) return;
  if (mazeGame?.isActive) {
    mazeGame.onResize();
    return;
  }
  if (!camera) return;
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}

function setupGui() {
  const gui = new GUI({ title: 'SE(3) Robot Arm' });

  const fModel = gui.addFolder('Model / GLB');
  fModel.add(params, 'geometryMode', ['GLB file', 'Procedural only']);
  fModel.add(params, 'meshStyle', ['bones', 'boxes']).name('procedural look');
  fModel.add(params, 'modelUrl').name('glTF URL');
  fModel.add(params, 'ctrlNodesCsv').name('ctrl names (csv)');
  fModel.add(params, 'eeHintsCsv').name('EE hints (csv)');
  fModel.add(params, 'jointAxesCsv').name('joint axes (x,y,z ×4)');
  fModel.add({ reload: () => applyGltfFromParams().catch(console.error) }, 'reload').name('Reload GLB');

  const fPointer = gui.addFolder('Pointer');
  fPointer.add(params, 'pointerMode', ['No Reaction', 'Reach Pointer', 'Avoid Pointer']);
  fPointer.add(params, 'pickScope', ['GLB model', 'Everything']);
  fPointer.add(params, 'pointerSmoothing', 2, 28, 1);
  fPointer.add(params, 'pointerPlanarSpeed', 0.4, 6, 0.1).name('kbd arrows plane');
  fPointer.add(params, 'pointerVerticalSpeed', 0.2, 3.5, 0.05).name('kbd W/S height');
  fPointer.add(params, 'pointerVerticalMin', -1.5, 0.5, 0.05).name('height min (m)');
  fPointer.add(params, 'pointerVerticalMax', 0.2, 3.5, 0.05).name('height max (m)');
  fPointer.add(params, 'gripContactDistance', 0.02, 0.2, 0.005).name('grip touch dist.');
  fPointer.add(params, 'showPointerTarget');
  fPointer.add(params, 'showAvoidanceZone');

  const fOpt = gui.addFolder('Optimization');
  fOpt.add(params, 'optimizationEnabled');
  fOpt.add(params, 'learningRate', 0.015, 0.35, 0.005);
  fOpt.add(params, 'gradientEpsilon', 0.012, 0.08, 0.002);
  fOpt.add(params, 'useCentralDifferences');
  fOpt.add(params, 'gradientClip', 2, 60, 1);
  fOpt.add(params, 'jointOutputSmoothing', 0.08, 0.85, 0.02);
  fOpt.add(params, 'targetWeight', 0, 8, 0.1);
  fOpt.add(params, 'smoothnessWeight', 0, 2, 0.02);
  fOpt.add(params, 'restPoseWeight', 0, 2, 0.02);
  fOpt.add(params, 'avoidanceWeight', 0, 5, 0.1);
  fOpt.add(params, 'safeRadius', 0.15, 1.5, 0.02);
  fOpt.add(params, 'jointLimitsEnabled');

  const fSpl = gui.addFolder('Spline');
  fSpl.add(params, 'splineMode', [
    'Linear',
    'Hermite',
    'Catmull-Rom',
    'Quadratic B-Spline',
    'Cubic B-Spline',
  ]);
  fSpl.add(params, 'animationSpeed', 0.02, 0.5, 0.01);
  fSpl.add(params, 'showSplineCurve');
  fSpl.add(params, 'showGhostPoses');
  fSpl.add(params, 'showTangents');
  fSpl.add(params, 'showAcceleration');

  const fRot = gui.addFolder('Rotation');
  fRot.add(params, 'rotationInterpolationMode', ['Euler', 'Quaternion SLERP']);
  fRot.add(params, 'showOrientationGhosts');

  const fVis = gui.addFolder('Visualization');
  fVis.add(params, 'showWorldAxes');
  fVis.add(params, 'showOriginArrows');
  fVis.add(params, 'showFrames');
  fVis.add(params, 'showEndEffectorTrail');
  fVis.add(params, 'showErrorLine');
  fVis.add(params, 'showObjectiveText');
  fVis.add(params, 'showGradientArrows');
  fVis.add(params, 'showSplinePanel');

  const fGame = gui.addFolder('Maze mini-game');
  fGame.add({ play: () => startMazeChallenge() }, 'play').name('Play maze challenge');

  fModel.close();
  // Pointer + Maze stay expanded by default for gameplay convenience.
  fOpt.close();
  fSpl.close();
  fRot.close();
  fVis.close();
}

function syncOptimizerFromGui() {
  optimizer.optimizationEnabled = params.optimizationEnabled;
  optimizer.learningRate = params.learningRate;
  optimizer.gradientEpsilon = params.gradientEpsilon;
  optimizer.useCentralDifferences = params.useCentralDifferences;
  optimizer.gradientClip = params.gradientClip;
  optimizer.jointOutputSmoothing = params.jointOutputSmoothing;
  optimizer.targetWeight = params.targetWeight;
  optimizer.smoothnessWeight = params.smoothnessWeight;
  optimizer.restPoseWeight = params.restPoseWeight;
  optimizer.avoidanceWeight = params.avoidanceWeight;
  optimizer.safeRadius = params.safeRadius;
  optimizer.jointLimitsEnabled = params.jointLimitsEnabled;
}

function updateEndEffectorOrientationDisplay(splineTime) {
  if (params.pointerMode === 'No Reaction') {
    if (robot.eeOrientationGroup) robot.eeOrientationGroup.quaternion.identity();
    return;
  }

  if (!robot.eeOrientationGroup) return;

  const qFk = robot.getEndEffectorQuaternion();
  const eePos = robot.getEndEffectorPosition();
  const target = smoothedPointer.clone();
  const dirW = target.sub(eePos);
  if (dirW.lengthSq() < 1e-8) {
    robot.eeOrientationGroup.quaternion.identity();
    return;
  }
  dirW.normalize();
  const inv = qFk.clone().invert();
  const dirLocal = dirW.clone().applyQuaternion(inv);
  const qAlign = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(1, 0, 0), dirLocal);

  if (params.rotationInterpolationMode === 'Euler') {
    const ePrev = new THREE.Euler().setFromQuaternion(prevWristQuat, 'XYZ');
    const eTgt = new THREE.Euler().setFromQuaternion(qAlign, 'XYZ');
    const eSm = new THREE.Euler(
      THREE.MathUtils.lerp(ePrev.x, eTgt.x, 0.18),
      THREE.MathUtils.lerp(ePrev.y, eTgt.y, 0.18),
      THREE.MathUtils.lerp(ePrev.z, eTgt.z, 0.18),
      'XYZ'
    );
    const qSm = new THREE.Quaternion().setFromEuler(eSm);
    prevWristQuat.copy(qSm);
    robot.eeOrientationGroup.quaternion.copy(qFk.clone().invert().multiply(qSm));
  } else {
    const qSm = new THREE.Quaternion().slerpQuaternions(prevWristQuat, qAlign, 0.22);
    prevWristQuat.copy(qSm);
    robot.eeOrientationGroup.quaternion.copy(qFk.clone().invert().multiply(qSm));
  }
}

function updateGhosts() {
  if (params.pointerMode !== 'No Reaction' || !params.showGhostPoses) {
    for (const m of ghostMeshes) m.visible = false;
    return;
  }
  for (let i = 0; i < ghostMeshes.length; i++) {
    const u = i / (ghostMeshes.length - 1);
    const ang = spline.getJointAngles(u, params.splineMode);
    const saved = robot.theta.slice();
    robot.setJointAngles(ang);
    robot.forwardKinematics();
    robot.updateMeshes();
    const p = robot.getEndEffectorPosition();
    robot.setJointAngles(saved);
    robot.forwardKinematics();
    robot.updateMeshes();
    ghostMeshes[i].position.copy(p);
    ghostMeshes[i].visible = true;
  }
}

function animate() {
  requestAnimationFrame(animate);
  if (!renderer || !robot) return;
  const dt = clock.getDelta();

  if (mazeGame?.isActive) {
    mazeGame.update(dt);
    mazeGame.render();
    return;
  }

  syncOptimizerFromGui();

  const vAxis =
    (keyPointerVertical.up ? 1 : 0) - (keyPointerVertical.down ? 1 : 0);
  if (vAxis !== 0) {
    pointer.verticalOffset +=
      vAxis * params.pointerVerticalSpeed * dt;
    pointer.verticalOffset = THREE.MathUtils.clamp(
      pointer.verticalOffset,
      params.pointerVerticalMin,
      params.pointerVerticalMax
    );
    pointer.syncTargetWorld();
  }

  let mx = 0;
  let mz = 0;
  const { forward, right } = getCameraPlanarAxes(camera);
  if (keyPointerPlanar.up) {
    mx += forward.x;
    mz += forward.z;
  }
  if (keyPointerPlanar.down) {
    mx -= forward.x;
    mz -= forward.z;
  }
  if (keyPointerPlanar.right) {
    mx += right.x;
    mz += right.z;
  }
  if (keyPointerPlanar.left) {
    mx -= right.x;
    mz -= right.z;
  }
  const planarLen = Math.hypot(mx, mz);
  if (planarLen > 1e-8) {
    const step = (params.pointerPlanarSpeed * dt) / planarLen;
    pointer.planeHit.x += mx * step;
    pointer.planeHit.z += mz * step;
    pointer.syncTargetWorld();
  }

  const ptrSmooth = Math.min(1, dt * params.pointerSmoothing);
  smoothedPointer.lerp(pointer.targetWorld, ptrSmooth);

  if (worldAxes) worldAxes.visible = params.showWorldAxes;
  if (originArrowsGroup) originArrowsGroup.visible = params.showOriginArrows;

  robot.setFramesVisible(params.showFrames);

  if (params.pointerMode === 'No Reaction') {
    robot.setJointAngles(robot.thetaRest);
    robot.forwardKinematics();
    updateEndEffectorOrientationDisplay(splineU);
  } else {
    splineU += params.animationSpeed * dt;
    const target = smoothedPointer;
    const avoid = params.pointerMode === 'Avoid Pointer';
    optimizer.thetaPrev = robot.theta.slice();
    if (params.optimizationEnabled) {
      const next = optimizer.step(robot.theta, target, robot.thetaRest, avoid);
      robot.setJointAngles(next);
      robot.forwardKinematics();
    } else {
      optimizer.updateObjectiveValue(robot.theta, target, robot.thetaRest, avoid);
      robot.forwardKinematics();
    }
    updateEndEffectorOrientationDisplay(splineU);
  }

  const eeDist = robot.getEndEffectorPosition().distanceTo(smoothedPointer);
  const wantGrip =
    params.pointerMode === 'Reach Pointer' && eeDist < params.gripContactDistance;
  robot.setGripperGoal(wantGrip ? 1 : 0);

  robot.updateMeshes();

  pointerTargetMesh.visible = params.showPointerTarget;
  if (pointer.hasHit || params.pointerMode !== 'No Reaction') {
    pointerTargetMesh.position.copy(smoothedPointer);
  }

  const showAvoid =
    params.showAvoidanceZone && params.pointerMode === 'Avoid Pointer';
  avoidZoneMesh.visible = showAvoid;
  if (showAvoid) {
    avoidZoneMesh.position.copy(pointer.targetWorld);
    avoidZoneMesh.scale.setScalar(params.safeRadius);
  }

  viz.updateErrorLine(
    params.showErrorLine &&
      params.pointerMode !== 'No Reaction' &&
      pointer.hasHit,
    robot.getEndEffectorPosition(),
    smoothedPointer
  );

  if (params.showEndEffectorTrail) {
    viz.updateTrails(true);
  } else {
    viz.clearTrail();
    viz.trailLine.visible = false;
  }

  viz.updateSplinePanel(
    params.showSplinePanel,
    spline,
    params.splineMode,
    params.showSplineCurve,
    params.showTangents,
    params.showAcceleration
  );

  if (params.pointerMode === 'No Reaction') {
    optimizer.lastObjective = 0;
    optimizer.lastGradient = [0, 0, 0, 0];
  }

  const gradNorm = Math.hypot(...optimizer.lastGradient);
  viz.updateObjectiveText(
    params.showObjectiveText,
    optimizer.lastObjective,
    params.pointerMode,
    params.splineMode,
    spline.getContinuityDescription(params.splineMode),
    splineU % 1,
    gradNorm
  );

  const showGrad =
    params.showGradientArrows &&
    params.optimizationEnabled &&
    (params.pointerMode === 'Reach Pointer' || params.pointerMode === 'Avoid Pointer');
  viz.updateGradientArrows(showGrad, robot, optimizer.lastGradient);

  updateGhosts();

  rotCompare.update(
    dt,
    false,
    params.showOrientationGhosts,
    false
  );

  controls.update();
  renderer.render(scene, camera);
}

bootstrap().catch((err) => {
  console.error(err);
  setModelStatus(`Error: ${err?.message || err}`);
});
