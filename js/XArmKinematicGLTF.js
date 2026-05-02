import * as THREE from 'three';
import { GLTFLoader } from 'https://unpkg.com/three@0.160.0/examples/jsm/loaders/GLTFLoader.js';

/**
 * Load arbitrary GLB and drive named ctrl nodes + EE hint (xArm-compatible defaults).
 */
export const DEFAULT_GLTF_JOINT_AXES = /** @type {const} */ (['y', 'z', 'z', 'z']);

export function findByExactName(root, name) {
  let found = null;
  root.traverse((o) => {
    if (o.name === name) found = o;
  });
  return found;
}

export function findEEObject(root, hints) {
  for (const h of hints) {
    const o = findByExactName(root, h);
    if (o) return o;
  }
  let g = null;
  root.traverse((o) => {
    if (g || !o.name) return;
    const n = o.name.toLowerCase();
    if (hints.some((h) => n.includes(String(h).toLowerCase()))) g = o;
  });
  if (g) return g;
  root.traverse((o) => {
    if (!g && o.name && /grabber|gripper|tcp|tool|ee_link|end_effector/i.test(o.name)) g = o;
  });
  return g;
}

/**
 * Resolve ctrl nodes + EE on an already-loaded/cloned glTF root (e.g. after clone()).
 * @param {THREE.Object3D} root
 * @param {string[]} [ctrlNames]
 * @param {string[]} [eeHints]
 */
export function getKinematicBinding(root, ctrlNames, eeHints) {
  const cn = ctrlNames || ['ctrl_1', 'ctrl_2', 'ctrl_3', 'ctrl_4'];
  const eh =
    eeHints || ['grabber', 'polySurface3', 'tool0', 'EE', 'end_effector', 'tcp'];
  const ctrlNodes = cn.map((nm) => findByExactName(root, nm));
  const eeObject = findEEObject(root, eh);
  return { ctrlNodes, eeObject };
}

/**
 * @param {string} url
 * @param {{
 *   ctrlNames?: string[],
 *   endEffectorHints?: string[],
 *   targetHeight?: number
 * }} [opts]
 */
export async function loadKinematicGLTF(url, opts = {}) {
  const ctrlNames = opts.ctrlNames || ['ctrl_1', 'ctrl_2', 'ctrl_3', 'ctrl_4'];
  const eeHints =
    opts.endEffectorHints || ['grabber', 'polySurface3', 'tool0', 'EE', 'end_effector', 'tcp'];
  const targetH = opts.targetHeight ?? 1.15;

  const loader = new GLTFLoader();
  let gltf;
  try {
    gltf = await loader.loadAsync(url);
  } catch (e) {
    console.warn('[KinematicGLTF] load failed', url, e);
    return null;
  }

  const root = gltf.scene;
  root.name = 'kinematic_gltf_root';

  root.traverse((o) => {
    if (o.isMesh) {
      o.castShadow = true;
      o.receiveShadow = true;
      o.userData.pickable = true;
      if (Array.isArray(o.material)) {
        o.material = o.material.map((mat) => {
          if (!mat) return mat;
          const m = mat.clone();
          m.side = THREE.DoubleSide;
          return m;
        });
      } else if (o.material) {
        o.material = o.material.clone();
        o.material.side = THREE.DoubleSide;
      }
    }
  });

  const ctrlNodes = ctrlNames.map((nm) => findByExactName(root, nm));
  ctrlNames.forEach((nm, i) => {
    if (!ctrlNodes[i]) console.warn('[KinematicGLTF] Missing node', nm);
  });

  const eeObject = findEEObject(root, eeHints);

  const box = new THREE.Box3().setFromObject(root);
  const size = box.getSize(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z, 1e-6);
  const s = targetH / maxDim;
  root.scale.setScalar(s);

  const box2 = new THREE.Box3().setFromObject(root);
  const center = box2.getCenter(new THREE.Vector3());
  root.position.sub(center);
  root.position.y -= box2.min.y;

  console.log(
    '[KinematicGLTF]',
    url,
    '| ctrl:',
    ctrlNames.map((nm, i) => (ctrlNodes[i] ? nm : `${nm}(missing)`)).join(', '),
    '| EE:',
    eeObject ? eeObject.name : 'use FK'
  );

  return { root, ctrlNodes, eeObject };
}

/** @deprecated use loadKinematicGLTF */
export async function loadXArmKinematicScene(url) {
  return loadKinematicGLTF(url, {});
}
