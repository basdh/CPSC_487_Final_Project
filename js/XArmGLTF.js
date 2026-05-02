import * as THREE from 'three';
import { GLTFLoader } from 'https://unpkg.com/three@0.160.0/examples/jsm/loaders/GLTFLoader.js';

/**
 * Load xArm / UFACTORY GLB and attach mesh parts to our FK link groups.
 * Exporter naming varies; we log the full hierarchy so you can adjust mapping after upload.
 */

function logHierarchy(obj, depth = 0) {
  const pad = '  '.repeat(depth);
  const type = obj.type || '?';
  const name = obj.name || '(unnamed)';
  console.log(`${pad}${type} "${name}"`);
  for (const c of obj.children) logHierarchy(c, depth + 1);
}

function sortByNameIndex(nodes) {
  return [...nodes].sort((a, b) => {
    const ma = (a.name || '').match(/(\d+)/);
    const mb = (b.name || '').match(/(\d+)/);
    const ia = ma ? parseInt(ma[1], 10) : 999;
    const ib = mb ? parseInt(mb[1], 10) : 999;
    if (ia !== ib) return ia - ib;
    return (a.name || '').localeCompare(b.name || '');
  });
}

function collectMeshes(root) {
  const meshes = [];
  root.traverse((o) => {
    if (o.isMesh) meshes.push(o);
  });
  return sortByNameIndex(meshes);
}

/**
 * @param {string} url
 * @param {THREE.Scene} scene
 * @returns {Promise<{ root: THREE.Group; visualScene: THREE.Object3D } | null>}
 */
export async function loadXArmGLTF(url, scene) {
  const loader = new GLTFLoader();
  let gltf;
  try {
    gltf = await loader.loadAsync(url);
  } catch (e) {
    console.warn('[XArmGLTF] Could not load', url, e);
    return null;
  }

  const root = new THREE.Group();
  root.name = 'xarm_visual_root';

  const visualScene = gltf.scene.clone(true);
  root.add(visualScene);

  visualScene.traverse((o) => {
    if (o.isMesh) {
      o.castShadow = true;
      o.receiveShadow = true;
      o.userData.pickable = true;
      if (o.material && !Array.isArray(o.material)) {
        o.material = o.material.clone();
        /** CAD exports often default to single-sided; fixes “invisible” faces from some angles. */
        o.material.side = THREE.DoubleSide;
        o.material.needsUpdate = true;
      } else if (Array.isArray(o.material)) {
        o.material = o.material.map((m) => {
          const c = m.clone();
          c.side = THREE.DoubleSide;
          return c;
        });
      }
    }
  });

  const box = new THREE.Box3().setFromObject(root);
  const size = box.getSize(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z, 1e-6);
  const target = 2.5;
  const s = target / maxDim;
  root.scale.setScalar(s);

  const box2 = new THREE.Box3().setFromObject(root);
  const center = box2.getCenter(new THREE.Vector3());
  root.position.sub(center);
  root.position.y -= box2.min.y;

  console.log('[XArmGLTF] Loaded', url, '— hierarchy (use names to tune bind order):');
  logHierarchy(visualScene, 0);

  scene.add(root);
  return { root, visualScene };
}

/**
 * Remove procedural Mesh children from link groups; keeps AxesHelper, Groups (e.g. EE frame).
 * @param {import('./RobotArm.js').RobotArm} robot
 */
export function stripProceduralMeshes(robot) {
  for (const link of robot.links) {
    const g = link.mesh;
    const remove = [];
    g.children.forEach((ch) => {
      if (ch.isMesh) remove.push(ch);
    });
    for (const ch of remove) {
      ch.geometry?.dispose();
      if (ch.material) {
        if (Array.isArray(ch.material)) ch.material.forEach((m) => m.dispose?.());
        else ch.material.dispose?.();
      }
      g.remove(ch);
    }
  }
}

/**
 * Move meshes from GLTF onto serial link groups (best-effort).
 * @param {import('./RobotArm.js').RobotArm} robot
 * @param {THREE.Object3D} visualScene — cloned gltf.scene
 */
export function bindXArmMeshesToLinks(robot, visualScene) {
  const meshes = collectMeshes(visualScene);
  const nLinks = robot.links.length;

  console.log(
    '[XArmGLTF] Binding',
    meshes.length,
    'meshes to',
    nLinks,
    'links. Order:',
    meshes.map((m) => m.name || '(mesh)')
  );

  if (meshes.length === 0) {
    visualScene.parent?.remove(visualScene);
    robot.rootLink.mesh.add(visualScene);
    return;
  }

  if (meshes.length >= nLinks) {
    for (let i = 0; i < nLinks; i++) {
      const m = meshes[i];
      m.parent?.remove(m);
      robot.links[i].mesh.add(m);
      m.position.set(0, 0, 0);
      m.rotation.set(0, 0, 0);
    }
    /** Leftover meshes → last link */
    for (let j = nLinks; j < meshes.length; j++) {
      const m = meshes[j];
      m.parent?.remove(m);
      robot.endEffectorLink.mesh.add(m);
    }
  } else {
    /** Few meshes: stack onto chain endpoints */
    const m0 = meshes[0];
    m0.parent?.remove(m0);
    robot.rootLink.mesh.add(m0);
    for (let j = 1; j < meshes.length; j++) {
      const m = meshes[j];
      m.parent?.remove(m);
      robot.endEffectorLink.mesh.add(m);
    }
  }

  /** Remove empty GLTF shell from scene if unused */
  let p = visualScene;
  while (p && p.children.length === 0) {
    const next = p.parent;
    next?.remove(p);
    p = next;
  }
}
