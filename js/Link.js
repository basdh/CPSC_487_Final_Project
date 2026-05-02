import * as THREE from 'three';

/**
 * Link — a rigid body in the kinematic tree.
 * Links are spatial objects with SE(3) poses stored as homogeneous transforms.
 * Each link carries a visual mesh; worldMatrix places that mesh in the scene.
 */
export class Link {
  /**
   * @param {string} name
   * @param {THREE.Object3D} mesh — visual for this link (group or mesh)
   */
  constructor(name, mesh) {
    this.name = name;
    this.mesh = mesh;
    /** @type {Link | null} */
    this.parent = null;
    /** @type {Link[]} */
    this.children = [];
    /** Constant offset of mesh geometry in the link frame (usually identity). */
    this.localMatrix = new THREE.Matrix4().identity();
    /** World pose: homogeneous transform from link frame to world (Unit 4.1 FK output). */
    this.worldMatrix = new THREE.Matrix4().identity();
  }
}
