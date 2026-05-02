import * as THREE from 'three';

/**
 * Joint — defines the transformation between parent and child link frames.
 * The pose of the child relative to the parent is:
 *   T_child = T_parent * T_constant * T_variable(theta)
 * where theta is the joint parameter (e.g. revolute angle in radians).
 */
export class Joint {
  /**
   * @param {string} name
   * @param {Link} parentLink
   * @param {Link} childLink
   * @param {THREE.Vector3} axis — unit axis of rotation in the joint frame after T_constant
   * @param {THREE.Matrix4} T_constant — fixed SE(3) from parent link frame to joint axis / child attachment
   */
  constructor(name, parentLink, childLink, axis, T_constant) {
    this.name = name;
    this.parentLink = parentLink;
    this.childLink = childLink;
    this.axis = axis.clone().normalize();
    this.theta = 0;
    this.minTheta = -Infinity;
    this.maxTheta = Infinity;
    this.T_constant = T_constant.clone();

    parentLink.children.push(childLink);
    childLink.parent = parentLink;
  }

  /**
   * Variable joint transform: rotation about the revolute axis by theta.
   * @param {number} theta
   * @returns {THREE.Matrix4}
   */
  getVariableTransform(theta) {
    const m = new THREE.Matrix4();
    return m.makeRotationAxis(this.axis, theta);
  }

  /**
   * Full SE(3) from parent link frame to child link frame for this joint.
   * T_joint = T_constant * R(axis, theta)
   */
  getJointTransform(theta) {
    const variable = this.getVariableTransform(theta);
    return new THREE.Matrix4().multiplyMatrices(this.T_constant, variable);
  }
}
