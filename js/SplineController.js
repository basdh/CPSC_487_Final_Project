import * as THREE from 'three';

/**
 * Spline-driven joint trajectories for autonomous motion (Unit 4.3).
 *
 * “Linear splines are C^0 but not generally C^1 at knots.”
 * “Hermite splines can enforce C^1 continuity through specified endpoint derivatives.”
 * “Catmull–Rom splines create smoother motion by using neighboring control points.”
 */
export class SplineController {
  /**
   * @param {RobotArm} robot — used to sample end-effector quaternions at key poses
   */
  constructor(robot) {
    this.robot = robot;
    /** @type {{ angles: number[] }[]} */
    this.keyframes = [];
    /** Cached EE quaternions at each keyframe (from FK) */
    this.keyQuaternions = [];
    /** Cached Euler angles (same FK poses) for naive component interpolation */
    this.keyEulers = [];
    this.buildDefaultKeyframes();
  }

  buildDefaultKeyframes() {
    // Four key poses (radians) — cycles nicely for demo
    this.keyframes = [
      { angles: [0, 0.4, -0.5, 0.1] },
      { angles: [0.8, 0.9, -1.0, 0.4] },
      { angles: [-0.6, 1.1, -1.3, -0.3] },
      { angles: [0.4, 0.6, -0.8, 0.2] },
      { angles: [0, 0.4, -0.5, 0.1] }, // close loop
    ];
    this.refreshKeyQuaternions();
  }

  /** Recompute stored EE orientations from FK at each key pose. */
  refreshKeyQuaternions() {
    const saved = this.robot.theta.slice();
    this.keyQuaternions = [];
    this.keyEulers = [];
    const eTmp = new THREE.Euler();
    for (const kf of this.keyframes) {
      this.robot.setJointAngles(kf.angles);
      this.robot.forwardKinematics();
      this.robot.updateMeshes();
      const q = this.robot.getEndEffectorQuaternion().clone();
      this.keyQuaternions.push(q);
      eTmp.setFromQuaternion(q, 'XYZ');
      this.keyEulers.push(new THREE.Euler(eTmp.x, eTmp.y, eTmp.z, eTmp.order));
    }
    this.robot.setJointAngles(saved);
    this.robot.forwardKinematics();
    this.robot.updateMeshes();
  }

  /**
   * Naive Euler interpolation: same spline parameterization applied independently to
   * roll–pitch–yaw components derived from each key pose’s FK quaternion.
   */
  getEndEffectorQuaternionEulerNaive(t, splineMode) {
    const n = this.keyEulers.length;
    if (n < 2) return new THREE.Quaternion();
    const saved = this.keyframes;
    this.keyframes = this.keyEulers.map((e) => ({
      angles: [e.x, e.y, e.z, 0],
    }));
    const c = this.getJointAngles(t, splineMode);
    this.keyframes = saved;
    const e0 = new THREE.Euler(c[0], c[1], c[2], 'XYZ');
    return new THREE.Quaternion().setFromEuler(e0);
  }

  getContinuityDescription(mode) {
    switch (mode) {
      case 'Linear':
        return 'C^0 position continuous; not generally C^1 at internal knots';
      case 'Hermite':
        return 'C^1 within segments when endpoint tangents are matched';
      case 'Catmull-Rom':
        return 'Typically C^1 at interior knots; local control from neighboring keys';
      case 'Quadratic B-Spline':
        return 'Uniform quadratic B-spline: typically C^1 continuous at interior knots';
      case 'Cubic B-Spline':
        return 'Uniform cubic (polynomial) B-spline: typically C^2 continuous at interior knots';
      default:
        return '';
    }
  }

  /** Map global u in [0,1) to segment index and local v in [0,1]. */
  _segmentParams(u, numSeg) {
    const clamped = ((u % 1) + 1) % 1;
    const f = clamped * numSeg;
    const seg = Math.min(Math.floor(f), numSeg - 1);
    const v = f - seg;
    return { seg, v };
  }

  /** @param {number} t — parameter in [0,1) */
  evaluateLinear(t) {
    const n = this.keyframes.length;
    if (n < 2) return this.keyframes[0]?.angles.slice() ?? [0, 0, 0, 0];
    const numSeg = n - 1;
    const { seg, v } = this._segmentParams(t, numSeg);
    const a0 = this.keyframes[seg].angles;
    const a1 = this.keyframes[seg + 1].angles;
    return a0.map((x, i) => x + (a1[i] - x) * v);
  }

  /** Cubic Hermite on scalars: p0,p1,m0,m1, v in [0,1] */
  _hermiteScalar(p0, p1, m0, m1, v) {
    const v2 = v * v;
    const v3 = v2 * v;
    const h00 = 2 * v3 - 3 * v2 + 1;
    const h10 = v3 - 2 * v2 + v;
    const h01 = -2 * v3 + 3 * v2;
    const h11 = v3 - v2;
    return h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1;
  }

  /** Tangents for Hermite: Catmull–Rom style finite differences */
  _tangentAt(i, compIdx) {
    const n = this.keyframes.length;
    const P = (k) => this.keyframes[k].angles[compIdx];
    if (i <= 0) return (P(1) - P(0)) * 0.5;
    if (i >= n - 1) return (P(n - 1) - P(n - 2)) * 0.5;
    return (P(i + 1) - P(i - 1)) * 0.5;
  }

  evaluateHermite(t) {
    const n = this.keyframes.length;
    if (n < 2) return this.keyframes[0]?.angles.slice() ?? [0, 0, 0, 0];
    const numSeg = n - 1;
    const { seg, v } = this._segmentParams(t, numSeg);
    const out = [];
    for (let c = 0; c < 4; c++) {
      const p0 = this.keyframes[seg].angles[c];
      const p1 = this.keyframes[seg + 1].angles[c];
      const m0 = this._tangentAt(seg, c);
      const m1 = this._tangentAt(seg + 1, c);
      out.push(this._hermiteScalar(p0, p1, m0, m1, v));
    }
    return out;
  }

  /** Catmull–Rom (centripetal would be more uniform; standard chordal used here) */
  evaluateCatmullRom(t) {
    const n = this.keyframes.length;
    if (n < 4) return this.evaluateHermite(t);
    // Segments between interior keys 1..n-2; extend endpoints by duplication for endpoints
    const P = (idx, comp) => {
      const ii = Math.max(0, Math.min(n - 1, idx));
      return this.keyframes[ii].angles[comp];
    };
    const numSeg = n - 3;
    const { seg, v } = this._segmentParams(t, Math.max(1, numSeg));
    const i = seg + 1;
    const out = [];
    for (let c = 0; c < 4; c++) {
      const p0 = P(i - 1, c);
      const p1 = P(i, c);
      const p2 = P(i + 1, c);
      const p3 = P(i + 2, c);
      out.push(
        0.5 *
          (2 * p1 +
            (-p0 + p2) * v +
            (2 * p0 - 5 * p1 + 4 * p2 - p3) * v * v +
            (-p0 + 3 * p1 - 3 * p2 + p3) * v * v * v)
      );
    }
    return out;
  }

  /** Uniform quadratic B-spline (degree 2, polynomial basis). */
  evaluateQuadraticBSpline(t) {
    const n = this.keyframes.length;
    if (n < 3) return this.evaluateHermite(t);
    const numSeg = n - 2;
    const { seg, v } = this._segmentParams(t, numSeg);
    const B0 = 0.5 * (1 - 2 * v + v * v);
    const B1 = 0.5 * (1 + 2 * v - 2 * v * v);
    const B2 = 0.5 * v * v;
    const out = [];
    for (let c = 0; c < 4; c++) {
      const p0 = this.keyframes[seg].angles[c];
      const p1 = this.keyframes[seg + 1].angles[c];
      const p2 = this.keyframes[seg + 2].angles[c];
      out.push(B0 * p0 + B1 * p1 + B2 * p2);
    }
    return out;
  }

  /** Uniform cubic B-spline (degree 3, polynomial basis). */
  evaluateCubicBSpline(t) {
    const n = this.keyframes.length;
    if (n < 4) return this.evaluateQuadraticBSpline(t);
    const numSeg = n - 3;
    const { seg, v } = this._segmentParams(t, numSeg);
    const v2 = v * v;
    const v3 = v2 * v;
    const B0 = (1 - 3 * v + 3 * v2 - v3) / 6;
    const B1 = (4 - 6 * v2 + 3 * v3) / 6;
    const B2 = (1 + 3 * v + 3 * v2 - 3 * v3) / 6;
    const B3 = v3 / 6;
    const out = [];
    for (let c = 0; c < 4; c++) {
      const p0 = this.keyframes[seg].angles[c];
      const p1 = this.keyframes[seg + 1].angles[c];
      const p2 = this.keyframes[seg + 2].angles[c];
      const p3 = this.keyframes[seg + 3].angles[c];
      out.push(B0 * p0 + B1 * p1 + B2 * p2 + B3 * p3);
    }
    return out;
  }

  /**
   * Dispatch by mode string.
   * @param {number} t
   * @param {'Linear' | 'Hermite' | 'Catmull-Rom' | 'Quadratic B-Spline' | 'Cubic B-Spline'} mode
   */
  getJointAngles(t, mode) {
    switch (mode) {
      case 'Linear':
        return this.evaluateLinear(t);
      case 'Hermite':
        return this.evaluateHermite(t);
      case 'Catmull-Rom':
        return this.evaluateCatmullRom(t);
      case 'Quadratic B-Spline':
        return this.evaluateQuadraticBSpline(t);
      case 'Cubic B-Spline':
        return this.evaluateCubicBSpline(t);
      default:
        return this.evaluateLinear(t);
    }
  }

  /**
   * End-effector quaternion along spline parameter (for wrist SLERP vs Euler).
   * @param {number} t
   * @param {'Linear' | 'Hermite' | 'Catmull-Rom' | 'Quadratic B-Spline' | 'Cubic B-Spline'} splineMode
   * @param {'Euler' | 'Quaternion SLERP'} rotMode
   */
  getEndEffectorQuaternionSpline(t, splineMode, rotMode) {
    const n = this.keyQuaternions.length;
    if (n < 2) return new THREE.Quaternion();
    if (rotMode === 'Euler') {
      return this.getEndEffectorQuaternionEulerNaive(t, splineMode);
    }
    const numSeg = n - 1;
    const { seg, v } = this._segmentParams(t, numSeg);
    const q0 = this.keyQuaternions[seg];
    const q1 = this.keyQuaternions[seg + 1];
    return new THREE.Quaternion().slerpQuaternions(q0, q1, v);
  }

  /**
   * One scalar joint component as function of u for visualization (joint index 0).
   * @param {number} u
   * @param {'Linear' | 'Hermite' | 'Catmull-Rom' | 'Quadratic B-Spline' | 'Cubic B-Spline'} mode
   */
  sampleJoint0ForPlot(u, mode) {
    if (mode === 'Linear') return this.evaluateLinear(u)[0];
    if (mode === 'Hermite') return this.evaluateHermite(u)[0];
    if (mode === 'Quadratic B-Spline') return this.evaluateQuadraticBSpline(u)[0];
    if (mode === 'Cubic B-Spline') return this.evaluateCubicBSpline(u)[0];
    return this.evaluateCatmullRom(u)[0];
  }
}
