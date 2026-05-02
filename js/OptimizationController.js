import * as THREE from 'three';

const softplus = (z, k = 10) => {
  const kz = k * z;
  if (kz > 40) return z;
  if (kz < -40) return Math.exp(kz) / k;
  return Math.log(1 + Math.exp(kz)) / k;
};

/**
 * Iterative optimization over joint angles theta in R^4.
 * Uses filtered joint angles + optional gradient clipping + central differences for stabler reach.
 */
export class OptimizationController {
  constructor(robot) {
    this.robot = robot;
    this.optimizationEnabled = true;
    /** Raw gradient-descent step size (scaled internally by filters). */
    this.learningRate = 0.065;
    this.gradientEpsilon = 0.035;
    /** If true, use central differences (2× FK calls per axis, less biased). */
    this.useCentralDifferences = true;
    /** Clamp each ∂E/∂θᵢ to [-gradientClip, gradientClip] (reduces spike jitter). */
    this.gradientClip = 18;
    /**
     * Low-pass on joint outputs each frame: θ_f ← θ_f + λ (θ* − θ_f).
     * Higher = snappier, lower = smoother (typical 0.2–0.45).
     */
    this.jointOutputSmoothing = 0.32;
    this.targetWeight = 4.0;
    this.smoothnessWeight = 0.42;
    this.restPoseWeight = 0.1;
    this.avoidanceWeight = 1.2;
    this.safeRadius = 0.55;
    this.jointLimitsEnabled = false;
    this.softplusK = 10;
    this.thetaPrev = [0, 0, 0, 0];
    this.lastObjective = 0;
    this.lastGradient = [0, 0, 0, 0];
    /** Smoothed joint state (returned to caller after each step). */
    this.thetaFiltered = [0, 0, 0, 0];
  }

  /** Call when robot rest pose or DOF set changes. */
  syncFilteredFromRobot() {
    this.thetaFiltered = this.robot.theta.slice();
  }

  copyTheta(theta) {
    return theta.slice();
  }

  eePos(theta) {
    this.robot.setJointAngles(theta);
    this.robot.forwardKinematics();
    this.robot.updateMeshes();
    return this.robot.getEndEffectorPosition();
  }

  computeReachObjective(theta, pointerWorld, thetaRest) {
    const x = this.eePos(theta);
    let E =
      this.targetWeight * x.distanceToSquared(pointerWorld) +
      this.smoothnessWeight * this._norm2Diff(theta, this.thetaPrev) +
      this.restPoseWeight * this._norm2Diff(theta, thetaRest);
    return E;
  }

  computeAvoidObjective(theta, pointerWorld, thetaRest) {
    const x = this.eePos(theta);
    const d2 = x.distanceToSquared(pointerWorld);
    const rs = this.safeRadius * this.safeRadius;
    const z = rs - d2;
    let E =
      this.avoidanceWeight * softplus(z, this.softplusK) +
      this.smoothnessWeight * this._norm2Diff(theta, this.thetaPrev) +
      this.restPoseWeight * this._norm2Diff(theta, thetaRest);
    return E;
  }

  computeObjective(theta, pointerWorld, thetaRest, avoidMode) {
    return avoidMode
      ? this.computeAvoidObjective(theta, pointerWorld, thetaRest)
      : this.computeReachObjective(theta, pointerWorld, thetaRest);
  }

  _norm2Diff(a, b) {
    let s = 0;
    for (let i = 0; i < 4; i++) {
      const d = a[i] - b[i];
      s += d * d;
    }
    return s;
  }

  _clipGradient(grad) {
    const gMax = this.gradientClip;
    if (!gMax || gMax <= 0) return grad;
    return grad.map((g) => THREE.MathUtils.clamp(g, -gMax, gMax));
  }

  finiteDifferenceGradient(theta, pointerWorld, thetaRest, avoidMode) {
    const grad = [0, 0, 0, 0];
    const eps = this.gradientEpsilon;
    const central = this.useCentralDifferences;

    if (!central) {
      const base = this.computeObjective(theta, pointerWorld, thetaRest, avoidMode);
      for (let i = 0; i < 4; i++) {
        const tp = this.copyTheta(theta);
        tp[i] += eps;
        const Ep = this.computeObjective(tp, pointerWorld, thetaRest, avoidMode);
        grad[i] = (Ep - base) / eps;
      }
    } else {
      for (let i = 0; i < 4; i++) {
        const tp = this.copyTheta(theta);
        const tm = this.copyTheta(theta);
        tp[i] += eps * 0.5;
        tm[i] -= eps * 0.5;
        const Ep = this.computeObjective(tp, pointerWorld, thetaRest, avoidMode);
        const Em = this.computeObjective(tm, pointerWorld, thetaRest, avoidMode);
        grad[i] = (Ep - Em) / eps;
      }
    }

    this.robot.setJointAngles(theta);
    this.robot.forwardKinematics();
    this.robot.updateMeshes();

    return this._clipGradient(grad);
  }

  clampToLimits(theta) {
    if (!this.jointLimitsEnabled) return theta;
    const out = theta.slice();
    for (let i = 0; i < this.robot.joints.length; i++) {
      const j = this.robot.joints[i];
      out[i] = THREE.MathUtils.clamp(out[i], j.minTheta, j.maxTheta);
    }
    return out;
  }

  /**
   * One stabilized step: gradient step → clamp → temporal low-pass on θ_filtered (reduces jitter).
   */
  step(theta, pointerWorld, thetaRest, avoidMode) {
    if (!this.optimizationEnabled) return this.copyTheta(theta);

    const grad = this.finiteDifferenceGradient(theta, pointerWorld, thetaRest, avoidMode);
    this.lastGradient = grad.slice();

    const alpha = this.learningRate;
    const rawNext = theta.map((t, i) => t - alpha * grad[i]);
    const clamped = this.clampToLimits(rawNext);

    const lam = THREE.MathUtils.clamp(this.jointOutputSmoothing, 0.04, 1);
    for (let i = 0; i < 4; i++) {
      this.thetaFiltered[i] = THREE.MathUtils.lerp(this.thetaFiltered[i], clamped[i], lam);
    }

    const outClamped = this.clampToLimits(this.thetaFiltered.slice());

    this.robot.setJointAngles(outClamped);
    this.robot.forwardKinematics();
    this.robot.updateMeshes();

    for (let i = 0; i < 4; i++) this.thetaFiltered[i] = outClamped[i];

    this.lastObjective = this.computeObjective(outClamped, pointerWorld, thetaRest, avoidMode);
    return outClamped;
  }

  updateObjectiveValue(theta, pointerWorld, thetaRest, avoidMode) {
    this.lastObjective = this.computeObjective(theta, pointerWorld, thetaRest, avoidMode);
  }
}
