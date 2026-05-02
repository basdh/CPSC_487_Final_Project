import * as THREE from 'three';

/**
 * Aggregates scene overlays: trails, spline debug panel, error line, objective HUD.
 */
export class VisualizationController {
  constructor(scene, robot, camera) {
    this.scene = scene;
    this.robot = robot;
    this.camera = camera;

    this.errorLineGeom = new THREE.BufferGeometry();
    const errorMat = new THREE.LineBasicMaterial({ color: 0xff3366, linewidth: 1 });
    this.errorLine = new THREE.Line(this.errorLineGeom, errorMat);
    this.errorLine.visible = false;
    scene.add(this.errorLine);

    this.trailMax = 120;
    this.trailPts = [];
    this.trailGeom = new THREE.BufferGeometry();
    const trailMat = new THREE.LineBasicMaterial({
      color: 0x38bdf8,
      transparent: true,
      opacity: 0.65,
    });
    this.trailLine = new THREE.Line(this.trailGeom, trailMat);
    this.trailLine.visible = false;
    scene.add(this.trailLine);

    /** Spline continuity panel (side view in world space) */
    this.splinePanel = new THREE.Group();
    this.splinePanel.position.set(4.2, 1.1, 0);
    scene.add(this.splinePanel);
    this.panelCurve = null;
    this.panelPoints = [];
    this.panelTangents = [];
    this.panelAccels = [];

    this.gradientArrows = [];
    for (let i = 0; i < 4; i++) {
      const a = new THREE.ArrowHelper(
        new THREE.Vector3(0, 1, 0),
        new THREE.Vector3(),
        0.25,
        0x84cc16,
        0.06,
        0.04
      );
      a.visible = false;
      scene.add(a);
      this.gradientArrows.push(a);
    }

    this.objectiveDiv = null;
  }

  attachObjectiveHud(container) {
    const div = document.createElement('div');
    div.style.cssText =
      'position:absolute;left:12px;top:12px;color:#e2e8f0;font:13px/1.4 system-ui,Segoe UI,sans-serif;' +
      'background:rgba(15,23,42,0.72);padding:10px 14px;border-radius:8px;pointer-events:none;max-width:340px;';
    container.appendChild(div);
    this.objectiveDiv = div;
  }

  updateObjectiveText(show, objective, mode, splineMode, continuity, u, gradientNorm) {
    if (!this.objectiveDiv) return;
    this.objectiveDiv.style.display = show ? 'block' : 'none';
    if (!show) return;
    this.objectiveDiv.innerHTML = `
      <div><strong>E(θ)</strong> = ${objective.toFixed(5)}</div>
      <div><strong>Pointer</strong>: ${mode}</div>
      <div><strong>Spline</strong>: ${splineMode} — ${continuity}</div>
      <div><strong>u</strong> = ${u.toFixed(3)}</div>
      <div><strong>‖∇E‖</strong> ≈ ${gradientNorm.toFixed(4)}</div>
    `;
  }

  updateErrorLine(show, from, to) {
    if (!show) {
      this.errorLine.visible = false;
      return;
    }
    const positions = new Float32Array([from.x, from.y, from.z, to.x, to.y, to.z]);
    this.errorLineGeom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    this.errorLineGeom.setDrawRange(0, 2);
    this.errorLine.visible = true;
  }

  updateTrails(show) {
    if (!show) {
      this.trailLine.visible = false;
      return;
    }
    const ee = this.robot.getEndEffectorPosition();
    this.trailPts.push(ee.clone());
    if (this.trailPts.length > this.trailMax) this.trailPts.shift();
    if (this.trailPts.length < 2) {
      this.trailLine.visible = false;
      return;
    }
    const arr = new Float32Array(this.trailPts.length * 3);
    for (let i = 0; i < this.trailPts.length; i++) {
      arr[i * 3] = this.trailPts[i].x;
      arr[i * 3 + 1] = this.trailPts[i].y;
      arr[i * 3 + 2] = this.trailPts[i].z;
    }
    this.trailGeom.setAttribute('position', new THREE.BufferAttribute(arr, 3));
    this.trailLine.visible = true;
  }

  clearTrail() {
    this.trailPts.length = 0;
    this.trailLine.visible = false;
  }

  /**
   * Side panel: plot theta_0(u) as a 2D curve (x=u scale, y=angle) with tangents at sample knots.
   */
  updateSplinePanel(show, spline, splineMode, showCurve, showTangents, showAcceleration) {
    while (this.splinePanel.children.length) {
      this.splinePanel.remove(this.splinePanel.children[0]);
    }
    this.panelPoints.length = 0;
    this.panelTangents.length = 0;
    if (!show) return;

    const scaleX = 2.2;
    const scaleY = 1.8;
    const samples = 48;
    const pts = [];
    for (let i = 0; i <= samples; i++) {
      const u = i / samples;
      const angles = spline.getJointAngles(u, splineMode);
      const x = u * scaleX - scaleX / 2;
      const y = angles[0] * scaleY * 0.5;
      pts.push(new THREE.Vector3(x, y, 0));
    }
    const geom = new THREE.BufferGeometry().setFromPoints(pts);
    const mat = new THREE.LineBasicMaterial({ color: 0x22d3ee });
    const line = new THREE.Line(geom, mat);
    line.visible = showCurve;
    this.splinePanel.add(line);

    const keyN = spline.keyframes.length;
    for (let k = 0; k < keyN; k++) {
      const u = k / Math.max(1, keyN - 1);
      const angles = spline.keyframes[k].angles;
      const x = u * scaleX - scaleX / 2;
      const y = angles[0] * scaleY * 0.5;
      const sp = new THREE.Mesh(
        new THREE.SphereGeometry(0.055, 12, 12),
        new THREE.MeshBasicMaterial({ color: 0xfacc15 })
      );
      sp.userData.excludeFromPick = true;
      sp.position.set(x, y, 0);
      this.splinePanel.add(sp);

      if (showTangents && k < keyN - 1) {
        const u1 = (k + 0.05) / Math.max(1, keyN - 1);
        const a0 = spline.getJointAngles(u, splineMode)[0];
        const a1 = spline.getJointAngles(Math.min(u1, 0.999), splineMode)[0];
        const dx = 0.15;
        const dy = (a1 - a0) * scaleY * 0.5;
        const dir = new THREE.Vector3(dx, dy, 0).normalize();
        const ah = new THREE.ArrowHelper(dir, new THREE.Vector3(x, y, 0), 0.35, 0xf97316, 0.07, 0.05);
        this.splinePanel.add(ah);
      }

      if (showAcceleration && k > 0 && k < keyN - 1) {
        const um = (k - 1) / Math.max(1, keyN - 1);
        const up = (k + 1) / Math.max(1, keyN - 1);
        const am = spline.getJointAngles(um, splineMode)[0];
        const ac = spline.getJointAngles(u, splineMode)[0];
        const ap = spline.getJointAngles(up, splineMode)[0];
        const d2 = ap - 2 * ac + am;
        const accVec = new THREE.Vector3(0.12, d2 * scaleY * 0.25, 0);
        const ah2 = new THREE.ArrowHelper(
          accVec.clone().normalize(),
          new THREE.Vector3(x, y, 0),
          Math.min(0.4, Math.abs(d2) * 0.5 + 0.05),
          0xa855f7,
          0.06,
          0.04
        );
        this.splinePanel.add(ah2);
      }
    }

    const axisGeom = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(-scaleX / 2, 0, 0),
      new THREE.Vector3(scaleX / 2, 0, 0),
    ]);
    this.splinePanel.add(
      new THREE.Line(
        axisGeom,
        new THREE.LineBasicMaterial({ color: 0x475569, transparent: true, opacity: 0.6 })
      )
    );
  }

  /**
   * Arrows at joint sites: direction ≈ joint axis × (-∂E/∂θ_i) sense as rotation tangent.
   * Uses axis direction scaled by negative gradient component (descent direction hint).
   */
  updateGradientArrows(show, robot, gradient) {
    if (!show || !gradient) {
      for (const a of this.gradientArrows) a.visible = false;
      return;
    }
    const positions = robot.getJointWorldPositions();
    for (let i = 0; i < robot.joints.length; i++) {
      const ax = robot.getJointAxisWorld(i);
      const g = gradient[i];
      const dir = ax.clone().multiplyScalar(-Math.sign(g) * Math.min(0.35, Math.abs(g)));
      const arr = this.gradientArrows[i];
      if (!arr) continue;
      arr.position.copy(positions[i]);
      if (dir.lengthSq() < 1e-10) arr.visible = false;
      else {
        arr.setDirection(dir.clone().normalize());
        arr.setLength(Math.min(0.55, dir.length() * 4), 0.09, 0.055);
        arr.visible = true;
      }
    }
  }

  updateGhostPoses(show, spline, splineMode, robot) {
    // Ghost poses handled in main by updating a dedicated group - placeholder for API parity
    return show;
  }
}
