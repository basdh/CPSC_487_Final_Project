import * as THREE from 'three';

/**
 * Maps pointer (mouse) to a 3D target by raycasting onto an interaction plane,
 * plus optional keyboard offsets: planar moves live on planeHit (arrows in main UI),
 * vertical uses verticalOffset (W/S for height).
 */
export class PointerController {
  constructor(camera, domElement, planeMesh) {
    this.camera = camera;
    this.domElement = domElement;
    this.planeMesh = planeMesh;
    this.raycaster = new THREE.Raycaster();
    this.pointerNdc = new THREE.Vector2();
    /** Last hit on the plane (XZ motion from mouse). If ray misses, this is left unchanged. */
    this.planeHit = new THREE.Vector3(0, 0.09, 1.2);
    /** Extra world Y added on top of the plane hit (keyboard vertical control). */
    this.verticalOffset = 0;
    /** Combined effective target = planeHit + (0, verticalOffset, 0) */
    this.targetWorld = new THREE.Vector3();
    this.hasHit = false;
    this.syncTargetWorld();
  }

  /** Recompute targetWorld from planeHit and verticalOffset (call every frame if verticalOffset changes). */
  syncTargetWorld() {
    this.targetWorld.set(
      this.planeHit.x,
      this.planeHit.y + this.verticalOffset,
      this.planeHit.z
    );
  }

  updateMouse(event) {
    const rect = this.domElement.getBoundingClientRect();
    this.pointerNdc.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    this.pointerNdc.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    this.raycastToPlane();
  }

  raycastToPlane() {
    this.raycaster.setFromCamera(this.pointerNdc, this.camera);
    const hits = this.raycaster.intersectObject(this.planeMesh, false);
    if (hits.length > 0) {
      this.planeHit.copy(hits[0].point);
      this.hasHit = true;
    } else {
      this.hasHit = false;
    }
    this.syncTargetWorld();
  }

  getPointerTarget() {
    return this.targetWorld.clone();
  }
}
