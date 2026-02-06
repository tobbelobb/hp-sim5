import Vector3 from './vector3.js';
import Quaternion from './quaternion.js';

export class CableLinkComponent {
  constructor(
    x = 0,
    y = 0,
    z = 0,
    orientation = null,
    planeNormal = null
  ) {
    this.prevCableAttachmentTimePos = new Vector3(x, y, z);
    this.prevCableAttachmentTimeOrientation = orientation ? orientation.clone() : new Quaternion();
    this.cablePlaneNormal = planeNormal ? planeNormal.clone() : new Vector3(0, 0, 1);
  }
}
