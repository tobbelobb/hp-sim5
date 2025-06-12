import Vector3 from './vector3.js';

/**
 * A class representing a quaternion.
 * Quaternions are used to represent rotations in 3D space.
 */
export default class Quaternion {
    constructor(x = 0, y = 0, z = 0, w = 1) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    set(q) {
        this.x = q.x;
        this.y = q.y;
        this.z = q.z;
        this.w = q.w;
        return this;
    }

    clone() {
        return new Quaternion(this.x, this.y, this.z, this.w);
    }

    /**
     * Sets the quaternion from a rotation axis and angle.
     * @param {Vector3} axis The rotation axis (must be normalized).
     * @param {number} angle The rotation angle in radians.
     */
    setFromAxisAngle(axis, angle) {
        const halfAngle = angle / 2;
        const s = Math.sin(halfAngle);
        this.x = axis.x * s;
        this.y = axis.y * s;
        this.z = axis.z * s;
        this.w = Math.cos(halfAngle);
        return this;
    }

    /**
     * Multiplies this quaternion by another quaternion q.
     * @param {Quaternion} q The quaternion to multiply by.
     */
    multiply(q) {
        return this.multiplyQuaternions(this, q);
    }

    /**
     * Sets this quaternion to the product of two quaternions a and b.
     * @param {Quaternion} a The first quaternion.
     * @param {Quaternion} b The second quaternion.
     */
    multiplyQuaternions(a, b) {
        const qax = a.x, qay = a.y, qaz = a.z, qaw = a.w;
        const qbx = b.x, qby = b.y, qbz = b.z, qbw = b.w;

        this.x = qax * qbw + qaw * qbx + qay * qbz - qaz * qby;
        this.y = qay * qbw + qaw * qby + qaz * qbx - qax * qbz;
        this.z = qaz * qbw + qaw * qbz + qax * qby - qay * qbx;
        this.w = qaw * qbw - qax * qbx - qay * qby - qaz * qbz;

        return this;
    }

    lengthSq() {
        return this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w;
    }

    normalize() {
        let l = Math.sqrt(this.lengthSq());
        if (l === 0) {
            this.x = 0;
            this.y = 0;
            this.z = 0;
            this.w = 1;
        } else {
            l = 1 / l;
            this.x *= l;
            this.y *= l;
            this.z *= l;
            this.w *= l;
        }
        return this;
    }

    conjugate() {
        this.x *= -1;
        this.y *= -1;
        this.z *= -1;
        return this;
    }

    /**
     * Rotates a vector by this quaternion.
     * @param {Vector3} v The vector to rotate.
     * @returns {Vector3} The rotated vector.
     */
    transformVector(v) {
        const x = v.x, y = v.y, z = v.z;
        const qx = this.x, qy = this.y, qz = this.z, qw = this.w;

        // calculate quat * vector
        const ix = qw * x + qy * z - qz * y;
        const iy = qw * y + qz * x - qx * z;
        const iz = qw * z + qx * y - qy * x;
        const iw = -qx * x - qy * y - qz * z;

        // calculate result * conjugate(quat)
        const rx = ix * qw + iw * -qx + iy * -qz - iz * -qy;
        const ry = iy * qw + iw * -qy + iz * -qx - ix * -qz;
        const rz = iz * qw + iw * -qz + ix * -qy - iy * -qx;

        const result = v.clone();
        result.x = rx;
        result.y = ry;
        result.z = rz;
        return result;
    }
}
