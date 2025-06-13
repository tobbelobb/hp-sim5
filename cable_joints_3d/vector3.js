export default class Vector3 {
  constructor(x = 0.0, y = 0.0, z = 0.0) { this.x = x; this.y = y; this.z = z; }
  set(v) { this.x = v.x; this.y = v.y; this.z = v.z; }
  clone() { return new Vector3(this.x, this.y, this.z); }
  add(v, s = 1.0) { this.x += v.x * s; this.y += v.y * s; this.z += v.z * s; return this; }
  addVectors(a, b) { this.x = a.x + b.x; this.y = a.y + b.y; this.z = a.z + b.z; return this; }
  subtract(v, s = 1.0) { this.x -= v.x * s; this.y -= v.y * s; this.z -= v.z * s; return this; }
  subtractVectors(a, b) { this.x = a.x - b.x; this.y = a.y - b.y; this.z = a.z - b.z; return this; }
  distanceTo(b) { return Math.hypot(this.x - b.x, this.y - b.y, this.z - b.z); }
  distanceToSq(b) { return (this.x - b.x)**2 + (this.y - b.y)**2 + (this.z - b.z)**2; }
  length() { return Math.hypot(this.x, this.y, this.z); }
  lengthSq() { return this.x**2 + this.y**2 + this.z**2; }
  scale(s) { this.x *= s; this.y *= s; this.z *= s; return this; }
  dot(v) { return this.x * v.x + this.y * v.y + this.z * v.z; }
  cross(v) { return new Vector3(
    this.y * v.z - this.z * v.y,
    this.z * v.x - this.x * v.z,
    this.x * v.y - this.y * v.x
  ); }
  normalize() { const l = this.length(); if (l>0) this.scale(1/l); return this; }
}
