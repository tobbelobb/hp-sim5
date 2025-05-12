export default class Vector2 {
  constructor(x = 0.0, y = 0.0) { this.x = x; this.y = y; }
  set(v) { this.x = v.x; this.y = v.y; }
  clone() { return new Vector2(this.x, this.y); }
  add(v, s = 1.0) { this.x += v.x * s; this.y += v.y * s; return this; }
  addVectors(a, b) { this.x = a.x + b.x; this.y = a.y + b.y; return this; }
  subtract(v, s = 1.0) { this.x -= v.x * s; this.y -= v.y * s; return this; }
  subtractVectors(a, b) { this.x = a.x - b.x; this.y = a.y - b.y; return this; }
  distanceTo(b) { return Math.hypot(this.x - b.x, this.y - b.y); }
  distanceToSq(b) { return (this.x - b.x)**2 + (this.y - b.y)**2; }
  length() { return Math.hypot(this.x, this.y); }
  lengthSq() { return this.x**2 + this.y**2; }
  scale(s) { this.x *= s; this.y *= s; return this; }
  dot(v) { return this.x * v.x + this.y * v.y; }
  perp() { return new Vector2(-this.y, this.x); }
  normalize() { const l = this.length(); if (l>0) this.scale(1/l); return this; }
  rotate(ang, center, cw) {
    if (!cw) ang = -ang;
    const cos = Math.cos(ang), sin = Math.sin(ang);
    this.subtract(center);
    const x = this.x, y = this.y;
    this.x = x*cos - y*sin;
    this.y = x*sin + y*cos;
    this.add(center);
    return this;
  }
}
