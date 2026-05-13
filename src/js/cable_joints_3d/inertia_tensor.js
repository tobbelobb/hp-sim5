import Vector3 from './vector3.js';

const EPSILON = 1e-12;
const DEFAULT_AXIS = new Vector3(0.0, 0.0, 1.0);

function finiteNumber(value, fallback = 0.0) {
  const n = Number(value);
  return Number.isFinite(n) ? n : fallback;
}

function zeroMatrix3() {
  return [
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0],
  ];
}

function diagonalMatrix3(x, y = x, z = x) {
  return [
    [x, 0.0, 0.0],
    [0.0, y, 0.0],
    [0.0, 0.0, z],
  ];
}

export function cloneMatrix3(matrix) {
  return [
    [finiteNumber(matrix?.[0]?.[0]), finiteNumber(matrix?.[0]?.[1]), finiteNumber(matrix?.[0]?.[2])],
    [finiteNumber(matrix?.[1]?.[0]), finiteNumber(matrix?.[1]?.[1]), finiteNumber(matrix?.[1]?.[2])],
    [finiteNumber(matrix?.[2]?.[0]), finiteNumber(matrix?.[2]?.[1]), finiteNumber(matrix?.[2]?.[2])],
  ];
}

export function normalizeInertiaTensor(value) {
  if (typeof value === 'number') {
    const inertia = Number.isFinite(value) && value > 0.0 ? value : 0.0;
    return diagonalMatrix3(inertia);
  }

  const tensorValue = value?.inertiaTensor ?? value?.tensor ?? value;
  if (Array.isArray(tensorValue)) {
    if (
      tensorValue.length >= 3
      && Array.isArray(tensorValue[0])
      && Array.isArray(tensorValue[1])
      && Array.isArray(tensorValue[2])
    ) {
      return [
        [
          finiteNumber(tensorValue[0][0]),
          finiteNumber(tensorValue[0][1]),
          finiteNumber(tensorValue[0][2]),
        ],
        [
          finiteNumber(tensorValue[1][0]),
          finiteNumber(tensorValue[1][1]),
          finiteNumber(tensorValue[1][2]),
        ],
        [
          finiteNumber(tensorValue[2][0]),
          finiteNumber(tensorValue[2][1]),
          finiteNumber(tensorValue[2][2]),
        ],
      ];
    }
    if (tensorValue.length >= 9) {
      return [
        [
          finiteNumber(tensorValue[0]),
          finiteNumber(tensorValue[1]),
          finiteNumber(tensorValue[2]),
        ],
        [
          finiteNumber(tensorValue[3]),
          finiteNumber(tensorValue[4]),
          finiteNumber(tensorValue[5]),
        ],
        [
          finiteNumber(tensorValue[6]),
          finiteNumber(tensorValue[7]),
          finiteNumber(tensorValue[8]),
        ],
      ];
    }
  }

  return zeroMatrix3();
}

function isDiagonalMatrix3(matrix) {
  return (
    Math.abs(matrix[0][1]) <= EPSILON
    && Math.abs(matrix[0][2]) <= EPSILON
    && Math.abs(matrix[1][0]) <= EPSILON
    && Math.abs(matrix[1][2]) <= EPSILON
    && Math.abs(matrix[2][0]) <= EPSILON
    && Math.abs(matrix[2][1]) <= EPSILON
  );
}

export function invertMatrix3(matrix) {
  const m = cloneMatrix3(matrix);
  const a = m[0][0], b = m[0][1], c = m[0][2];
  const d = m[1][0], e = m[1][1], f = m[1][2];
  const g = m[2][0], h = m[2][1], i = m[2][2];

  const det = (
    a * ((e * i) - (f * h))
    - b * ((d * i) - (f * g))
    + c * ((d * h) - (e * g))
  );

  if (Number.isFinite(det) && Math.abs(det) > EPSILON) {
    const invDet = 1.0 / det;
    return [
      [
        ((e * i) - (f * h)) * invDet,
        ((c * h) - (b * i)) * invDet,
        ((b * f) - (c * e)) * invDet,
      ],
      [
        ((f * g) - (d * i)) * invDet,
        ((a * i) - (c * g)) * invDet,
        ((c * d) - (a * f)) * invDet,
      ],
      [
        ((d * h) - (e * g)) * invDet,
        ((b * g) - (a * h)) * invDet,
        ((a * e) - (b * d)) * invDet,
      ],
    ];
  }

  if (isDiagonalMatrix3(m)) {
    return diagonalMatrix3(
      m[0][0] > EPSILON ? 1.0 / m[0][0] : 0.0,
      m[1][1] > EPSILON ? 1.0 / m[1][1] : 0.0,
      m[2][2] > EPSILON ? 1.0 / m[2][2] : 0.0,
    );
  }

  return zeroMatrix3();
}

export function multiplyMatrix3Vector(matrix, vector) {
  const m = matrix || zeroMatrix3();
  return new Vector3(
    (finiteNumber(m[0]?.[0]) * vector.x) + (finiteNumber(m[0]?.[1]) * vector.y) + (finiteNumber(m[0]?.[2]) * vector.z),
    (finiteNumber(m[1]?.[0]) * vector.x) + (finiteNumber(m[1]?.[1]) * vector.y) + (finiteNumber(m[1]?.[2]) * vector.z),
    (finiteNumber(m[2]?.[0]) * vector.x) + (finiteNumber(m[2]?.[1]) * vector.y) + (finiteNumber(m[2]?.[2]) * vector.z),
  );
}

export function addMatrix3(a, b) {
  return [
    [
      finiteNumber(a?.[0]?.[0]) + finiteNumber(b?.[0]?.[0]),
      finiteNumber(a?.[0]?.[1]) + finiteNumber(b?.[0]?.[1]),
      finiteNumber(a?.[0]?.[2]) + finiteNumber(b?.[0]?.[2]),
    ],
    [
      finiteNumber(a?.[1]?.[0]) + finiteNumber(b?.[1]?.[0]),
      finiteNumber(a?.[1]?.[1]) + finiteNumber(b?.[1]?.[1]),
      finiteNumber(a?.[1]?.[2]) + finiteNumber(b?.[1]?.[2]),
    ],
    [
      finiteNumber(a?.[2]?.[0]) + finiteNumber(b?.[2]?.[0]),
      finiteNumber(a?.[2]?.[1]) + finiteNumber(b?.[2]?.[1]),
      finiteNumber(a?.[2]?.[2]) + finiteNumber(b?.[2]?.[2]),
    ],
  ];
}

export function multiplyMatrix3(a, b) {
  const result = zeroMatrix3();
  for (let row = 0; row < 3; row += 1) {
    for (let col = 0; col < 3; col += 1) {
      result[row][col] =
        (finiteNumber(a?.[row]?.[0]) * finiteNumber(b?.[0]?.[col]))
        + (finiteNumber(a?.[row]?.[1]) * finiteNumber(b?.[1]?.[col]))
        + (finiteNumber(a?.[row]?.[2]) * finiteNumber(b?.[2]?.[col]));
    }
  }
  return result;
}

export function transposeMatrix3(matrix) {
  return [
    [finiteNumber(matrix?.[0]?.[0]), finiteNumber(matrix?.[1]?.[0]), finiteNumber(matrix?.[2]?.[0])],
    [finiteNumber(matrix?.[0]?.[1]), finiteNumber(matrix?.[1]?.[1]), finiteNumber(matrix?.[2]?.[1])],
    [finiteNumber(matrix?.[0]?.[2]), finiteNumber(matrix?.[1]?.[2]), finiteNumber(matrix?.[2]?.[2])],
  ];
}

export function rotationMatrixFromQuaternion(quaternion) {
  const q = quaternion?.clone?.();
  if (!q) {
    return diagonalMatrix3(1.0);
  }
  q.normalize();
  const x = q.x, y = q.y, z = q.z, w = q.w;
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2;
  const yy = y * y2, yz = y * z2, zz = z * z2;
  const wx = w * x2, wy = w * y2, wz = w * z2;
  return [
    [1.0 - yy - zz, xy - wz, xz + wy],
    [xy + wz, 1.0 - xx - zz, yz - wx],
    [xz - wy, yz + wx, 1.0 - xx - yy],
  ];
}

export function transformInertiaTensorToWorld(tensor, orientation) {
  const rotation = rotationMatrixFromQuaternion(orientation);
  return multiplyMatrix3(multiplyMatrix3(rotation, tensor), transposeMatrix3(rotation));
}

export function parallelAxisTensor(mass, offset) {
  if (!(Number.isFinite(mass) && mass > 0.0) || !offset) {
    return zeroMatrix3();
  }
  const x = finiteNumber(offset.x);
  const y = finiteNumber(offset.y);
  const z = finiteNumber(offset.z);
  const rr = (x * x) + (y * y) + (z * z);
  return [
    [mass * (rr - (x * x)), -mass * x * y, -mass * x * z],
    [-mass * y * x, mass * (rr - (y * y)), -mass * y * z],
    [-mass * z * x, -mass * z * y, mass * (rr - (z * z))],
  ];
}

function normalizedAxis(axis) {
  const result = axis?.clone?.() ?? DEFAULT_AXIS.clone();
  if (result.lengthSq() <= EPSILON) {
    return DEFAULT_AXIS.clone();
  }
  return result.normalize();
}

function toLocalVector(orientation, vector) {
  if (!orientation?.clone) {
    return vector.clone();
  }
  return orientation.clone().conjugate().normalize().transformVector(vector);
}

function toWorldVector(orientation, vector) {
  if (!orientation?.clone) {
    return vector.clone();
  }
  return orientation.clone().normalize().transformVector(vector);
}

export function effectiveInertiaAboutLocalAxis(momentOfInertia, axisLocal = DEFAULT_AXIS) {
  if (!momentOfInertia) {
    return 0.0;
  }
  const axis = normalizedAxis(axisLocal);
  const tensor = momentOfInertia.inertiaTensor ?? momentOfInertia.tensor ?? zeroMatrix3();
  const projected = multiplyMatrix3Vector(tensor, axis);
  const inertia = axis.dot(projected);
  return Number.isFinite(inertia) && inertia > EPSILON ? inertia : 0.0;
}

export function effectiveInertiaAboutWorldAxis(momentOfInertia, orientation, axisWorld = DEFAULT_AXIS) {
  const axisLocal = toLocalVector(orientation, normalizedAxis(axisWorld));
  return effectiveInertiaAboutLocalAxis(momentOfInertia, axisLocal);
}

export function constrainedInvInertiaAboutLocalAxis(momentOfInertia, axisLocal = DEFAULT_AXIS) {
  const inertia = effectiveInertiaAboutLocalAxis(momentOfInertia, axisLocal);
  return inertia > EPSILON ? 1.0 / inertia : 0.0;
}

export function constrainedInvInertiaAboutWorldAxis(momentOfInertia, orientation, axisWorld = DEFAULT_AXIS) {
  const inertia = effectiveInertiaAboutWorldAxis(momentOfInertia, orientation, axisWorld);
  return inertia > EPSILON ? 1.0 / inertia : 0.0;
}

export function applyWorldInverseInertia(momentOfInertia, orientation, vectorWorld) {
  if (!momentOfInertia || !vectorWorld) {
    return new Vector3();
  }
  const invTensor = momentOfInertia.invInertiaTensor
    ?? momentOfInertia.inverseInertiaTensor
    ?? diagonalMatrix3(momentOfInertia.invInertia ?? 0.0);
  const localVector = toLocalVector(orientation, vectorWorld);
  const localResult = multiplyMatrix3Vector(invTensor, localVector);
  return toWorldVector(orientation, localResult);
}

export function inverseInertiaQuadraticForm(momentOfInertia, orientation, vectorWorld) {
  if (!momentOfInertia || !vectorWorld) {
    return 0.0;
  }
  const applied = applyWorldInverseInertia(momentOfInertia, orientation, vectorWorld);
  const value = vectorWorld.dot(applied);
  return Number.isFinite(value) && value > EPSILON ? value : 0.0;
}

export function hasAnyInverseInertia(momentOfInertia) {
  const inv = momentOfInertia?.invInertiaTensor ?? momentOfInertia?.inverseInertiaTensor;
  if (!inv) {
    return (momentOfInertia?.invInertia ?? 0.0) > EPSILON;
  }
  for (let row = 0; row < 3; row += 1) {
    for (let col = 0; col < 3; col += 1) {
      if (Math.abs(finiteNumber(inv[row]?.[col])) > EPSILON) {
        return true;
      }
    }
  }
  return false;
}

export class MomentOfInertiaComponent {
  constructor(inertia = 1.0, options = {}) {
    this.inertiaTensor = normalizeInertiaTensor(inertia);
    this.tensor = this.inertiaTensor;
    this.invInertiaTensor = invertMatrix3(this.inertiaTensor);
    this.inverseInertiaTensor = this.invInertiaTensor;

    const axisLocal = options?.axisLocal ?? options?.axis ?? DEFAULT_AXIS;
    this.inertia = effectiveInertiaAboutLocalAxis(this, axisLocal);
    this.invInertia = this.inertia > EPSILON ? 1.0 / this.inertia : 0.0;
  }
}
