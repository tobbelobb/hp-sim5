import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import { getAttribute } from '../../../src/js/usd/stage.js';
import { normalizeSpoolAxisLocal } from '../hangprinter_spools.js';

const DEFAULT_PLANE_NORMAL = new Vector3(0, 0, 1);

export function normalizeToPlainArray(value) {
    if (Array.isArray(value)) {
        return value;
    }
    if (value && typeof ArrayBuffer !== 'undefined' && ArrayBuffer.isView && ArrayBuffer.isView(value)) {
        return Array.from(value);
    }
    return value;
}

export function readNumericAttribute(primNode, attributeName) {
    const rawValue = getAttribute(primNode, attributeName);
    if (typeof rawValue === 'number') {
        return Number.isFinite(rawValue) ? rawValue : null;
    }
    if (rawValue === null || rawValue === undefined) {
        return null;
    }
    const parsed = Number(rawValue);
    return Number.isFinite(parsed) ? parsed : null;
}

export function readVectorAttribute(primNode, attributeName) {
    const rawValue = getAttribute(primNode, attributeName);
    if (!rawValue || !Array.isArray(rawValue) || rawValue.length < 2) {
        return null;
    }
    const x = Number(rawValue[0]);
    const y = Number(rawValue[1]);
    const z = Number(rawValue[2] ?? 0.0);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
        return null;
    }
    return new Vector3(x, y, z);
}

export function readQuaternionAttribute(primNode, attributeName) {
    const rawValue = getAttribute(primNode, attributeName);
    if (!rawValue || !Array.isArray(rawValue) || rawValue.length < 4) {
        return null;
    }
    const w = Number(rawValue[0]);
    const x = Number(rawValue[1]);
    const y = Number(rawValue[2]);
    const z = Number(rawValue[3]);
    if (!Number.isFinite(w) || !Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
        return null;
    }
    return new Quaternion(x, y, z, w).normalize();
}

export function readRotateABCAttribute(primNode, attributeName, rotationOrder) {
    const rawValue = getAttribute(primNode, attributeName);
    if (!rawValue || !Array.isArray(rawValue) || rawValue.length < 3) {
        return null;
    }
    const xDeg = Number(rawValue[0]);
    const yDeg = Number(rawValue[1]);
    const zDeg = Number(rawValue[2]);
    if (!Number.isFinite(xDeg) || !Number.isFinite(yDeg) || !Number.isFinite(zDeg)) {
        return null;
    }
    const degToRad = Math.PI / 180.0;
    const axisRotations = {
        X: new Quaternion().setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), xDeg * degToRad),
        Y: new Quaternion().setFromAxisAngle(new Vector3(0.0, 1.0, 0.0), yDeg * degToRad),
        Z: new Quaternion().setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), zDeg * degToRad),
    };
    const combined = new Quaternion();
    for (const axisName of rotationOrder) {
        const axisRotation = axisRotations[axisName];
        if (!axisRotation) {
            return null;
        }
        combined.multiplyQuaternions(axisRotation, combined).normalize();
    }
    return combined;
}

export function readRotationOpAttribute(primNode, opName) {
    if (typeof opName !== 'string') {
        return null;
    }
    const normalizedOpName = opName.startsWith('!invert!') ? opName.slice('!invert!'.length) : opName;
    if (/^xformOp:orient(?:[:].+)?$/.test(normalizedOpName)) {
        return readQuaternionAttribute(primNode, normalizedOpName);
    }
    const rotateMatch = normalizedOpName.match(/^xformOp:rotate(XYZ|XZY|YXZ|YZX|ZXY|ZYX)(?:[:].+)?$/);
    return rotateMatch ? readRotateABCAttribute(primNode, normalizedOpName, rotateMatch[1]) : null;
}

export function readOrientationAttribute(primNode) {
    const xformOpOrder = normalizeToPlainArray(getAttribute(primNode, 'xformOpOrder'));
    const rotationOrder = Array.isArray(xformOpOrder) ? xformOpOrder : [];
    if (rotationOrder.length > 0) {
        const combined = new Quaternion();
        let found = false;
        for (const opName of rotationOrder) {
            const opRotation = readRotationOpAttribute(primNode, opName);
            if (!opRotation) {
                continue;
            }
            combined.multiplyQuaternions(opRotation, combined).normalize();
            found = true;
        }
        if (found) {
            return combined;
        }
    }
    return (
        readQuaternionAttribute(primNode, 'xformOp:orient')
        || readRotateABCAttribute(primNode, 'xformOp:rotateXYZ', 'XYZ')
        || readRotateABCAttribute(primNode, 'xformOp:rotateXZY', 'XZY')
        || readRotateABCAttribute(primNode, 'xformOp:rotateYXZ', 'YXZ')
        || readRotateABCAttribute(primNode, 'xformOp:rotateYZX', 'YZX')
        || readRotateABCAttribute(primNode, 'xformOp:rotateZXY', 'ZXY')
        || readRotateABCAttribute(primNode, 'xformOp:rotateZYX', 'ZYX')
    );
}

export function readSpoolAxisLocal(primNode) {
    const axis =
        readVectorAttribute(primNode, 'spool:axisLocal')
        || readVectorAttribute(primNode, 'machine:axisLocal')
        || readVectorAttribute(primNode, 'physics:rotationAxis')
        || DEFAULT_PLANE_NORMAL;
    return normalizeSpoolAxisLocal(axis);
}

export function effectiveInertiaAboutAxis(inertiaTensor, axisLocal) {
    if (!Array.isArray(inertiaTensor) || inertiaTensor.length < 3) {
        return null;
    }
    const axis = normalizeSpoolAxisLocal(axisLocal);
    const rows = inertiaTensor.map((row) => Array.isArray(row) ? row : []);
    const xx = Number(rows[0][0] ?? 0.0);
    const xy = Number(rows[0][1] ?? 0.0);
    const xz = Number(rows[0][2] ?? 0.0);
    const yx = Number(rows[1][0] ?? 0.0);
    const yy = Number(rows[1][1] ?? 0.0);
    const yz = Number(rows[1][2] ?? 0.0);
    const zx = Number(rows[2][0] ?? 0.0);
    const zy = Number(rows[2][1] ?? 0.0);
    const zz = Number(rows[2][2] ?? 0.0);
    const projectedX = (xx * axis.x) + (xy * axis.y) + (xz * axis.z);
    const projectedY = (yx * axis.x) + (yy * axis.y) + (yz * axis.z);
    const projectedZ = (zx * axis.x) + (zy * axis.y) + (zz * axis.z);
    const inertia =
        (axis.x * projectedX)
        + (axis.y * projectedY)
        + (axis.z * projectedZ);
    return Number.isFinite(inertia) ? inertia : null;
}

export function parseRigidGroupRenderSegments(rawValue) {
    if (rawValue === null || rawValue === undefined) {
        return null;
    }
    let parsed = rawValue;
    if (typeof parsed === 'string') {
        const trimmed = parsed.trim();
        if (!trimmed) {
            return null;
        }
        try {
            parsed = JSON.parse(trimmed);
        } catch (err) {
            console.warn('setupScene: failed to parse rigidGroup:renderIndices JSON', err);
            return null;
        }
    }
    parsed = normalizeToPlainArray(parsed);
    if (!Array.isArray(parsed)) {
        return null;
    }

    const segments = [];
    const pushSequence = (sequence) => {
        const flattened = normalizeToPlainArray(sequence);
        if (!Array.isArray(flattened)) {
            return;
        }
        const indices = [];
        for (const value of flattened) {
            const asNumber = Number(value);
            if (Number.isInteger(asNumber)) {
                indices.push(asNumber);
            }
        }
        if (indices.length >= 2) {
            segments.push(indices);
        }
    };

    const hasNested = parsed.some((entry) => Array.isArray(normalizeToPlainArray(entry)));
    if (hasNested) {
        for (const entry of parsed) {
            pushSequence(entry);
        }
    } else {
        pushSequence(parsed);
    }

    return segments.length > 0 ? segments : null;
}
