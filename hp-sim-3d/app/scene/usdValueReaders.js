import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
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

function readVectorAttribute(primNode, attributeName) {
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

export function readSpoolAxisLocal(primNode) {
    const axis =
        readVectorAttribute(primNode, 'spool:axisLocal')
        || readVectorAttribute(primNode, 'machine:axisLocal')
        || readVectorAttribute(primNode, 'physics:rotationAxis')
        || DEFAULT_PLANE_NORMAL;
    return normalizeSpoolAxisLocal(axis);
}

// Accepts a JSON-like list of rigid body members, by index or prim path, e.g.
//   "[0,1,2,0]", "[</World/Scene/CornerSW>, </World/Scene/CornerSE>, ...]"
// or nested runs of either. Runs of two or more form one polyline; numbers
// are member positions, paths are resolved by the caller.
export function parseRenderPolyline(rawValue) {
    if (rawValue === null || rawValue === undefined) {
        return null;
    }
    let parsed = normalizeToPlainArray(rawValue);
    if (typeof parsed === 'string') {
        const trimmed = parsed.trim();
        if (!trimmed) {
            return null;
        }
        parsed = scanRenderReferenceList(trimmed);
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
        const entries = [];
        for (const value of flattened) {
            const entry = normalizeRenderReference(value);
            if (entry !== null) {
                entries.push(entry);
            }
        }
        if (entries.length >= 2) {
            segments.push(entries);
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

// Prim paths in a render polyline are positions in the rigid body's member list.
export function resolveRenderPolylineMembers(segments, memberPaths) {
    if (!Array.isArray(segments)) {
        return null;
    }
    const paths = Array.isArray(memberPaths) ? memberPaths : [];
    const resolved = [];
    for (const segment of segments) {
        const indices = [];
        let unresolved = null;
        for (const entry of segment) {
            if (typeof entry === 'number') {
                indices.push(entry);
                continue;
            }
            const index = paths.indexOf(entry);
            if (index < 0) {
                unresolved = entry;
                break;
            }
            indices.push(index);
        }
        if (unresolved !== null) {
            console.warn(`setupScene: rigidBody:renderPolyline references ${unresolved}, which is not a rigid body member; skipping that segment.`);
            continue;
        }
        if (indices.length >= 2) {
            resolved.push(indices);
        }
    }
    return resolved.length > 0 ? resolved : null;
}

function scanRenderReferenceList(text) {
    let cursor = 0;
    const skipWhitespace = () => {
        while (cursor < text.length && /\s/.test(text[cursor])) cursor += 1;
    };
    const readScalar = () => {
        skipWhitespace();
        const start = cursor;
        const first = text[cursor];
        if (first === '<') {
            const end = text.indexOf('>', cursor);
            cursor = end < 0 ? text.length : end + 1;
        } else if (first === '"' || first === "'") {
            const end = text.indexOf(first, cursor + 1);
            cursor = end < 0 ? text.length : end + 1;
        } else {
            while (cursor < text.length && !/[,[\]\s]/.test(text[cursor])) cursor += 1;
        }
        return text.slice(start, cursor);
    };
    const readValue = () => {
        skipWhitespace();
        if (text[cursor] !== '[') {
            return readScalar();
        }
        cursor += 1;
        const values = [];
        skipWhitespace();
        while (cursor < text.length && text[cursor] !== ']') {
            values.push(readValue());
            skipWhitespace();
            if (text[cursor] === ',') {
                cursor += 1;
                skipWhitespace();
            }
        }
        if (text[cursor] === ']') cursor += 1;
        return values;
    };
    return readValue();
}

function normalizeRenderReference(value) {
    if (typeof value === 'number') {
        return Number.isInteger(value) ? value : null;
    }
    if (typeof value !== 'string') {
        return null;
    }
    const trimmed = value.trim();
    if (!trimmed) {
        return null;
    }
    if (/^-?\d+$/.test(trimmed)) {
        return Number.parseInt(trimmed, 10);
    }
    const unwrapped = (trimmed.startsWith('<') && trimmed.endsWith('>'))
        || (trimmed.length > 1 && (trimmed[0] === '"' || trimmed[0] === "'"))
        ? trimmed.slice(1, -1).trim()
        : trimmed;
    return unwrapped.length > 0 ? unwrapped : null;
}
