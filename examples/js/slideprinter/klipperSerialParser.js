import klipperDictRaw from '../../klipper/linux_mcu/klipper.dict?raw';

const baseDictionary = JSON.parse(klipperDictRaw);

export const MESSAGE_MIN = 5;
export const MESSAGE_MAX = 64;
export const MESSAGE_HEADER_SIZE = 2;
export const MESSAGE_TRAILER_SIZE = 3;
export const MESSAGE_TRAILER_SYNC = 1;
export const MESSAGE_TRAILER_CRC = 3;
export const MESSAGE_SEQ_MASK = 0x0f;
export const MESSAGE_DEST = 0x10;
export const MESSAGE_SYNC = 0x7e;

function crc16Ccitt(bytes, length) {
  const len = typeof length === 'number' ? length : bytes.length;
  let crc = 0xffff;
  for (let i = 0; i < len; i += 1) {
    let data = bytes[i];
    data ^= crc & 0xff;
    data ^= (data & 0x0f) << 4;
    crc = (((data << 8) & 0xffff) | ((crc >> 8) & 0xff)) ^ (data >> 4) ^ ((data << 3) & 0xffff);
  }
  return [(crc >> 8) & 0xff, crc & 0xff];
}

function parseVarint(bytes, pos, { signed } = { signed: false }) {
  if (pos >= bytes.length) {
    throw new Error('Truncated varint');
  }
  let c = bytes[pos];
  pos += 1;
  let v = BigInt(c & 0x7f);
  if (signed && (c & 0x60) === 0x60) {
    v |= BigInt(-0x20);
  }
  while (c & 0x80) {
    if (pos >= bytes.length) {
      throw new Error('Truncated varint continuation');
    }
    c = bytes[pos];
    pos += 1;
    v = (v << 7n) | BigInt(c & 0x7f);
  }
  if (signed) {
    return { value: Number(v), pos };
  }
  const mask = 0xffffffffn;
  const u = Number(v & mask);
  return { value: u >>> 0, pos };
}

function encodeInt32(value) {
  const out = [];
  let v = Number(value);
  if (v >= 0xc000000 || v < -0x4000000) {
    out.push(((v >> 28) & 0x7f) | 0x80);
  }
  if (v >= 0x180000 || v < -0x80000) {
    out.push(((v >> 21) & 0x7f) | 0x80);
  }
  if (v >= 0x3000 || v < -0x1000) {
    out.push(((v >> 14) & 0x7f) | 0x80);
  }
  if (v >= 0x60 || v < -0x20) {
    out.push(((v >> 7) & 0x7f) | 0x80);
  }
  out.push(v & 0x7f);
  return out;
}

class UInt32Type {
  constructor() {
    this.isDynamicString = false;
  }
  parse(bytes, pos) {
    return parseVarint(bytes, pos, { signed: false });
  }
  toString(value) {
    return String(value);
  }
}

class Int32Type {
  constructor() {
    this.isDynamicString = false;
  }
  parse(bytes, pos) {
    return parseVarint(bytes, pos, { signed: true });
  }
  toString(value) {
    return String(value);
  }
}

class UInt16Type extends UInt32Type {}
class Int16Type extends Int32Type {}
class ByteType extends UInt32Type {}

class BufferType {
  constructor() {
    this.isDynamicString = true;
    this.decoder = typeof TextDecoder !== 'undefined' ? new TextDecoder('utf-8', { fatal: false }) : null;
  }
  parse(bytes, pos) {
    if (pos >= bytes.length) {
      throw new Error('Truncated buffer length');
    }
    const length = bytes[pos];
    const start = pos + 1;
    const end = start + length;
    if (end > bytes.length) {
      throw new Error('Truncated buffer payload');
    }
    const slice = bytes.subarray(start, end);
    return { value: slice, pos: end };
  }
  toString(value) {
    if (value == null) {
      return '';
    }
    if (this.decoder) {
      try {
        return JSON.stringify(this.decoder.decode(value));
      } catch (_) {
        // fall through to hex formatting
      }
    }
    const hex = Array.from(value, (b) => b.toString(16).padStart(2, '0')).join('');
    return `0x${hex}`;
  }
}

class EnumerationType {
  constructor(innerType, enumName, reverseMap) {
    this.innerType = innerType;
    this.enumName = enumName;
    this.reverseMap = reverseMap;
    this.isDynamicString = innerType.isDynamicString;
  }
  parse(bytes, pos) {
    const { value, pos: nextPos } = this.innerType.parse(bytes, pos);
    const mapped = this.reverseMap.get(value);
    if (mapped !== undefined) {
      return { value: mapped, pos: nextPos };
    }
    return { value: `?${value}`, pos: nextPos };
  }
  toString(value) {
    return String(value);
  }
}

function createBaseType(fmt) {
  switch (fmt) {
    case '%u':
      return new UInt32Type();
    case '%i':
      return new Int32Type();
    case '%hu':
      return new UInt16Type();
    case '%hi':
      return new Int16Type();
    case '%c':
      return new ByteType();
    case '%s':
    case '%.*s':
    case '%*s':
      return new BufferType();
    default:
      throw new Error(`Unsupported format specifier: ${fmt}`);
  }
}

function buildEnumerationReverse(rawEnumerations = {}) {
  const out = new Map();
  for (const [enumName, entries] of Object.entries(rawEnumerations)) {
    const map = new Map();
    for (const [key, value] of Object.entries(entries || {})) {
      if (Array.isArray(value)) {
        const [startValue, count] = value;
        let root = key;
        while (root && /\d$/.test(root)) {
          root = root.slice(0, -1);
        }
        const suffix = key.slice(root.length);
        const startIndex = suffix ? parseInt(suffix, 10) : 0;
        for (let idx = 0; idx < count; idx += 1) {
          map.set(startValue + idx, `${root}${startIndex + idx}`);
        }
      } else {
        map.set(value, key);
      }
    }
    out.set(enumName, map);
  }
  return out;
}

class MessageFormat {
  constructor(msgid, msgformat, type, enumerations) {
    this.msgid = msgid;
    this.msgformat = msgformat;
    this.type = type;
    const parts = msgformat.trim().split(/\s+/);
    this.name = parts[0];
    this.paramDefs = [];
    this.msgidBytes = encodeInt32(msgid);
    const paramParts = parts.slice(1);
    for (const param of paramParts) {
      if (!param.includes('=')) {
        continue;
      }
      const [rawName, fmt] = param.split('=');
      const name = rawName.trim();
      const baseType = createBaseType(fmt.trim());
      let typeInstance = baseType;
      for (const [enumName, reverse] of enumerations) {
        if (name === enumName || name.endsWith(`_${enumName}`)) {
          typeInstance = new EnumerationType(baseType, enumName, reverse);
          break;
        }
      }
      this.paramDefs.push({ name, type: typeInstance });
    }
  }

  parse(bytes, offset) {
    let pos = offset + this.msgidBytes.length;
    const params = {};
    for (const { name, type } of this.paramDefs) {
      const { value, pos: nextPos } = type.parse(bytes, pos);
      params[name] = value;
      pos = nextPos;
    }
    return { params, nextPos: pos };
  }

  format(params) {
    const pieces = [this.name];
    for (const { name, type } of this.paramDefs) {
      const value = params[name];
      pieces.push(`${name}=${type.toString(value)}`);
    }
    return pieces.join(' ');
  }
}

class UnknownFormat {
  constructor() {
    this.name = '#unknown';
  }

  parse(bytes, offset) {
    const payloadEnd = bytes.length - MESSAGE_TRAILER_SIZE;
    return {
      params: { '#msg': bytes.subarray(offset, payloadEnd) },
      nextPos: payloadEnd,
    };
  }

  format(params) {
    const payload = params['#msg'];
    if (!payload) {
      return '#unknown';
    }
    const hex = Array.from(payload, (b) => b.toString(16).padStart(2, '0')).join('');
    return `#unknown 0x${hex}`;
  }
}

export class KlipperSerialDecoder {
  constructor(dictionary = baseDictionary) {
    this.dictionary = dictionary;
    this.enumerations = buildEnumerationReverse(dictionary?.enumerations);
    this.messagesById = new Map();
    this.msgidParser = new Int32Type();
    this.unknown = new UnknownFormat();
    this._initMessages(dictionary?.commands, 'command');
    this._initMessages(dictionary?.responses, 'response');
    this._initMessages(dictionary?.output, 'output');
  }

  _initMessages(defs = {}, type) {
    if (!defs) {
      return;
    }
    for (const [msgformat, msgid] of Object.entries(defs)) {
      const fmt = new MessageFormat(Number(msgid), msgformat, type, this.enumerations);
      this.messagesById.set(Number(msgid), fmt);
    }
  }

  checkPacket(buffer) {
    if (!buffer || buffer.length < MESSAGE_MIN) {
      return 0;
    }
    const msglen = buffer[0];
    if (msglen < MESSAGE_MIN || msglen > MESSAGE_MAX) {
      return -1;
    }
    if (buffer.length < msglen) {
      return 0;
    }
    const seqByte = buffer[1];
    if ((seqByte & ~MESSAGE_SEQ_MASK) !== MESSAGE_DEST) {
      return -1;
    }
    if (buffer[msglen - MESSAGE_TRAILER_SYNC] !== MESSAGE_SYNC) {
      return -1;
    }
    const crcIndex = msglen - MESSAGE_TRAILER_CRC;
    const expectedCrcHi = buffer[crcIndex];
    const expectedCrcLo = buffer[crcIndex + 1];
    const [crcHi, crcLo] = crc16Ccitt(buffer.subarray(0, msglen - MESSAGE_TRAILER_SIZE));
    if (crcHi !== expectedCrcHi || crcLo !== expectedCrcLo) {
      return -1;
    }
    return msglen;
  }

  decodePacket(packet) {
    if (!packet || packet.length < MESSAGE_MIN) {
      return { seq: null, lines: [] };
    }
    const msglen = packet[0];
    const seq = packet[1] & MESSAGE_SEQ_MASK;
    const lines = [];
    let pos = MESSAGE_HEADER_SIZE;
    const end = msglen - MESSAGE_TRAILER_SIZE;
    while (pos < end) {
      const { value: msgid } = this.msgidParser.parse(packet, pos);
      const handler = this.messagesById.get(msgid) || this.unknown;
      const { params, nextPos } = handler.parse(packet, pos);
      pos = nextPos;
      const formatted = handler.format(params);
      if (formatted) {
        lines.push(formatted);
      }
      if (pos >= end) {
        break;
      }
    }
    return { seq, lines };
  }
}

let sharedDecoder = null;

function getSharedDecoder() {
  if (!sharedDecoder) {
    sharedDecoder = new KlipperSerialDecoder(baseDictionary);
  }
  return sharedDecoder;
}

function asUint8Array(chunk) {
  if (chunk instanceof Uint8Array) {
    return chunk;
  }
  if (chunk instanceof ArrayBuffer) {
    return new Uint8Array(chunk);
  }
  if (ArrayBuffer.isView(chunk)) {
    return new Uint8Array(chunk.buffer, chunk.byteOffset, chunk.byteLength);
  }
  return null;
}

function concatBuffers(existing, next) {
  if (!existing || existing.length === 0) {
    return next ? next.slice() : new Uint8Array(0);
  }
  if (!next || next.length === 0) {
    return existing;
  }
  const out = new Uint8Array(existing.length + next.length);
  out.set(existing, 0);
  out.set(next, existing.length);
  return out;
}

export class SerialLineDecoder {
  constructor(decoder) {
    this.decoder = decoder || getSharedDecoder();
    this.buffer = new Uint8Array(0);
  }

  push(chunk) {
    const data = asUint8Array(chunk);
    if (!data || data.length === 0) {
      return [];
    }
    this.buffer = concatBuffers(this.buffer, data);
    const lines = [];
    while (true) {
      const len = this.decoder.checkPacket(this.buffer);
      if (len > 0) {
        const packet = this.buffer.subarray(0, len);
        const { lines: newLines } = this.decoder.decodePacket(packet);
        if (newLines.length) {
          lines.push(...newLines);
        }
        this.buffer = this.buffer.subarray(len);
        continue;
      }
      if (len < 0) {
        this.buffer = this.buffer.subarray(1);
        continue;
      }
      break;
    }
    return lines;
  }

  flush() {
    const lines = [];
    while (this.buffer.length >= MESSAGE_MIN) {
      const len = this.decoder.checkPacket(this.buffer);
      if (len > 0) {
        const packet = this.buffer.subarray(0, len);
        const { lines: newLines } = this.decoder.decodePacket(packet);
        if (newLines.length) {
          lines.push(...newLines);
        }
        this.buffer = this.buffer.subarray(len);
      } else if (len < 0) {
        this.buffer = this.buffer.subarray(1);
      } else {
        break;
      }
    }
    this.buffer = new Uint8Array(0);
    return lines;
  }
}

export async function* iterateSerialLines(stream, decoder) {
  const activeDecoder = decoder || getSharedDecoder();
  const reader = stream.getReader();
  const lineDecoder = new SerialLineDecoder(activeDecoder);
  try {
    while (true) {
      const { value, done } = await reader.read();
      if (done) {
        break;
      }
      const chunk = asUint8Array(value);
      if (!chunk || chunk.length === 0) {
        continue;
      }
      const lines = lineDecoder.push(chunk);
      for (const line of lines) {
        yield line;
      }
    }
  } finally {
    if (reader.releaseLock) {
      reader.releaseLock();
    }
  }
  const remaining = lineDecoder.flush();
  for (const line of remaining) {
    yield line;
  }
}

export function decodeBase64Chunk(b64) {
  if (typeof b64 !== 'string' || !b64) {
    return null;
  }
  if (typeof atob !== 'function') {
    return null;
  }
  const binary = atob(b64);
  const len = binary.length;
  const bytes = new Uint8Array(len);
  for (let i = 0; i < len; i += 1) {
    bytes[i] = binary.charCodeAt(i);
  }
  return bytes;
}

export function createKlipperSerialDecoder() {
  return new KlipperSerialDecoder(baseDictionary);
}
