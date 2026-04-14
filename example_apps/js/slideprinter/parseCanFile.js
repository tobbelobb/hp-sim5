async function parseCanFile(url) {
    const response = await fetch(url);
    const buffer = await response.arrayBuffer();
    const dataView = new DataView(buffer);

    const movements = [];
    let offset = 0;
    let globalIndex = 0;

    // "Sticky" State Context
    let ctxTime = 0n; // BigInt
    let ctxCol3 = 0;
    let ctxCol4 = 0;
    let ctxCol5 = 0;
    let ctxAccel = 0.0; // Float
    let ctxDecel = 0.0; // Float

    while (offset < buffer.byteLength) {
        // 1. Read Header Info
        const mask = dataView.getUint8(offset++);
        const count = dataView.getUint8(offset++);

        // 2. Update Sticky Context based on Mask
        // Bit 0: Time (8 bytes, Uint64)
        if (mask & 1) {
            ctxTime = dataView.getBigUint64(offset, true);
            offset += 8;
        }
        // Bit 1: Col3 (4 bytes, Int32)
        if (mask & 2) {
            ctxCol3 = dataView.getInt32(offset, true);
            offset += 4;
        }
        // Bit 2: Col4 (4 bytes, Int32)
        if (mask & 4) {
            ctxCol4 = dataView.getInt32(offset, true);
            offset += 4;
        }
        // Bit 3: Col5 (4 bytes, Int32)
        if (mask & 8) {
            ctxCol5 = dataView.getInt32(offset, true);
            offset += 4;
        }
        // Bit 4: Accel (4 bytes, Float32)
        if (mask & 16) {
            ctxAccel = dataView.getFloat32(offset, true);
            offset += 4;
        }
        // Bit 5: Decel (4 bytes, Float32)
        if (mask & 32) {
            ctxDecel = dataView.getFloat32(offset, true);
            offset += 4;
        }

        // 3. Read Body Items
        for (let i = 0; i < count; i++) {
            const headerByte = dataView.getUint8(offset++);

            // Extract Bits
            const typeCode = (headerByte >> 6) & 0x03; // Top 2 bits
            const canId    = headerByte & 0x3F;        // Bottom 6 bits

            let val;

            // Switch based on compressed type
            switch (typeCode) {
                case 0: // Int8 (1 byte)
                    val = dataView.getInt8(offset);
                    offset += 1;
                    break;
                case 1: // Int16 (2 bytes)
                    val = dataView.getInt16(offset, true);
                    offset += 2;
                    break;
                case 2: // Int32 (4 bytes)
                    val = dataView.getInt32(offset, true);
                    offset += 4;
                    break;
                case 3: // Float32 (4 bytes)
                    val = dataView.getFloat32(offset, true);
                    offset += 4;
                    break;
            }

            movements.push({
                index: globalIndex++,
                canId: canId,
                col2: ctxTime,
                col3: ctxCol3,
                col4: ctxCol4,
                col5: ctxCol5,
                accel: ctxAccel,
                decel: ctxDecel,
                val: val
            });
        }
    }

    return movements;
}

// Example usage:
// parseCanFile('data.can').then(console.log);
