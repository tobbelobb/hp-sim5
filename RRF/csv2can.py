import sys
import csv
import struct
import os

# Sticky + Compact Binary Schema (v2):
# We define a Block. Groups of rows sharing the same context are combined.
# We use "Sticky" headers (only writing changed context) and variable-width values.

# 1. Block Header (Variable Size: 2 to 30 Bytes)
# | Offset | Type   | Description                                                         |
# | :---   | :---   | :---                                                                |
# |      0 | Uint8  | Change Mask                                                         |
# |        |        | - Bit 0: Time (Col 2) changed                                       |
# |        |        | - Bit 1: Col 3 changed                                              |
# |        |        | - Bit 2: Col 4 changed                                              |
# |        |        | - Bit 3: Col 5 changed                                              |
# |        |        | - Bit 4: Accel (Col 7) changed                                      |
# |        |        | - Bit 5: Decel (Col 8) changed                                      |
# |      1 | Uint8  | Count (N): How many items follow in this block?                     |
# |   2... | Var    | Conditional Fields (Only present if corresponding Bit in Mask is 1) |
# |        |        | - Bit 0: Uint64 (8 bytes)                                           |
# |        |        | - Bit 1: Int32  (4 bytes)                                           |
# |        |        | - Bit 2: Int32  (4 bytes)                                           |
# |        |        | - Bit 3: Int32  (4 bytes)                                           |
# |        |        | - Bit 4: Float32 (4 bytes)                                          |
# |        |        | - Bit 5: Float32 (4 bytes)                                          |

# 2. Block Body (Repeated N times - Variable Size: 2 to 5 Bytes each)
# | Offset | Type   | Description                                                         |
# | :---   | :---   | :---                                                                |
# |      0 | Uint8  | Bits 0-5: CAN ID (Max 63)                                           |
# |        |        | Bits 6-7: Type Code (Determines payload size)                       |
# |   1... | Var    | Column 6 Value (Size depends on Type Code)                          |

# Type Codes (Bits 6-7 of Body Byte 0):
# | Code | Binary | Data Type | Payload Size | Total Item Size |
# | :--- | :---   | :---      | :---         | :---            |
# |    0 | 00     | Int8      | 1 Byte       | 2 Bytes         |
# |    1 | 01     | Int16     | 2 Bytes      | 3 Bytes         |
# |    2 | 10     | Int32     | 4 Bytes      | 5 Bytes         |
# |    3 | 11     | Float32   | 4 Bytes      | 5 Bytes         |

def convert_csv_to_highly_compressed(input_path):
    output_path = os.path.splitext(input_path)[0] + '.can'

    print(f"Processing {input_path}...")

    with open(input_path, 'r') as f_in, open(output_path, 'wb') as f_out:
        # Check for JSON header
        pos = f_in.tell()
        if not f_in.readline().strip().startswith('{'):
            f_in.seek(pos)

        csv_reader = csv.reader(f_in)

        # State tracking for "Sticky" compression
        prev_c2 = None
        prev_c3 = None
        prev_c4 = None
        prev_c5 = None
        prev_c7 = None # Accel
        prev_c8 = None # Decel

        # Buffer for current group
        # Key includes Accel/Decel now
        current_group_key = None # (c2, c3, c4, c5, c7, c8)
        group_buffer = [] # list of (id, value_string)

        def flush_group():
            nonlocal prev_c2, prev_c3, prev_c4, prev_c5, prev_c7, prev_c8
            nonlocal current_group_key, group_buffer

            if not group_buffer:
                return

            c2, c3, c4, c5, c7, c8 = current_group_key

            # --- 1. Build Header with Bitmask ---
            mask = 0
            header_payload = bytearray()

            # Bit 0: Col2 (Time) - Uint64
            if c2 != prev_c2:
                mask |= 1
                header_payload.extend(struct.pack('<Q', c2))
                prev_c2 = c2

            # Bit 1: Col3 - Int32
            if c3 != prev_c3:
                mask |= 2
                header_payload.extend(struct.pack('<i', c3))
                prev_c3 = c3

            # Bit 2: Col4 - Int32
            if c4 != prev_c4:
                mask |= 4
                header_payload.extend(struct.pack('<i', c4))
                prev_c4 = c4

            # Bit 3: Col5 - Int32
            if c5 != prev_c5:
                mask |= 8
                header_payload.extend(struct.pack('<i', c5))
                prev_c5 = c5

            # Bit 4: Col7 (Accel) - Float32
            if c7 != prev_c7:
                mask |= 16
                header_payload.extend(struct.pack('<f', c7))
                prev_c7 = c7

            # Bit 5: Col8 (Decel) - Float32
            if c8 != prev_c8:
                mask |= 32
                header_payload.extend(struct.pack('<f', c8))
                prev_c8 = c8

            # Write Mask and Count
            count = len(group_buffer)
            f_out.write(struct.pack('<BB', mask, count))
            # Write the variable header fields
            f_out.write(header_payload)

            # --- 2. Write Body Items with Variable Width ---
            for (can_id, val_str) in group_buffer:
                can_id = int(can_id)

                if '.' in val_str:
                    # Float32 -> Type 11 (0x3)
                    val = float(val_str)
                    type_bits = 3
                    header_byte = (type_bits << 6) | (can_id & 0x3F)
                    f_out.write(struct.pack('<Bf', header_byte, val))
                else:
                    val = int(val_str)
                    if -128 <= val <= 127:
                        # Int8 -> Type 00 (0x0)
                        type_bits = 0
                        header_byte = (type_bits << 6) | (can_id & 0x3F)
                        f_out.write(struct.pack('<Bb', header_byte, val))
                    elif -32768 <= val <= 32767:
                        # Int16 -> Type 01 (0x1)
                        type_bits = 1
                        header_byte = (type_bits << 6) | (can_id & 0x3F)
                        f_out.write(struct.pack('<Bh', header_byte, val))
                    else:
                        # Int32 -> Type 10 (0x2)
                        type_bits = 2
                        header_byte = (type_bits << 6) | (can_id & 0x3F)
                        f_out.write(struct.pack('<Bi', header_byte, val))

        # --- Main Loop ---
        expected_idx = 0
        for row in csv_reader:
            if not row: continue

            # Validation
            if int(row[0]) != expected_idx:
                raise ValueError(f"Index mismatch at {expected_idx}. Got {row[0]}")
            expected_idx += 1

            # Extract keys
            # 0:Idx, 1:ID, 2:Time, 3:C3, 4:C4, 5:C5, 6:Val, 7:Accel, 8:Decel
            c2 = int(row[2])
            c3 = int(row[3])
            c4 = int(row[4])
            c5 = int(row[5])
            c7 = float(row[7])
            c8 = float(row[8])

            row_key = (c2, c3, c4, c5, c7, c8)

            if (row_key != current_group_key) or (len(group_buffer) >= 255):
                flush_group()
                current_group_key = row_key
                group_buffer = []

            group_buffer.append((row[1], row[6]))

        flush_group()

    print(f"Success! Saved to {output_path}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python csv2can.py <input.csv>")
    else:
        convert_csv_to_highly_compressed(sys.argv[1])
