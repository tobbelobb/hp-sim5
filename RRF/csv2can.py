import csv
from pathlib import Path
import struct
import sys


# Compact CAN blocks begin with a one-byte change mask and one-byte item count.
# Bits 0-5 indicate the shared fields present in this block, in the order below.
# Omitted fields retain their values from the previous block.
CONTEXT_FIELDS = ('Q', 'i', 'i', 'i', 'f', 'f')

# Legacy item headers store a 6-bit CAN address and a 2-bit value type.
# Bit 6 of the block mask selects a 16-bit little-endian item header:
# bits 0-6 CAN address, bits 7-9 driver index, bits 10-11 value type.
EXTENDED_MOTOR_ID = 0x40


def write_block(output, context, previous_context, items):
    extended = any(address > 63 or driver != 0 for address, driver, _ in items)
    mask = EXTENDED_MOTOR_ID if extended else 0
    header = bytearray()
    for bit, (fmt, value, previous) in enumerate(zip(CONTEXT_FIELDS, context, previous_context)):
        if value != previous:
            mask |= 1 << bit
            header.extend(struct.pack('<' + fmt, value))

    output.write(struct.pack('<BB', mask, len(items)))
    output.write(header)

    for address, driver, value_text in items:
        if '.' in value_text:
            value = float(value_text)
            value_type, value_format = 3, 'f'
        else:
            value = int(value_text)
            if -128 <= value <= 127:
                value_type, value_format = 0, 'b'
            elif -32768 <= value <= 32767:
                value_type, value_format = 1, 'h'
            else:
                value_type, value_format = 2, 'i'

        if extended:
            motor_header = address | (driver << 7) | (value_type << 10)
            output.write(struct.pack('<H' + value_format, motor_header, value))
        else:
            motor_header = address | (value_type << 6)
            output.write(struct.pack('<B' + value_format, motor_header, value))


def convert_csv_to_highly_compressed(input_path):
    output_path = Path(input_path).with_suffix('.can')
    print(f"Processing {input_path}...")

    with open(input_path, newline='') as source, output_path.open('wb') as output:
        if not source.readline().strip().startswith('{'):
            source.seek(0)

        previous_context = (None,) * len(CONTEXT_FIELDS)
        context = None
        items = []
        expected_index = 0

        for row in csv.reader(source):
            if not row:
                continue
            if int(row[0]) != expected_index:
                raise ValueError(f"Index mismatch at {expected_index}. Got {row[0]}")
            expected_index += 1

            address = int(row[1])
            values = row[6:-2]
            if not 0 <= address <= 126 or not 1 <= len(values) <= 8:
                raise ValueError(f"Invalid CAN address or driver count at row {row[0]}")

            row_context = (*map(int, row[2:6]), *map(float, row[-2:]))
            for driver, value in enumerate(values):
                if items and (row_context != context or len(items) == 255):
                    write_block(output, context, previous_context, items)
                    previous_context = context
                    items = []
                context = row_context
                items.append((address, driver, value))

        if items:
            write_block(output, context, previous_context, items)

    print(f"Success! Saved to {output_path}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python csv2can.py <input.csv>")
    else:
        convert_csv_to_highly_compressed(sys.argv[1])
