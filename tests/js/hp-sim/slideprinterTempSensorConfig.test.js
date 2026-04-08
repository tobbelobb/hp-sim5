let readFileSync;

async function importFs() {
  if (readFileSync) {
    return;
  }
  ({ readFileSync } = await import('node:fs'));
}

function readFixture(relativePath) {
  return readFileSync(relativePath, 'utf8');
}

describe('Klipper temperature simulation config', () => {
  beforeAll(async () => {
    await importFs();
  });

  test('uses shared file-backed temperature sensors instead of fake analog pins', () => {
    const heaterConfigs = [
      {
        relativePath: 'examples/klipper/hp3/printer-hp3-linux-mcu-with-buildup.cfg',
        extruderSensorPath: 'examples/klipper/hp3/extruder.temp',
        bedSensorPath: 'examples/klipper/hp3/bed.temp',
      },
      {
        relativePath: 'examples/klipper/slideprinter/printer-slideprinter-linux-mcu.cfg.incl_extruder_and_heatbed',
        extruderSensorPath: 'examples/klipper/slideprinter/extruder.temp',
        bedSensorPath: 'examples/klipper/slideprinter/bed.temp',
      },
      {
        relativePath: 'examples/klipper/slideprinter/printer-slideprinter-linux-mcu.simple.cfg.incl_extruder_and_heatbed',
        extruderSensorPath: 'examples/klipper/slideprinter/extruder.temp',
        bedSensorPath: 'examples/klipper/slideprinter/bed.temp',
      },
    ];

    for (const { relativePath, extruderSensorPath, bedSensorPath } of heaterConfigs) {
      const text = readFixture(relativePath);
      expect(text).toContain('sensor_type: temperature_host');
      expect(text).toContain(`sensor_path: ${extruderSensorPath}`);
      expect(text).toContain(`sensor_path: ${bedSensorPath}`);
      expect(text).not.toMatch(/REPLACE_ME_WITH_FAKE_ANALOG_PIN_|sensor_pin:\s*analog\d+/i);
    }
  });

  test('make-fake-pin-chip only patches fake gpio chip placeholders', () => {
    const script = readFixture('scripts/make-fake-pin-chip.sh');
    expect(script).toContain('GPIO_PLACEHOLDER="REPLACE_ME_WITH_FAKE_GPIO_CHIP_NUMBER"');
    expect(script).not.toMatch(/ANALOG_PLACEHOLDER_PREFIX|create_iio_dummy|iio_dummy|KLIPPER_IIO_|analog\d+/i);
  });
});
