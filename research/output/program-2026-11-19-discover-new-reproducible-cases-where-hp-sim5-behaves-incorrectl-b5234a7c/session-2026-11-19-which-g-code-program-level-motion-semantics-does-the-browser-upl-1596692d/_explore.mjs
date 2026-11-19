import { measure, printMeasurement } from './harness.mjs';

const cases = {
  'baseline abs+E': 'M82\nG1 X10 Y0 E1 F600\nG1 X20 Y0 E2 F600\n',
  'G91 minimal': 'M82\nG91\nG1 X10 Y0 E1 F600\nG1 X10 Y0 E2 F600\n',
  'G91 square': 'M82\nG91\nG1 X10 Y0 E0.5 F600\nG1 X0 Y10 E1.0 F600\nG1 X-10 Y0 E1.5 F600\nG1 X0 Y-10 E2.0 F600\n',
  'G0 realistic': 'M82\nG0 X10 Y0 F9000\nG1 X10 Y10 E1 F600\nG0 X20 Y10 F9000\nG1 X20 Y20 E2 F600\n',
  'G0 corner with E': 'M82\nG0 X10 Y0 E1 F600\nG1 X10 Y10 E2 F600\n',
  'G0 collinear (manager)': 'M82\nG0 X10 Y0 E1 F600\nG1 X20 Y0 E2 F600\n',
  'G0 carries F': 'M82\nG0 F9000\nG1 X10 Y0 E1\n',
  'G01 leading zero': 'M82\nG01 X10 Y0 E1 F600\nG01 X20 Y0 E2 F600\n',
  'N + checksum': 'M82\nN10 G1 X10 Y0 E1 F600*63\nN20 G1 X20 Y0 E2 F600*65\n',
  'G20 inches': 'M82\nG20\nG1 X1 Y0 E1 F600\nG1 X1.5 Y0 E2 F600\n',
  'G2 trailing arc': 'M82\nG1 X10 Y0 E1 F600\nG2 X20 Y10 I10 J0 E2 F600\n',
  'G3 mid-file arc': 'M82\nG3 X20 Y0 I10 J0 E2 F600\nG1 X20 Y20 E3 F600\n',
  'G4 dwell': 'M82\nG1 X10 Y0 E1 F600\nG4 P1000\nG1 X20 Y0 E2 F600\n',
  'no-F default': 'M82\nG1 X10 Y0 E1\nG1 X20 Y0 E2\n',
  'two commands one line': 'M82 G1 X10 Y0 E1 F600\nG1 X20 Y0 E2 F600\n',
  'CRLF + no trailing NL': 'M82\r\nG1 X10 Y0 E1 F600\r\nG1 X20 Y0 E2 F600',
  'lowercase': 'm82\ng1 x10 y0 e1 f600\ng1 x20 y0 e2 f600\n',
};

for (const [name, text] of Object.entries(cases)) {
  const m = await measure(text);
  printMeasurement(name, m);
  console.log();
}
