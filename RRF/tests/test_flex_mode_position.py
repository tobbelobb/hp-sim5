"""Exercise the real M666/standstill pipeline and captured CAN motion.

Run from the repo root with .venv/bin/python RRF/tests/test_flex_mode_position.py.
Build RRF/build/rrf_simulator first.
"""

import csv
import math
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
POSITION = re.compile(r'^X:([-\d.]+) Y:([-\d.]+) Z:([-\d.]+)(?: U:([-\d.]+))?.* Count ([-\d ]+) Machine', re.M)


class FlexModePositionTest(unittest.TestCase):
    def run_gcode(self, code, config='config_hp4.g', hidden_anchor=False, allow_error=False):
        with tempfile.TemporaryDirectory(prefix='rrf-flex-') as directory:
            vsd = Path(directory)
            shutil.copytree(ROOT / 'RRF/run/vsd/sys', vsd / 'sys')
            if hidden_anchor:
                config_path = vsd / 'sys' / config
                config_path.write_text(config_path.read_text().replace('P4 ;', 'P3 ;'))
            (vsd / 'gcodes').mkdir()
            (vsd / 'logs').mkdir()
            # The host file-input reader needs trailing data to execute short files.
            (vsd / 'gcodes/test.gcode').write_text(code + '\n' + '; padding\n' * 1000)
            result = subprocess.run([
                str(ROOT / 'RRF/build/rrf_simulator'), '--vsd', str(vsd),
                '--gcode', 'gcodes/test.gcode', '-c', f'sys/{config}',
                '--can-log', 'logs/test.csv',
            ], capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            positions = [tuple(float(value or 0) for value in match[:4]) for match in POSITION.findall(result.stdout)]
            counts = [tuple(map(int, match[4].split())) for match in POSITION.findall(result.stdout)]
            with (vsd / 'logs/test.csv').open() as capture:
                next(capture)  # JSON metadata
                motion = list(csv.reader(capture))
            if not allow_error:
                self.assertNotIn('Error:', result.stdout)
            return positions, counts, motion, result.stdout

    def test_cube_corner_mode_requires_exactly_eight_anchors(self):
        _, _, _, output = self.run_gcode('M666\nM669 N7\nM666\n', 'config_cubecorners.g')
        self.assertIn('M666 A3', output)
        self.assertIn('U1:1:1:1:1:1:1:1', output)
        _, _, _, output = self.run_gcode('M666 A3\nM666\n')
        self.assertIn('M666 A1', output)

    def test_cube_corner_moves_stay_inside_cube_or_lower_prism(self):
        for invalid in ('X1300', 'Z1950'):
            positions, _, _, output = self.run_gcode(
                'G28\nG1 X0 Y0 Z-500 F1200\nM400\nM114\n'
                f'G1 {invalid}\nM400\nM114\n', 'config_cubecorners.g')
            # RRF cancels the file at an unreachable move, retaining its last pose.
            self.assertEqual(len(positions), 1, output)
            self.assertIn('Cancelled printing file', output)
            for actual, wanted in zip(positions[0][:3], (0.,0.,-500.)):
                self.assertAlmostEqual(actual, wanted, delta=.1)

    def test_planar_all_top_forward_keeps_the_lower_branch(self):
        positions, _, _, output = self.run_gcode(
            'M666 F0\nM669 A1000:-1000:1900 B1000:1000:1900 C-1000:1000:1900 D-1000:-1000:1900\n'
            'G28\nG1 X100 Y200 Z300 F1200\nM400\nM114\nM666 F1\nM114\n', 'config_skycam.g')
        self.assertEqual(len(positions), 2, output)
        for position in positions:
            for actual, wanted in zip(position[:3], (100.,200.,300.)):
                self.assertAlmostEqual(actual, wanted, delta=.1)

    def assert_same_pose(self, before, after):
        for axis, (start, end) in enumerate(zip(before, after)):
            # Motor-step quantisation and the Hangprinter FK tolerance remain.
            self.assertAlmostEqual(start, end, delta=0.05, msg=f'axis {axis}: {before} -> {after}')

    def test_five_anchor_motion_and_flex_modes(self):
        for machine in ('hp5', 'hp5_pulley_reduced'):
            with self.subTest(machine=machine):
                positions, counts, motion, output = self.run_gcode(
                    'G28\nM669\nM666 F1\nG1 X300 Y-200 Z50 F1200\nM400\nM114\n'
                    'M666 F2\nM114\nG1 X-200 Y300 Z100\nM400\nM114\n', f'config_{machine}.g')
                self.assertEqual(len(positions), 3, output)
                self.assert_same_pose(positions[0], positions[1])
                self.assertEqual(counts[0], counts[1])
                # Direct-spool motor steps are 0.059 mm; FK also sees the
                # independent rounding of five lengths. Allow two steps.
                for actual, wanted in zip(positions[0][:3], (300., -200., 50.)):
                    self.assertAlmostEqual(actual, wanted, delta=.1)
                for actual, wanted in zip(positions[2][:3], (-200., 300., 100.)):
                    self.assertAlmostEqual(actual, wanted, delta=.1)
                self.assertTrue(any(row[1] == '44' and abs(float(row[6])) > 0 for row in motion))

    def test_spool_storage_excludes_buildup(self):
        _, _, _, output = self.run_gcode('M666 Q0 K0.015 V1\nM666\n')
        self.assertIn('Q0.0000 K0.0150 V1', output)
        # A rejected command changes nothing, not even its other parameters.
        _, _, _, output = self.run_gcode('M666 Q0 K0.015 V1\nM666 Q0.007 R31:31:31:31\nM666\nM666 V2\nM666 V1.5\n'
                                         'M666 K-1\nM666 K1e99\nM666 Q0.007 K0 V0 R31:\nM666\nM666 V0 K0 Q0.007\nM666\n',
                                         allow_error=True)
        # (A malformed array can still overwrite its own leading values, as for any M666 array.)
        replies = re.findall(r'^M666 A1 (Q\S+ K\S+ V\d)$', output, re.M)
        self.assertEqual(replies, ['Q0.0000 K0.0150 V1'] * 2 + ['Q0.0070 K0.0000 V0'], output)
        self.assertIn('R30.00:30.00:30.00:30.00', output.split('M666 A1')[1])

    def test_spool_storage_survives_m500_and_m501(self):
        with tempfile.TemporaryDirectory(prefix='rrf-m500-') as directory:
            vsd = Path(directory)
            shutil.copytree(ROOT / 'RRF/run/vsd/sys', vsd / 'sys')
            (vsd / 'gcodes').mkdir()
            (vsd / 'logs').mkdir()
            outputs = []
            for code in ('M666 K0.02 V1 W0.081\nM500\n', 'M666 K0 V0 W1\nM501\nM666\n'):
                (vsd / 'gcodes/test.gcode').write_text(code + '; padding\n' * 1000)
                outputs.append(subprocess.run([str(ROOT / 'RRF/build/rrf_simulator'), '--vsd', str(vsd),
                    '--gcode', 'gcodes/test.gcode', '-c', 'sys/config_hp4.g', '--can-log', 'logs/test.csv'],
                    capture_output=True, text=True, timeout=30, check=True).stdout)
            saved = (vsd / 'sys/config-override.g').read_text()
        hangprinter = saved[saved.index('; Hangprinter parameters'):saved.index('; Workplace')]
        self.assertTrue(all(len(line) < 160 for line in hangprinter.splitlines()), hangprinter)
        self.assertIn('M666 U2:2:2:4\n', hangprinter)
        self.assertIn('Q0.0000 K0.0200 V1', outputs[1])
        self.assertRegex(outputs[1], r'\bW0\.08100\b')

    def test_spool_storage_forward_transform_matches_targets(self):
        code = 'G28\nM666 F1\nG1 X400 Y-300 Z50 F3000\nM400\nM114\nG1 X-500 Y450 Z300\nM400\nM114\n'
        for machine in ('hp3', 'hp4', 'hp5', 'hp5_pulley_reduced'):
            with self.subTest(machine=machine):
                runs = [self.run_gcode(f'M666 Q0 {model}\n' + code, f'config_{machine}.g')
                        for model in ('K0 V0', 'K0.015 V1')]
                for positions, _, _, output in runs:
                    self.assertEqual(len(positions), 2, output)
                    for position, wanted in zip(positions, ((400., -300., 50.), (-500., 450., 300.))):
                        for actual, expected in zip(position[:3], wanted):
                            self.assertAlmostEqual(actual, expected, delta=.03)
                # The spool model moves some motor by at least two steps, so the
                # forward transform must invert it to recover the same pose.
                shift = max(abs(a - b) for a, b in zip(runs[0][1][1], runs[1][1][1]))
                self.assertGreaterEqual(shift, 2, (runs[0][1], runs[1][1]))

    def test_reported_weight_preserves_light_mover_precision(self):
        _, _, _, output = self.run_gcode('M666 W0.081\nM666\n')
        self.assertRegex(output, r'\bW0\.08100\b')

    def test_enable_at_home_does_not_move_or_shift_position(self):
        for machine in ('hp3', 'hp4', 'slideprinter', 'skycam', 'cubecorners'):
            for layers in ('', '_w_line_layers'):
                config = f'config_{machine}{layers}.g'
                with self.subTest(config=config):
                    positions, counts, motion, output = self.run_gcode('G28\nM400\nM114\nM666 F1\nM400\nM114', config)
                    self.assertEqual(len(positions), 2, output)
                    self.assert_same_pose(*positions)
                    self.assertEqual(counts[0], counts[1], 'M666 must not rebase motor counters')
                    self.assertEqual(motion, [], 'M666 must not queue motion')

    def test_disable_and_algorithm_changes_preserve_position(self):
        positions, _, motion, output = self.run_gcode('''G28
M114
M666 F1
M400
M114
M666 F2
M400
M114
M666 F0
M400
M114
''')
        self.assertEqual(len(positions), 4, output)
        for before, after in zip(positions, positions[1:]):
            self.assert_same_pose(before, after)
        self.assertEqual(motion, [])

    def test_repeated_invalid_and_zero_length_moves_preserve_pending_position(self):
        code = 'G28\nM666 F1\nM400\nM114\n'
        code += 'M666 F1\nM400\nM114\n' * 5
        code += 'M666 F3\nG1 X0 Y0 Z0 F60\nM400\nM114\n'
        positions, counts, motion, output = self.run_gcode(code)
        self.assertEqual(len(positions), 7, output)
        self.assertTrue(all(count == counts[0] for count in counts), counts)
        self.assertTrue(all(pos == positions[0] for pos in positions), positions)
        self.assertEqual(motion, [])

    def test_changes_away_from_home_preserve_position_and_extruder(self):
        positions, _, motion, output = self.run_gcode('''G28
G1 X100 Y-50 Z25 F1200
M400
G92 E7
M114
M666 F1
M400
M114
M666 F2
M400
M114
M666 F0
M400
M114
''')
        self.assertEqual(len(positions), 4, output)
        for before, after in zip(positions, positions[1:]):
            self.assert_same_pose(before, after)
        self.assertEqual(output.count('E:7.000'), 4, output)
        self.assertTrue(motion, 'the setup travel must produce real motion')
        reference = self.run_gcode('G28\nG1 X100 Y-50 Z25 F1200\nM400')[2]
        self.assertEqual(motion, reference, 'flex-mode changes must not add motion')

    def test_next_xy_move_keeps_z_and_emits_motion(self):
        positions, counts, motion, output = self.run_gcode('''G28
M666 F1
M400
M114
G1 X10 Y10 F1200
M400
M114
''')
        self.assertEqual(len(positions), 2, output)
        self.assertAlmostEqual(positions[-1][0], 10, delta=0.05)
        self.assertAlmostEqual(positions[-1][1], 10, delta=0.05)
        self.assertAlmostEqual(positions[-1][2], 0, delta=0.05)
        self.assertEqual(positions[-1][3], 0)
        self.assertTrue(motion)
        self.assertEqual(counts[0], (0, 0, 0, 0), 'keep the initial tension correction for the first move')
        for drive, endpoint in zip(range(40, 44), counts[-1]):
            steps = sum(int(row[6]) for row in motion if int(row[1]) == drive)
            self.assertAlmostEqual(steps, endpoint, delta=1)

    def test_disable_after_compensated_move_preserves_position(self):
        positions, _, motion, output = self.run_gcode('''G28
M666 F1
G1 X100 Y-50 Z25 F1200
M400
M114
M666 F0
M400
M114
G1 X101 Y-50
M400
M114
''')
        self.assertEqual(len(positions), 3, output)
        self.assertEqual(positions[0], positions[1])
        self.assertAlmostEqual(positions[-1][0], 101, delta=0.05)
        self.assertAlmostEqual(positions[-1][2], positions[1][2], delta=0.05)
        self.assertTrue(motion)

    def test_geometry_changes_invalidate_saved_position(self):
        for command in ('M666 S20000', 'M669 P1500'):
            with self.subTest(command=command):
                positions, _, motion, output = self.run_gcode(
                    f'G28\nM666 F1\nM400\nM114\n{command}\nM400\nM114')
                self.assertEqual(len(positions), 2, output)
                self.assertEqual(positions[0][2], 0)
                self.assertGreater(positions[-1][2], 0.1)
                self.assertEqual(motion, [])

    def test_attachments_balance_moments_on_the_mover(self):
        # A soft line makes each motor step a small force, so that step
        # rounding does not hide the moments.
        spring, weight, guy_wire = 2000.0, 0.216 * 9.81, 117.2
        target = (300.0, 200.0, 0.0)
        corners = [(x, y, z) for z in (200.0, -200.0)
                   for x, y in ((200.0, -200.0), (200.0, 200.0), (-200.0, 200.0), (-200.0, -200.0))]
        anchors = [(1284.9 * x / 200.0, 1284.9 * y / 200.0, 1900.0 if z > 0.0 else -280.0) for x, y, z in corners]
        distances = [math.dist(anchor, target) for anchor in anchors]
        move = 'G1 X%g Y%g Z%g F6000\nM400\nM114' % target
        geometric = self.run_gcode(f'G28\n{move}', 'config_cubecorners.g')[1][-1]
        steps_per_mm = [steps / (distance - math.dist(anchor, (0.0, 0.0, 0.0)))
                        for steps, distance, anchor in zip(geometric, distances, anchors)]

        def moment(extra_config):
            counts = self.run_gcode(f'M666 S{spring}\n{extra_config}G28\nM666 F1\n{move}', 'config_cubecorners.g')[1][-1]
            force = [0.0, 0.0, -weight]
            moment = [0.0, 0.0, 0.0]
            for i in range(8):
                flex = (counts[i] - geometric[i]) / steps_per_mm[i]
                tension = -flex * spring / (distances[i] + guy_wire)
                self.assertGreater(tension, 2.5)
                pull = [tension * (anchors[i][axis] - target[axis]) / distances[i] for axis in range(3)]
                arm = corners[i]
                for axis in range(3):
                    force[axis] += pull[axis]
                moment[0] += arm[1] * pull[2] - arm[2] * pull[1]
                moment[1] += arm[2] * pull[0] - arm[0] * pull[2]
                moment[2] += arm[0] * pull[1] - arm[1] * pull[0]
            for value in force:
                self.assertAlmostEqual(value, 0.0, delta=0.2)
            return max(abs(value) for value in moment)  # N*mm

        self.assertLess(moment(''), 40.0)
        point_model = 'M666 E' + ':'.join(['0'] * 24) + '\n'
        self.assertGreater(moment(point_model), 100.0)

    def test_invisible_anchor_preserves_position(self):
        positions, counts, motion, output = self.run_gcode(
            'G28\nM114\nM666 F1\nM400\nM114', hidden_anchor=True)
        self.assertEqual(len(positions), 2, output)
        self.assertEqual(positions[0], positions[1])
        self.assertEqual(counts[0], counts[1])
        self.assertEqual(motion, [])

    def test_return_to_old_motor_steps_uses_new_model_after_moving(self):
        positions, _, motion, output = self.run_gcode('''G28
M666 F1
M400
M114
G1 X10 Y10 F1200
M400
G1 H2 X0 Y0 Z0 U0 F60
M400
M114
''')
        self.assertEqual(len(positions), 2, output)
        self.assertEqual(positions[0][2], 0)
        self.assertGreater(positions[-1][2], 0.1, 'the old pose snapshot must expire after movement')
        self.assertTrue(motion)


if __name__ == '__main__':
    unittest.main()
