from pathlib import Path

import numpy as np
import pytest

from hp_sim5_research.reference_data import incomplete, read_scope, read_tracker

FIXTURES = Path(__file__).resolve().parents[1] / 'fixtures' / 'reference_data'

SCOPE = (
    'Name\tMeasurement\t\r\n'
    'Starttime of export\t133293940795297500\tMittwoch, 24. Mai 2023\t11:27:59.529\r\n'
    '\r\n'
    'Name\tf_T\tName\tbTrigger_Record\t\r\n'
    'Data-Type\tREAL64\tData-Type\tBIT\t\r\n'
    'SampleTime[ms]\t1\tSampleTime[ms]\t1\t\r\n'
    '\r\n'
    '0\t125.5\t0\t0\t\r\n'
    '1\t125.0\t1\t1\t\r\n'
    '2\t124.5\t2\t1\t\r\n'
)


def test_scope_export_reads_names_units_and_record_window(tmp_path):
    path = tmp_path / 'Export 1.csv'
    path.write_text(SCOPE, encoding='latin-1')
    s = read_scope(path)
    assert list(s.channels) == ['f_T', 'bTrigger_Record']
    np.testing.assert_allclose(s.t, [0, 1e-3, 2e-3])
    rec = s.recorded('bTrigger_Record')
    np.testing.assert_allclose(rec.t, [0, 1e-3])
    np.testing.assert_allclose(rec['f_T'], [125.0, 124.5])


def test_scope_export_rejects_mixed_sample_times(tmp_path):
    path = tmp_path / 'Export 1.csv'
    path.write_text(SCOPE.replace('SampleTime[ms]\t1\tSampleTime[ms]\t1', 'SampleTime[ms]\t1\tSampleTime[ms]\t10'))
    with pytest.raises(ValueError, match='sample times'):
        read_scope(path)


def test_tracker_reads_decimal_commas(tmp_path):
    path = tmp_path / 'CircPath.csv'
    path.write_text('-0,357442687; -1,913166795; 0,519448675; 1.731574947691135E9; 0,0; 0,0; 0,0\n')
    pos, t = read_tracker(path)
    np.testing.assert_allclose(pos, [[-0.357442687, -1.913166795, 0.519448675]])
    np.testing.assert_allclose(t, [1.731574947691135e9])


def test_partial_download_marks_directory_incomplete(tmp_path):
    (tmp_path / 'a').mkdir()
    assert not incomplete(tmp_path)
    (tmp_path / 'a' / 'Export 1.csv.download').touch()
    assert incomplete(tmp_path)


def test_matched_pair_fixture_covers_ten_metres_each_way():
    data = np.load(FIXTURES / 'darus3600_WF125_F120000_torque_pair.npz')
    for direction, sign in [('unwind', 1), ('rewind', -1)]:
        drum = data[f'{direction}/dPosition_test']
        assert sign * (drum[-1] - drum[0]) == pytest.approx(10.0, abs=0.05)
        assert data[f'{direction}/dForce_test'].mean() == pytest.approx(125, abs=3)


def test_darus5171_and_4614_fixtures_have_expected_coverage():
    import csv
    with (FIXTURES / 'darus5171_fdp_fc_map.csv').open() as f:
        ident = list(csv.DictReader(f))
    assert len(ident) == 75
    assert {r['rope'] for r in ident} == {'LIROS_D-Pro', 'Trowis_VectranHT_T190PU', 'Trowis_PolyTropFOX'}

    with (FIXTURES / 'darus5171_stat_valid.csv').open() as f:
        valid = list(csv.DictReader(f))
    assert len(valid) == 54
    assert {r['rope'] for r in valid} == {'LIROS_D-Pro', 'Trowis_VectranHT_T190PU', 'Trowis_PolyTropFOX'}

    with (FIXTURES / 'darus4614_planeA.csv').open() as f:
        plane_a = list(csv.DictReader(f))
    assert len(plane_a) == 48  # 8 experiments x 6 poses (P0..P5)
    assert {r['pose'] for r in plane_a} == {'P0', 'P1', 'P2', 'P3', 'P4', 'P5'}

    with (FIXTURES / 'darus5171_replay_wc180_dp75.csv').open() as f:
        replay_5171 = list(csv.DictReader(f))
    assert len(replay_5171) >= 700
    assert any(r['trig_rec'] == '1' for r in replay_5171)

    with (FIXTURES / 'darus3600_replay_pair.csv').open() as f:
        replay_3600 = list(csv.DictReader(f))
    assert {r['phase'] for r in replay_3600} == {'unwind', 'rewind'}
