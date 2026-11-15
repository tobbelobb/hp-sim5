import pytest

from autocal.tools.sanity_battery import anchor_errors


def test_anchor_errors_remove_common_translation():
    initial = 'A:0, -2, -1\nB:2, 1, -1\nC:-2, 1, -1\nD:0, 0, 3'
    final = 'M669 A1:0:2 B3:3:2 C-1:3:2 D1:2:6'
    result = anchor_errors(initial, final)
    assert result['mean_translation_mm'] == [1, 2, 3]
    assert result['max_abs_coordinate_mm'] == 3
    assert result['max_aligned_abs_coordinate_mm'] == 0


def test_anchor_errors_reject_missing_anchor():
    with pytest.raises(ValueError, match='missing anchor'):
        anchor_errors('A:0,0,0', 'A:0,0,0')
