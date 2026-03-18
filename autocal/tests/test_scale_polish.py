import numpy as np

from autocal.scale_polish import apply_final_scale_polish, run_uniform_scale_polish


def test_run_uniform_scale_polish_skips_when_find_radii_off():
    radii = np.asarray([39.0, 39.0, 39.0], dtype=float)
    anchors = np.asarray([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]], dtype=float)

    same_radii, same_anchors, _spool, _dataset, cost, info = run_uniform_scale_polish(
        search_r=False,
        num_anchors=3,
        radii_mm=radii,
        buildup_factor=np.zeros(3, dtype=float),
        anchors_eval=anchors,
        prior_cost=lambda *_args, **_kwargs: 0.0,
        data_cost=lambda *_args, **_kwargs: 12.5,
        build_dataset_and_params=lambda radii_mm, buildup_factor: (
            {"radii": radii_mm.copy(), "buildup": buildup_factor.copy()},
            {"dataset": True},
        ),
        radii_respect_bounds=lambda _radii: True,
        uniform_radius_scale=lambda _new, _old: 1.0,
    )

    assert np.array_equal(same_radii, radii)
    assert np.array_equal(same_anchors, anchors)
    assert cost == 12.5
    assert info["accepted"] is False
    assert info["message"] == "scale polish skipped (find_radii=off)"


def test_apply_final_scale_polish_rejects_rank_regression():
    best = {
        "cost": 10.0,
        "total_cost": 10.0,
        "rank_score": 1.0,
        "rank_internal": 1.0,
        "radii_mm": np.asarray([39.0, 39.0, 39.0], dtype=float),
        "buildup_factor": np.asarray([0.0, 0.0, 0.0], dtype=float),
        "anchors": np.asarray([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]], dtype=float),
        "spool_params": {"ok": True},
        "dataset": {"ok": True},
    }

    updated, info = apply_final_scale_polish(
        best=best,
        use_scale_fix_2=True,
        use_scale_fix_3=False,
        prior_cost=lambda *_args, **_kwargs: 0.0,
        spool_rank_score=lambda *_args, **_kwargs: (2.0, 2.0),
        rank_better=lambda rank_try, total_try, rank_ref, total_ref: (
            rank_try < rank_ref or (rank_try == rank_ref and total_try < total_ref)
        ),
        run_uniform_scale_polish_fn=lambda **_kwargs: (
            np.asarray([38.5, 38.5, 38.5], dtype=float),
            np.asarray([[0.0, 0.0], [0.9, 0.0], [0.0, 0.9]], dtype=float),
            {"polished": True},
            {"dataset": "polished"},
            5.0,
            {
                "attempted": True,
                "success": True,
                "accepted": True,
                "message": "scale polish improved total objective",
                "start_data_cost": 10.0,
                "best_data_cost": 5.0,
            },
        ),
    )

    assert updated is best
    assert info["accepted"] is False
    assert info["accepted_total_objective"] is True
    assert info["rank_better"] is False
    assert info["message"] == "scale polish rejected by rank objective"
