from pathlib import Path

import autocal.ellipse_active as ellipse_active


def test_ellipse_active_main_passes_dataset_and_collector_args(monkeypatch, tmp_path):
    dataset = tmp_path / "demo.json"
    dataset.write_text("{}", encoding="utf-8")

    captured = {}
    printed = {}

    def fake_plan_next_ellipse_sweep(**kwargs):
        captured.update(kwargs)
        return {"plan": True}

    def fake_print(plan, **kwargs):
        printed["plan"] = plan
        printed["kwargs"] = kwargs

    monkeypatch.setattr(ellipse_active, "plan_next_ellipse_sweep", fake_plan_next_ellipse_sweep)
    monkeypatch.setattr(ellipse_active, "_print_ellipse_plan", fake_print)

    rc = ellipse_active.main(
        [
            str(dataset),
            "--find-radii",
            "global",
            "--collector-output",
            str(tmp_path / "next.json"),
            "--collector-args",
            "--return-to-origin",
        ]
    )

    assert rc == 0
    assert captured["dataset_path"] == Path(dataset)
    assert captured["find_radii"] == "global"
    assert captured["collector_args"] == ["--return-to-origin"]
    assert printed["plan"] == {"plan": True}
    assert printed["kwargs"]["output_with_explanations"] is False
