# Wrong machine geometry in USD scene loading and forward-transform measurement units

Four reproducible defects: HP4, HP4-worn-bearings and HP5-pulley-reduced wheels are 0.20–3.09 m from their USD-authored positions because `xformOpOrder` is ignored; deleting one non-geometric spool attribute removes a motor and leaves 2.5 m-dangling cable joints before the first physics step throws; `metersPerUnit` is ignored, so a valid millimetre scene loads 1000× too large; and the forward-transform dataset noise/bias sweeps are 25.5× (A–D) to 51× (I) weaker than their labels. Forward round-trips and the catalogued anchor/spool/firmware presets otherwise agree.

Topics: USD, scene loading, transforms, unit handling, HP4, HP5, catalog presets, forward transform.

See [report](report.md) for evidence, negative results and limitations.
