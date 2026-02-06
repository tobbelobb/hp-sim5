## Focus (2D -> 3D Cable Joints Port)
- The XPBD cable joints engine lives in `src/js/cable_joints/`; keep its 3D counterpart aligned with `src/js/cable_joints_3d/` and mirror structure/naming when porting.
- The JavaScript engine has a line‑for‑line Python port in `src/python/cable_joints/`; keep interfaces and math consistent across JS/Python when you change core behavior.
- Reuse the existing math helpers (`vector2.js`/`vector3.js`, `geometry.js`/`geometry3.js`) instead of inventing new ones.
- Preserve cable path semantics (link ordering, rolling vs attachment links, stored length on wheels); changes here ripple across demos and tests.
