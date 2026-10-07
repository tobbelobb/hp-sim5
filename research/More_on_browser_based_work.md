The hp-sim-3d js app is a usable interactive simulator, while the Python version
is primarily a headless tool for scripted runs and analysis.
Use the browser based simulation whenever you want to
 - see and control a run as it happens,
 - try a print without preparing a simulation script,
 - explore machines interactively. (The browser UI lets you choose presets, upload authored scenes, and add or remove machines.)
 - Inspect browser-specific behavior.

For shared browser work call `start_browser_service(record=True)` when recording
is needed, open its exact URL, inspect `browser_status`, and pass that page's
`page_id` to `browser_action`. Browser JS, standalone JS parity fixtures, and
native Python are distinct backends. The browser API uses the existing scene,
worker, motor, timing and inspection controllers. Pause before bounded stepping;
finish active workers before direct motor commands. The optional WebMCP site
tools operate the same API in the open desktop page; use local MCP when those
site tools are unavailable. The supervisor owns the requested browser flight
recorder and sends recordings to its assigned Viewer.
