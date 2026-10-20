Use the headless production native/Wasm backends for research experiments and
stability checks. They share the browser's scene construction and physics
pipeline; `npm run test:physics` exercises the production HP4 logo without a UI.
Browser experiments should be rare and preferably performed by a human. Use
them when a browser-specific issue cannot be reproduced headlessly, or when the
user explicitly requests interactive inspection. The browser supports presets,
scene uploads and multiple machines for those cases.

For shared browser work call `start_browser_service(record=True)` when recording
is needed, open its exact URL, inspect `browser_status`, and pass that page's
`page_id` to `browser_action`. Browser JS, standalone JS parity fixtures, and
native Python are distinct backends. The browser API uses the existing scene,
worker, motor, timing and inspection controllers. Pause before bounded stepping;
finish active workers before direct motor commands. The optional WebMCP site
tools operate the same API in the open desktop page; use local MCP when those
site tools are unavailable. The supervisor owns the requested browser flight
recorder and sends recordings to its assigned Viewer.
