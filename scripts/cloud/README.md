# Cloud development environment

For research workers and reviewers using only the current cloud chat's login,
see the opt-in [cloud research client](RESEARCH.md). Install it with
`bash scripts/cloud/install_research.sh`, then source
`scripts/cloud/research-env.sh`. It needs an active authenticated parent chat
with subagent tools; it does not start an independently authenticated coding CLI.

This directory preserves the setup and startup steps validated during Codex
cloud onboarding. Use the existing checkout: cloud tasks are already isolated,
so a separate Git worktree is unnecessary unless explicitly requested.

## Install

The installer targets **Linux x86_64**. The image must already provide Python
3.12 with venv/pip support, a C/C++ compiler, Git, curl, tar (with xz support),
sha256sum and Chromium. It does not install OS packages or request credentials.
Set `PUPPETEER_EXECUTABLE_PATH` if Chromium is not `/usr/bin/chromium`.

From the repository root:

```bash
bash scripts/cloud/install.sh
source scripts/cloud/env.sh
```

The installer prepares:

- Node **23.11.1**, Rust **1.99.1** and `wasm32-unknown-unknown`, outside the checkout.
- Python dependencies in `.venv`, including the optional **CPU Warp and JAX**
  requirements, plus CMake **4.4.4** and PyYAML **6.0.3** (for skill helper scripts).
  Python requirements are ranges, not a lockfile.
- JS dependencies with `npm ci`, preserving `package-lock.json`.
- The RRF and autocal submodules needed by CI, fetched through HTTPS using the
  environment's existing Git authentication.
- A native physics addon without PGO, a Wasm build in Cargo's ignored target
  directory, and the RRF simulator with four build jobs.
- Microsoft's `playwright-cli` skill and CLI **0.1.22**. The skill's upstream
  commit and all file checksums are recorded in [playwright-skill.json](playwright-skill.json).

Setup leaves tracked source files and lockfiles unchanged, including the tracked
browser Wasm. The other project skills are maintained in the repository and are
not downloaded or replaced by setup. A modified existing Playwright skill is
preserved; its installer stops with a message rather than overwriting it.

Defaults put tools under `/workspace/.tools` and caches under `/workspace/.cache`.
Set `HP_SIM5_CLOUD_TOOLS` and `HP_SIM5_CLOUD_CACHE` before installation and in each
later shell to use other writable locations. `HP_SIM5_BUILD_JOBS` controls RRF
build parallelism. Proxy and CA settings are inherited; TLS and checksum
verification stay enabled. Required downloads use `nodejs.org`, `sh.rustup.rs`,
`static.rust-lang.org`, GitHub (including `raw.githubusercontent.com`), GitLab,
PyPI and npm package hosts.

Node 23 matches the CI major version used during onboarding. The supplied and
official Node 24.19.0 binaries both aborted in `napi_detach_arraybuffer` with the
native addon on October 16. Keep the explicit Node pin until compatibility is
rechecked; the main README's Node 24 quick start has a different assumption.
GPU/CUDA support, firmware flashing and optional Klipper setup are outside this
installer's scope. The pinned Klipper commit could not be fetched during the
original onboarding; retry separately if that workflow is needed.

## Start services in a new task

Files and dependencies can survive an environment snapshot; processes do not.
Source `scripts/cloud/env.sh` in every new shell. Start each required service in
its own terminal or a task-managed background process:

```bash
bash scripts/cloud/start.sh vite
# Another terminal, only when the firmware service is needed:
bash scripts/cloud/start.sh rrf
```

Vite listens on loopback port 5173 and RRF on 8080. Override with
`HP_SIM5_VITE_PORT` or `HP_SIM5_RRF_PORT` before starting a service. Verify the
actual responses, adjusting ports if overridden:

```bash
curl -fsS http://127.0.0.1:5173/hp-sim5/hp-sim-3d/  # HTML title: HP Simulation
curl -fsS -X POST http://127.0.0.1:8080/machine/code \
  -H 'Content-Type: text/plain' --data M115            # contains FIRMWARE_NAME
```

Use these URLs internally for validation. The onboarding UI does not support
localhost preview links. Stop only service processes you started.

## Headless browser automation

Read `.agents/skills/playwright-cli/SKILL.md` when using the CLI. After sourcing
the environment and starting Vite:

```bash
playwright-cli -s=hp-sim open http://127.0.0.1:5173/hp-sim5/hp-sim-3d/ \
  --config="$PLAYWRIGHT_MCP_CONFIG"
playwright-cli -s=hp-sim snapshot
playwright-cli -s=hp-sim eval 'document.querySelectorAll("canvas").length'
playwright-cli -s=hp-sim screenshot --filename=output/playwright-cli/hp-sim-3d.png
playwright-cli -s=hp-sim close
```

The generated config uses headless system Chromium, an in-memory profile and
software rendering. Snapshots and screenshots go under ignored `output/`.
No browser UI or external research-agent login is needed. Software rendering
does not establish GPU performance.

## Validate development readiness

With the environment sourced:

```bash
cargo test --manifest-path src/wasm/cable_solver/Cargo.toml --locked
npx jest --runInBand
.venv/bin/python -m pytest autocal/tests tests/python
bash tests/run_all_rrf_http_endpoint_tests.sh
```

RRF HTTP tests manage their own service instances; stop the manually started RRF
first or set `RRF_HTTP_BASE_PORT` to avoid a port collision. Slow Python tests are
excluded by default; CUDA checks require a GPU. Visible-browser manual checks
remain separate.

`npm test`, `npm run test:physics` and `scripts/run_ci_tests.sh` rebuild the
**tracked** `public/wasm/cable_solver.wasm`. Run the required physics checks when
changing physics as directed by `AGENTS.md`; inspect generated changes before
committing. Setup itself uses Cargo directly so it does not replace that file.
The native source-contract test requires byte equality with the fresh Wasm build.
Check hashes first: formatting a large mismatched Buffer assertion consumed
excessive memory during onboarding. A matching source tree does not imply byte
identity across toolchains.

## Original validation findings

These are historical observations from October 16–19, 2026, not a claim that
the current revision still has the same results:

- With Node 23: 162 Jest suites passed (789 tests, one skipped), 109 Wasm/UI/
  profiling checks passed, six native functional checks passed and three Rust
  integration tests passed. All six RRF HTTP checks passed.
- Warp CPU coverage passed 105 checks; five CUDA checks were skipped. Headless
  Chromium loaded the simulator with one canvas and zero console errors.
- The original full Python run had 1,064 passes and 23 failures on Node 24.
  Targeted Node 23 reruns resolved 21 native-process failures. The remaining
  assertions were `KeyError: default::SpoolD` in the loaded-torque lifecycle
  test and a non-Warp Flipper score of 63 where the test required greater than
  75. Report current failures rather than treating these notes as exemptions.
- Fresh and repeated restoration of the Playwright skill, plus snapshot,
  evaluation, screenshot and named-session close, were validated.

When extracting these scripts on October 20 at `2c0b86ff`, the installer, service
startup/readiness, Playwright workflow and six RRF HTTP checks passed. The Rust
suite had two passes and one failure: the existing cable-solve fixture contains
35 values and panics on an access at index 35. Jest had 800 passes, one skip and
one Flipper 3D failure (score 9, expected 7). The 36 headless-autocal checks had
one HTTP 400 during initialization; that case passed on a targeted rerun. No
physics code or test expectations were changed to obtain passing setup checks.

For a managed environment, use `bash scripts/cloud/install.sh` as the saved
installation script and this document as the startup reference. Save/review
configuration and publish through the product to retain the prepared filesystem.
Committing these files alone does not update or publish an environment.
