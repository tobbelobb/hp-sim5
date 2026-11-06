# Research Program cleanup, 6 November 2026

The review concentrated on 4–6 November and followed older file history where
needed to explain the layout. The complaint's desired workflow is the basis for
the new roles. This cleanup changes organization and prompts; it does not replace
the controller's execution model.

| Change | What happened |
| --- | --- |
| `55dbf8d9`, 5 November | Renamed `research/prompts/research-manager-workflow.txt` to the packaged `program_instructions/manager.md`, moved the reviewer and other workflow prompts, and added a validated mission registry and mandatory prompt comparison. It inherited the requirement that managers specify hypotheses and several budgets. |
| `d567ed2b`, 5 November | Added delivery requirements and per-collection checkpoints while leaving experiment planning and bookkeeping in the manager prompt. |
| `67f59ec8`, 4 November | Added interview/question flags and another generated manager instruction block in `program_communication.py`. That block now supplies flag values only; their meaning lives in MANAGER.md. |
| `fbd2c744`, 5 November | Put deployment setup text and sequential-execution policy in worker, pool and runtime code. These policies now live in the client adapter inside the cloud skill. |
| `3abb2d87`, 6 November | Moved human launch instructions into the main README, leaving stale links and the save wrapper there. Launch instructions now live in `human_docs/research-programs.md`; README contains a short link. |

The Python directory did not rename the project. It began with the research agent
in `aaf25cf7` on 5 October. Python identifiers use underscores because hyphens are
not valid in normal import statements. Program code was added there starting with
`f6bbda35` on 23 October. That mixed two distinct features. Program code now lives
in `research-program-feature/program/`; the older standalone agent tools remain
under `src/python/hp_sim5_research/`.

New researcher sessions go to `research/sessions/`; manager notes go to
`research/programs/PROGRAM_ID/manager/`. Existing saved sessions are left in place. Worktrees share the session
directory, and the client receives write access to assigned evidence outside its
checkout. Live connection files remain ignored.
`scripts/research_index.py` now defaults to `research/`; `--root output/research`
can still inspect older standalone sessions.

Other machinery found during the audit:

| Machinery | Current home and disposition |
| --- | --- |
| Claim/dependency graph and reviewed admissions | `program/graph.py`, `program/review.py`; retained, with their technical reference beside the code |
| Route plans and investigation records | `program/strategy.py`; still required by managed dispatch and the next candidate for simplification |
| Work/review/startup/cleanup reservations | `program/controller.py`; retained enforcement, still more elaborate than the desired manager loop |
| Store-wide worker pool and exclusive leases | `program/pool.py`; retained defaults and ownership checks |
| Combined-candidate worktree and check runner | `program/integration.py`, `scripts/`; retained; final delivery uses ordinary Git |
| Mission registry | `research/prompts/portfolio.json`; retained for existing queries and prompt provenance; role instructions no longer mandate portfolio comparison |
| Assignment and single-agent prompt layer | Deleted; it had no executable caller beyond the role loader and pointed at the deleted agenda |
| Result copying and scoped Git commits | Deleted, including CLI/tool operations, implementation, tests and save guide |
| Feature audits | `archive/`; historical material, excluded from runtime instructions |
| Seeded graph demo and old audit generator | Deleted; no runtime or test caller, with references to deleted tasks and agenda |
| Cloud pilot and deployment instructions | The cloud skill's `archive/` and adapter; absent from ordinary launch documentation |

The only manager/reviewer role sources are `role-instructions/MANAGER.md` and
`role-instructions/REVIEWER.md`. Launches read them directly. Runtime prompts also
carry assignment identities, response schemas and settings. Old prompts already
saved in past sessions remain historical records.

The manager's short communication rule draws on the local `unslop` skill and
[Writing Clearly and Concisely](https://skills.sh/obra/the-elements-of-style/writing-clearly-and-concisely):
use concrete subjects, active verbs and familiar words; remove needless words.
No extra writing prompt or installed dependency is needed at launch.
