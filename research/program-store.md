# Durable program storage and commands

Task 007 supplies a local program store and offline CLI. It records questions,
competing approaches, bounded assignments, evidence references and dated
decisions/steering. It starts no physics world, coding client or worker.
Task 008 adds the [typed evidence graph](program-graph/README.md), query commands
and withdrawal/applicability review events on this same store.
The full manager still requires [tasks 009–016](program-tasks/README.md),
including dispatch, reviewed admission and chat entry.

From the repository root, a question alone is sufficient:

```bash
./hp-sim5-research-program create 'Can the same HP3 data improve calibration?'
```

Commands print JSON with stable IDs, UTC timestamps and revisions. Use the
returned program `id` and root `question_id` in subsequent commands. The
following example uses `PROGRAM_ID` and `QUESTION_ID` as placeholders:

```bash
./hp-sim5-research-program approach PROGRAM_ID 'Screen saved candidates' \
  --id saved-screen --question QUESTION_ID --reason 'Cheap comparison using existing evidence'
./hp-sim5-research-program approach PROGRAM_ID 'Change the fitting objective' \
  --id fit-change --question QUESTION_ID --reason 'An alternative if selection alone is insufficient'
./hp-sim5-research-program assignment PROGRAM_ID 'Inspect one frozen history' \
  --id saved-check --approach saved-screen --allocation '{"wall_minutes":5,"fresh_fits":0}'
./hp-sim5-research-program overview PROGRAM_ID
./hp-sim5-research-program steering PROGRAM_ID 'Keep the measurement data fixed' --actor human
./hp-sim5-research-program transition PROGRAM_ID fit-change parked \
  --reason 'First resolve the cheaper saved-evidence question'
```

An optional `create --constraints '{"same_measurements":true,"fresh_fits":2}'`
preserves the supplied JSON object. Omitted caps stay unspecified; they do not
become zeros. Every assignment requires a positive finite `wall_minutes`
allocation, with other explicitly chosen nonnegative numeric quantities as
needed. These are recorded allocations. The store does not enforce physical
workload, shell spend or elapsed worker time; controller enforcement belongs to
task 013. No command here launches a worker.

| Command | Arguments and behavior |
| --- | --- |
| `create` | `QUESTION [--constraints JSON] [--id ID]`; atomically creates a program and its root question |
| `open`, `status`, `overview` | `PROGRAM`; equivalent complete views with questions, live alternatives, assignments and notes |
| `question` | `PROGRAM TEXT [--scope JSON] [--id ID]`; records another scoped question |
| `approach` | `PROGRAM TEXT --question ID --reason TEXT [--id ID]`; records a live alternative |
| `assignment` | `PROGRAM TEXT --approach ID --allocation JSON [--id ID]`; records a planned bounded objective |
| `artifact` | `PROGRAM REFERENCE --sha256 HASH --evidence-type TYPE --source-revision REV [--provenance JSON] [--id ID]`; records external evidence |
| `decision`, `steering` | `PROGRAM TEXT [--subject ID] [--evidence ARTIFACT_ID] [--actor NAME] [--id ID]`; appends a dated note; `--evidence` can repeat |
| `transition` | `PROGRAM RECORD_ID STATE --reason TEXT`; changes program, approach or assignment state and appends a decision in one transaction |
| `export` | `PROGRAM [--format json\|markdown]`; writes a complete snapshot to stdout |
| `import` | `JSON_FILE`; restores a version-1 snapshot, rejecting existing IDs and unsupported versions atomically |

Mutation commands other than create/import accept `--expected-revision N`.
This checks the current **program** revision, preventing stale decisions.
Every successful mutation advances it once. Individual records have their own
revision, incremented when their state changes. IDs are unique across programs
and records in one store; caller-supplied duplicate IDs fail without partial state.

Program states are `active → paused → active`, with `stopped` terminal from
active/paused. Approaches start `live`; live/promising can become parked/retired,
parked can reopen live/promising or retire, and retired can reopen live. A reason
is mandatory for every transition. Assignment execution states are
`planned → ready → running → completed|failed|cancelled`; planned/ready can also
be cancelled. Creating assignments or marking them ready/running requires an
active program and a live/promising approach. These transitions record execution
state only. They do not signal processes. Review stays `unreviewed`, knowledge
stays `unassessed`, and the overview honestly reports no scientific result.
Later tasks own evidence admission and actual worker lifecycle.

Artifact types are `simulated`, `synthetic`, `analytic`, `hardware` and `imported`.
The store retains the supplied external path/URL, SHA-256, source revision and
provenance. It neither copies nor verifies the scientific bytes. Keep those
files in the existing [session/handoff layout](handoffs.md); use a portable
handoff when the original session may disappear. Imports preserve recorded
execution and provenance but cannot turn unassessed records into accepted facts.

```bash
./hp-sim5-research-program export PROGRAM_ID > /tmp/program.json
./hp-sim5-research-program export PROGRAM_ID --format markdown > /tmp/program.md
./hp-sim5-research-program --store /tmp/restored-manager import /tmp/program.json
./hp-sim5-research-program --store /tmp/restored-manager open PROGRAM_ID
```

The JSON format is lossless and importable; Markdown is a full readable view.
Neither export includes large artifact contents. Import rejects unsupported
versions, invalid references/state, duplicate IDs and malformed records before
committing anything. There is no implicit migration or overwrite operation.

## Location and ownership

The authoritative default is `<repository>/.research-programs/programs.sqlite3`,
outside worker-shared `output/` and Git-ignored. The manager owns this directory;
workers receive bounded assignments and artifact locations, not authority over
its contents. The launcher does not link this store into worker worktrees.
`--store DIRECTORY` before the subcommand selects an explicit manager-owned
location; no environment variable redirects it silently.

Initialization uses a private directory/file (0700/0600), SQLite transactions,
schema version 1, foreign keys and full synchronous writes. Concurrent writers
serialize with a 30-second busy timeout. Reopening does not reinitialize an
existing database. A killed uncommitted writer rolls back on reopen. Unsupported
schema versions fail with an error. Inspection of a missing store fails rather
than creating a misleading empty program.

The store refuses symlinks in its path, linked database/journal files, unexpected
file ownership, hard-linked database files and store directories/files writable
by group/others. This protects against accidentally following a worker-writable
session alias. Same-UID processes can still modify each other's files; this is
not adversarial isolation. Decisions and steering also have database triggers
rejecting edits/deletes; append a correction as a new note.

## Python API and verification

`hp_sim5_research.program.ProgramStore(directory, create=False)` is a context
manager using only the standard library. The CLI directly uses `create_program`,
`get_program`, `get_record`, `add_question`, `add_approach`, `add_assignment`,
`add_artifact`, `record_note`, `transition`, `overview`, `snapshot` and
`import_snapshot`. `render_markdown(snapshot)` supplies the readable view.
`transaction()` exposes the store connection for the dependent graph/controller
tasks; keep their related SQL writes within one transaction. Public mutations
already open transactions and should not be nested. Invalid requests raise
`ProgramError`; SQL uniqueness failures raise `sqlite3.IntegrityError`. The CLI
reports expected request/storage errors on stderr with exit code 2.

`store.graph()` returns an `EvidenceGraph` using the same program revisions,
transactions and record table. Its additive record kinds fit the version-1
envelope without changing tables or existing snapshots. Older software which
does not know these kinds cannot import graph snapshots. Claims, artifacts,
derivations, relations, reported assessments and review events are append-only;
dated new versions/events preserve prior scope and evidence. See the graph guide
for APIs, query semantics and the inspected real-data seed.

Run the offline contract checks without optional test packages:

```bash
.venv/bin/python tests/python/test_research_program.py -v
```

The checks exercise fresh CLI processes, complete exports/imports, allocations,
state/revision guards, append-only history, concurrent initialization/writers,
interrupted-write recovery, and unsafe-path/version rejection. They are storage
and command checks, with no model, numerical or physics runs. They do not prove
the complete manager release or any scientific claim.
