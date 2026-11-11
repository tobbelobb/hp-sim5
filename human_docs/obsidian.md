# Using Obsidian with research output

Research output is already plain Markdown with relative links, so Obsidian needs
no export step and agents need no special instructions. Obsidian is a reader
and note-taking layer over the same files; the agents never depend on it.

## Setup (once)

1. Install Obsidian, then *Open folder as vault* → `research/` in this repo.
   The shared `research/.obsidian/app.json` makes new links standard relative
   Markdown links (agents and GitHub can follow them), keeps links updated when
   you rename files, and hides `scratchbook/` and `recovery-originals/`.
   Other `.obsidian/` files are your own and ignored by Git.
2. Trust the vault. No community plugins are needed.
3. Regenerate the generated pages after a run (the launcher already does at
   startup and exit): `.venv/bin/python scripts/research_index.py`.
   It writes `output/index.md` (one sentence per session) and `output/topics.md`
   (one hub per topic shared by two or more session abstracts).

Reports link to code with paths like `../../../autocal/...`; those leave the
vault and show as unresolved. Open them in your editor instead. Program
`summary.html` pages are not rendered by Obsidian; open them in a browser.

## What you get

| Obsidian feature | Use it for |
| --- | --- |
| Quick switcher (`Ctrl+O`) and search (`Ctrl+Shift+F`) | Find a session by title or any word in the reports. Search operators such as `path:output/ "slip"` or `file:report` narrow it. |
| `output/index.md` | Skim every session's result sentence in one page. |
| `output/topics.md` | Jump from a topic (e.g. `hp3`, `candidate ranking`) to all sessions about it. |
| Backlinks pane | On a report, see which other sessions, notes or catalog pages cite it. |
| Local graph | Open it on a session to see its report, evidence and the sessions it builds on. Sessions with no neighbours are research nobody built on. |
| Hover preview (`Ctrl`+hover) | Read a linked report without leaving the one you are in. |
| Outline pane | Navigate 4000-line reports by heading. |
| Properties/Bases (core) | Optional: add `status:` or `decision:` frontmatter to your own notes and tabulate them. |

## Your own notes: the human-in-the-loop part

Reading is the cheaper half. The valuable half is writing back.

- Put your decisions, doubts and follow-up ideas in `research/notes/` (create
  it; it is yours, commit it or not). Link the report you are reacting to:
  `[slip recovery](../output/session-2026-10-09-slip-recovery-and-encoder-model/report.md)`.
  Obsidian's backlinks then show your reaction on the report.
- To steer an agent, point it at the note in plain words, for example
  `./hp-sim5-research-agent --claude --prompt 'Follow research/notes/next-hp3-question.md'` or tell the
  manager "see research/notes/next-hp3-question.md for my goals and limits".
  Agents read repo files, so no prompt changes are needed. This is the cheap
  way to give a manager (`--grill-me`) your goals once and reuse them.
- A good note per question: goal, what counts as success (with units), what
  authority the agent has (may it touch main?), and links to the 2-3 earlier
  reports it must not repeat.
- After a session, write three lines in a note: what you believed, what the
  report showed, what you will ask next. Over weeks this is the research
  narrative that the agents' one-sentence abstracts cannot give you.
- Triage with the graph: topics with many sessions and no conclusion are where
  to commission a reviewer; topics with one session are unexplored.

## What this does not do

It does not change what agents read or write, verify any finding, or make
`index.md`/`topics.md` authoritative: they are generated snapshots of the
sessions present on this machine. Topic names are free text in abstracts, so
near-duplicates (`autocal`, `autocalibration`) stay separate hubs.
