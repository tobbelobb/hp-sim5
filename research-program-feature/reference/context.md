# Role context

Each locally launched role is injected through the coding client's instruction
mechanism: Codex receives `developer_instructions`; Claude receives
`--append-system-prompt-file`. This applies to managers, independent and
commissioned researchers, and reviewers. These augment the client's standard
instructions. Repository instructions still apply.

The launcher chooses the role explicitly. Removing research/AGENTS.md and the
role text in MCP initialization prevents unrelated agents from becoming
researchers just by reading research files or connecting physics tools.

| Entry | Additional context supplied by this feature |
| --- | --- |
| Local manager (`program/entry.py:client_plan`) | MANAGER.md, the manager connection path, and the two communication flag values |
| Existing-chat manager (`program/entry.py:main`) | MANAGER.md and communication flags in the launch response, plus the attachment and a compact overview |
| Researcher (`scripts/research_agent.py:main`) | RESEARCHER.md, assigned report/abstract/scratchbook paths and the fresh-world notice; Claude also receives root AGENTS.md explicitly |
| Commissioned researcher (`program/workers.py:start`) | The question, relevant saved human direction, session path, manager question/answer mailbox and extra instructions explicitly supplied by the manager; a cloud adapter adds its environment setup command |
| Reviewer (`program/review.py:plan`) | REVIEWER.md injected as developer/appended system instructions; the task message carries the submission directory and identity, known gaps and JSON response schema |
| Cloud roles | The cloud bridge passes the same saved instructions and task once to a fresh actor, plus its final-response path and the researcher's tool relay |

Existing-chat and cloud handoffs supply role instructions in chat messages. The
current cloud adapter cannot set a custom system/developer message or rewrite
the system prompt of an existing conversation.

The physics MCP server supplies only tool usage, units and the session path; it
does not inject a role or reporting workflow into connected agents.

The manager's first overview includes the question, assignment states, researcher
report paths and pending questions, review states, integration states and the
program summary path. It does not include reports, transcripts or the store's
full record history. `capabilities` describes tool
arguments; individual status and query calls expose details when requested.

Role files point to their own learned-wisdom directory. Hints are read as files;
they are not expanded into another generated prompt. The directories start empty.
Add only short lessons actually demonstrated in a session.

Tool descriptions, runtime manifests and reviewer response validation remain
operational data. Historical records can be retained in saved databases, but no
longer determine dispatch or appear in the overview.
Existing model conversations retain their earlier context; these changes apply
to fresh launches and subsequent tool responses.
