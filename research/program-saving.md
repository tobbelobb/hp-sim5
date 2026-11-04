# Saving Research Program results

```bash
./hp-sim5-research-program save PROGRAM_ID --commit
# Include additional small analysis scripts, inputs or configuration:
./hp-sim5-research-program save PROGRAM_ID --file path/to/analysis.py --commit
```

This writes `research/programs/PROGRAM_ID/`: program memory, a readable overview,
worker notes, referenced small evidence and fixed review submissions. It commits
only that directory, preserving unrelated staged work. It never pushes. Code
changes belong in their own explicitly selected commits. The controller performs
the Git operation in its own execution context.

Runtime stores matching `.research-programs*/` and generated `research/programs/`
archives are ignored by default. An explicit save --commit stages just that
program's archive with git add --force. This keeps abandoned trial archives out
of broad commits while allowing deliberate research-result commits. Saving without
--commit leaves the archive on disk and ignored.

The default copying limits are 1 MiB per file and 8 MiB total evidence. Missing,
large and private/runtime files are listed in files.json instead of copied.
Inspect those omissions: deliberately retain any larger evidence needed to
support the findings separately. Explicit --file selections fail if unsafe,
missing or too large. Credentials, live connection files and dependency trees
are not research results. These Git records preserve findings and provenance;
they are not a portable running environment.

