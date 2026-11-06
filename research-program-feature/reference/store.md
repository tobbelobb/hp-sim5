# Program store

`program.store.ProgramStore` stores program questions, approaches, assignments,
evidence and decisions in `.research-programs/programs.sqlite3`. IDs and revisions
identify records; stale updates are rejected. Stored process IDs are historical
observations and cannot authorize signals by themselves.

`create`, `status`, `overview`, `export` and `import` are available through the
root CLI; the bound manager tools provide the ordinary working path. Export/import preserves record history; selected submission files must
also be retained for future verification. Import is a trusted restore operation.
The graph stores claims, dependencies, assessments and withdrawn evidence.

Existing program data remains readable after the source relocation. Research
reports and data remain in their original session directories.
