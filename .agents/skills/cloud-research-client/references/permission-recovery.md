# Cloud permission recovery

If `launcher_sockets` reports EPERM, the launcher cannot start its supervisor
in that command context. Dependency installation, environment reload and another
child process cannot grant socket permission. Use the platform's supported
permission mechanism or a command context that permits local sockets.

Supervised collection helps only when an attached supervisor is already running
in a permitted context. It cannot rescue a supervisor whose own socket creation
was denied. Worker actors can use the assigned file relay without command sockets.

The controller keeps its process's original permissions and imported Python code.
A permission grant or source change in a later tool-call command does not update
an existing controller. Finish or cancel owned work, call `tool CONNECTION
shutdown`, wait for cleanup, and reopen the same program with `--cloud --program
PROGRAM_ID` in the permitted context. Run `runtime_check` there. Keep the same
store, history and bound transport.

Worker worktrees and Git commits need write permission for the checkout's
Git metadata. Use the platform's supported permission mechanism when access is
denied; retain the actual diagnostic instead of treating a failed commit as success.
