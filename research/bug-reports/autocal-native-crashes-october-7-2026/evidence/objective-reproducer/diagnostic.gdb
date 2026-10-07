set pagination off
set confirm off
set debuginfod enabled off
set print thread-events off
handle SIGPIPE nostop noprint pass
run
if $_inferior_thread_count > 0
  p $_siginfo
  info registers
  x/16i $pc-24
  bt 60
  thread apply all bt 8
end
