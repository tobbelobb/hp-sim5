# Strategy and prompt tools

`program.strategy.ProgramStrategy` records approaches, route decisions and
assignments. `route_plan` currently requires a hypothesis, decision criteria and
cost information; `commission` creates the corresponding investigation. These
requirements remain implementation debt after the directory cleanup.

`prompt_portfolio` lists reusable research questions in `research/prompts/`.
Selecting a prompt can return its text and checksum; route records retain that
identity. `portfolio.json` registers those missions. It contains no role prompts.

Graph queries return support, dependencies and earlier decisions. Reviewed
findings can update an approach without adopting a code change. Human steering
is retained and can change the next allocation.
