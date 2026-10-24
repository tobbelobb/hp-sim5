import anyio, importlib.util, sys
from pathlib import Path
spec = importlib.util.spec_from_file_location('program_mcp', sys.argv[1])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
server = module.build_server(Path(sys.argv[2]))
async def timer():
    while True:
        await anyio.sleep(.01)
async def main():
    async with anyio.create_task_group() as group:
        group.start_soon(timer)
        await server.run_stdio_async()
        group.cancel_scope.cancel()
anyio.run(main)
