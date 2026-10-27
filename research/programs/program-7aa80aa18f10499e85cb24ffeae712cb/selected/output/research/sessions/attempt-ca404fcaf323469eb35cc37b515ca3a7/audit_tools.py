import asyncio
import json
import os
from pathlib import Path
import sys
import time

from mcp import Client, StdioServerParameters

ROOT = Path('/workspace/hp-sim5-worktrees/bounded-research-assignment-0889b918')
SESSION = Path('/workspace/hp-sim5/output/research/sessions/attempt-ca404fcaf323469eb35cc37b515ca3a7')
CONNECTION = SESSION / 'connection.json'

def save(name, value):
    (SESSION / name).write_text(json.dumps(value, indent=2, default=str) + '\n')

async def main():
    parameters = StdioServerParameters(command=str(ROOT / '.venv/bin/python'), args=[str(ROOT / 'scripts/research_attach.py'), str(CONNECTION)], cwd=str(ROOT), env=dict(os.environ))
    async with Client(parameters, read_timeout_seconds=90) as client:
        tools = (await client.list_tools()).tools
        save('tool-schemas.json', [tool.model_dump() for tool in tools])
        print(json.dumps({'tool_count': len(tools), 'tool_names': [t.name for t in tools]}), flush=True)
        async def call(name, args=None, filename=None):
            started = time.monotonic()
            result = await client.call_tool(name, args or {})
            value = {'tool': name, 'args': args or {}, 'wall_s': time.monotonic()-started, 'is_error':result.is_error, 'result':result.structured_content, 'content':[c.model_dump() for c in result.content] if result.is_error else []}
            save(filename or name + '.json', value)
            with (SESSION / 'tool-calls.jsonl').open('a') as stream:
                stream.write(json.dumps(value, default=str) + '\n')
            print(json.dumps({'tool': name, 'wall_s': value['wall_s'], 'is_error': result.is_error, 'result': result.structured_content if name not in ('capabilities','runtime_status') else 'saved'}), flush=True)
            return result.structured_content
        await call('capabilities')
        await call('runtime_status', filename='runtime-initial.json')
        await call('research_preflight')

asyncio.run(main())
