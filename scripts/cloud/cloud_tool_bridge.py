"""Launcher-owned MCP calls for cloud actors whose command sockets are denied.

Only the existing hp-sim5 and assigned Rerun attachments are available. This is
a cooperative same-UID file transport, not a shell runner or a new physics host.
"""
import argparse
import asyncio
from contextlib import AsyncExitStack
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import uuid


def write(path, value):
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        with open(temporary, 'x', opener=lambda p, flags: os.open(p, flags, 0o600)) as stream:
            json.dump(value, stream)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def read(path):
    with Path(path).open('rb') as stream:
        data = stream.read(1024 * 1024 + 1)
    if len(data) > 1024 * 1024:
        raise ValueError('Tool request or descriptor exceeds 1 MiB')
    return json.loads(data)


def active(connection):
    owner = read(connection['owner_request'])
    return owner['request_id'] == connection['request_id'] and owner['state'] in ('awaiting_agent', 'running')


def call(connection_path, server, operation, *, tool=None, arguments=None, timeout=120):
    """Never retry a mutation after a timeout; its outcome may be unknown."""
    connection = read(connection_path)
    if connection.get('version') != 1 or server not in ('hp_sim5', 'rerun'):
        raise ValueError('Use the returned version-1 connection and hp_sim5 or rerun')
    if operation not in ('list_tools', 'call_tool') or (operation == 'call_tool' and not isinstance(tool, str)):
        raise ValueError('Select list_tools or call_tool with a tool name')
    if arguments is not None and not isinstance(arguments, dict):
        raise ValueError('Tool arguments must be an object')
    if isinstance(timeout, bool) or not math.isfinite(timeout) or not 0 < timeout <= 14500:
        raise ValueError('Tool timeout must be finite and at most 14500 seconds')
    directory = Path(connection['directory'])
    if not active(connection) or (directory / 'closed.json').exists():
        raise RuntimeError('Cloud tool attachment has ended; do not replay historical requests')
    identity = uuid.uuid4().hex
    queued = directory / (identity + '.request')
    response = directory / (identity + '.response')
    write(queued, {'request_id': connection['request_id'], 'server': server, 'operation': operation,
                   'tool': tool, 'arguments': arguments or {}, 'expires': time.time() + timeout})
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            if response.exists():
                result = json.loads(response.read_text())
                if not result['ok']:
                    raise RuntimeError(result['error'])
                return result['result']
            if not active(connection) or (directory / 'closed.json').exists():
                raise RuntimeError('Cloud tool attachment ended; inspect saved state before retrying a mutation')
            time.sleep(.02)
        raise TimeoutError('Cloud tool response timed out; outcome may be unknown. Inspect saved state before retrying a mutation')
    finally:
        queued.unlink(missing_ok=True)
        response.unlink(missing_ok=True)


async def serve(connection_path):
    from mcp import Client, StdioServerParameters

    connection = read(connection_path)
    directory = Path(connection['directory'])
    root = Path(connection['root'])
    sys.path.insert(0, str(root / 'src/python'))
    from hp_sim5_research.program_workers import process_live
    descriptor = Path(connection['session']) / 'connection.json'
    loop = asyncio.get_running_loop()
    owner_task = asyncio.current_task()
    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, owner_task.cancel)
    clients = {}

    async def watch_owner():
        while True:
            if not active(connection) or not process_live(connection['owner_process']):
                owner_task.cancel()
                return
            await asyncio.sleep(.1)

    monitor = asyncio.create_task(watch_owner())

    async with AsyncExitStack() as stack:
        async def client(server):
            if server not in ('hp_sim5', 'rerun'):
                raise ValueError('Only hp_sim5 and the assigned rerun attachment are available')
            if server not in clients:
                args = [str(root / 'scripts/research_attach.py'), str(descriptor)]
                if server == 'rerun':
                    args.append('--rerun')
                parameters = StdioServerParameters(command=str(root / '.venv/bin/python'), args=args,
                                                   cwd=str(root), env=dict(os.environ))
                clients[server] = await stack.enter_async_context(Client(parameters, read_timeout_seconds=14500))
            return clients[server]

        async def execute(path):
            response = path.with_suffix('.response')
            running = path.with_suffix('.running')
            started = time.monotonic()
            envelope = {}
            try:
                path.rename(running)
                envelope = read(running)
                if envelope.get('request_id') != connection['request_id'] or envelope.get('expires', 0) < time.time():
                    raise ValueError('Stale or expired cloud tool request; no tool was called')
                if not active(connection) or envelope['expires'] < time.time():
                    raise ValueError('Cloud actor ended or tool request expired before execution')
                peer = await client(envelope['server'])
                if envelope['operation'] == 'list_tools':
                    result = [tool.model_dump(mode='json', by_alias=True) for tool in (await peer.list_tools()).tools]
                elif envelope['operation'] == 'call_tool':
                    result = (await peer.call_tool(envelope['tool'], envelope['arguments'])).model_dump(mode='json', by_alias=True)
                else:
                    raise ValueError('Unsupported cloud tool operation')
                write(response, {'ok': True, 'result': result})
            except asyncio.CancelledError:
                write(response, {'ok': False, 'error': 'Launcher tool relay stopped; mutation outcome may be unknown'})
                raise
            except Exception as error:
                write(response, {'ok': False, 'error': str(error)})
            finally:
                running.unlink(missing_ok=True)
                with (directory / 'calls.jsonl').open('a') as stream:
                    stream.write(json.dumps({'id': path.stem, 'server': envelope.get('server'),
                                             'operation': envelope.get('operation'), 'tool': envelope.get('tool'),
                                             'wall_s': time.monotonic() - started}) + '\n')

        try:
            # Keep MCP context entry, calls and exit in this same asyncio task.
            # Long production jobs return IDs and remain owned by the supervisor.
            while active(connection):
                for path in sorted(directory.glob('*.request')):
                    await execute(path)
                await asyncio.sleep(.02)
        finally:
            monitor.cancel()
            await asyncio.gather(monitor, return_exceptions=True)
            write(directory / 'closed.json', {'request_id': connection['request_id'], 'closed_at': time.time()})


def start(root, session, owner_request):
    """Called only by the permitted worker launcher, never by a review actor."""
    import subprocess
    root, session, owner_request = Path(root), Path(session), Path(owner_request)
    if not (session / 'connection.json').is_file():
        return None, None
    owner = read(owner_request)
    directory = owner_request.parent / 'tool-queue'
    directory.mkdir(mode=0o700)
    connection_path = owner_request.parent / 'tool-connection.json'
    sys.path.insert(0, str(root / 'src/python'))
    from hp_sim5_research.program_workers import process_identity
    from hp_sim5_research.services import register_test_process
    connection = {'version': 1, 'request_id': owner['request_id'], 'directory': str(directory),
                  'root': str(root), 'session': str(session), 'owner_request': str(owner_request),
                  'owner_process': process_identity(os.getpid())}
    write(connection_path, connection)
    with (session / 'cloud-tools.log').open('w') as log:
        process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), 'serve', str(connection_path)],
                                   cwd=root, stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
    register_test_process(process)
    connection['relay_process'] = process_identity(process.pid)
    write(connection_path, connection)
    return process, connection_path


def stop(process):
    if process is not None:
        root = Path(__file__).resolve().parents[2]
        sys.path.insert(0, str(root / 'src/python'))
        from hp_sim5_research.services import stop_process
        stop_process(process)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('serve', 'list', 'call'))
    parser.add_argument('connection', type=Path)
    parser.add_argument('tool', nargs='?')
    parser.add_argument('--server', choices=('hp_sim5', 'rerun'), default='hp_sim5')
    parser.add_argument('--params', default='{}')
    parser.add_argument('--timeout', type=float, default=120)
    args = parser.parse_args()
    if args.operation == 'serve':
        try:
            asyncio.run(serve(args.connection))
        except asyncio.CancelledError:
            raise SystemExit(130)
    else:
        result = call(args.connection, args.server, 'list_tools' if args.operation == 'list' else 'call_tool',
                      tool=args.tool, arguments=json.loads(args.params), timeout=args.timeout)
        print(json.dumps(result))


if __name__ == '__main__':
    main()
