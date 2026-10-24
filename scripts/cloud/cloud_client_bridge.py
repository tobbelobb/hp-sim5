"""Bridge actual Codex cloud agents to batch research clients, without model credentials.

The authenticated parent chat must dispatch requests with its collaboration tools.
This is an explicit cloud client transport, not a simulated coding-client result.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
QUEUE = Path(os.environ.get('HP_SIM5_CLOUD_QUEUE', str(ROOT / 'output/cloud-research/client-queue')))

def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.chmod(0o600)
    temporary.replace(path)

def read(path):
    return json.loads(path.read_text())

def client(arguments):
    if arguments == ['--version']:
        print('Codex cloud-session transport, actual authenticated agents dispatched by parent chat')
        return 0
    if not arguments or arguments[0] != 'exec':
        print('Cloud transport supports batch workers/reviewers. Operate the manager in the existing authenticated cloud chat.', file=sys.stderr)
        return 2
    def option(*names):
        for index, argument in enumerate(arguments):
            if argument in names and index + 1 < len(arguments):
                return arguments[index + 1]
            for name in names:
                if argument.startswith(name + '='):
                    return argument.split('=', 1)[1]
        return None
    root = Path(option('--cd', '-C') or os.getcwd()).resolve()
    final = option('--output-last-message')
    if not final:
        print('Missing final-message destination', file=sys.stderr)
        return 2
    final = Path(final).resolve()
    if not final.is_relative_to((root / 'output').resolve()):
        print('Final message must be inside repository output', file=sys.stderr)
        return 2
    prompt = sys.stdin.read()
    identity = uuid.uuid4().hex
    directory = QUEUE / identity
    directory.mkdir(parents=True, mode=0o700)
    (directory / 'prompt.md').write_text(prompt)
    (directory / 'prompt.md').chmod(0o600)
    request = {'request_id': identity, 'transport': 'authenticated-cloud-agent',
               'state': 'awaiting_agent', 'created_at': time.time(), 'root': str(root),
               'session': str(final.parent), 'final_output': str(final),
               'prompt_path': str(directory / 'prompt.md'),
               'instructions_path': str(final.parent / 'instructions.md'),
               'connection_path': str(final.parent / 'connection.json'),
               'pid': os.getpid(), 'agent_id': None,
               'requested_sandbox': option('--sandbox', '-s'),
               'effective_permissions': 'Existing cloud agent permission profile; CLI sandbox flags are not enforced by this adapter',
               'boundary': 'Cooperative same-UID files; parent chat owns actual cloud model sessions'}
    write(directory / 'request.json', request)
    print(json.dumps({'type': 'cloud_agent.requested', 'request_id': identity}), flush=True)
    stopping = []
    def cancel(signum, _frame):
        stopping.append(signum)
    signal.signal(signal.SIGTERM, cancel)
    signal.signal(signal.SIGINT, cancel)
    deadline = time.monotonic() + 600
    cancel_deadline = None
    while True:
        response = directory / 'response.json'
        if response.exists():
            result = read(response)
            if result.get('request_id') != identity or not result.get('agent_id'):
                raise ValueError('Cloud response does not identify the actual actor')
            state = read(directory / 'request.json')
            if state.get('agent_id') != result['agent_id']:
                raise ValueError('Cloud response actor differs from the bound actor')
            if result.get('state') == 'cancelled':
                return 130
            if not final.is_file() or hashlib.sha256(final.read_bytes()).hexdigest() != result.get('final_sha256'):
                raise ValueError('Actual actor final message missing or changed')
            print(json.dumps({'type': 'cloud_agent.completed', 'request_id': identity,
                              'agent_id': result['agent_id'], 'returncode': result['returncode']}), flush=True)
            return int(result['returncode'])
        if stopping or time.monotonic() >= deadline:
            if cancel_deadline is None:
                request = read(directory / 'request.json')
                request.update(state='cancel_requested', cancel_requested_at=time.time())
                write(directory / 'request.json', request)
                cancel_deadline = time.monotonic() + 45
            if time.monotonic() >= cancel_deadline:
                request = read(directory / 'request.json')
                request.update(state='cancel_unconfirmed', cleanup='Cloud actor stop not confirmed; interrupt it through collaboration tools')
                write(directory / 'request.json', request)
                return 124
        time.sleep(.1)

def main(arguments):
    if arguments and arguments[0] == 'client':
        return client(arguments[1:])
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=['list', 'show', 'bind', 'complete', 'cancelled'])
    parser.add_argument('request_id', nargs='?')
    parser.add_argument('--agent-id')
    parser.add_argument('--returncode', type=int, default=0)
    args = parser.parse_args(arguments)
    if args.operation == 'list':
        print(json.dumps([read(p) for p in sorted(QUEUE.glob('*/request.json'))], indent=2))
        return 0
    if not args.request_id or len(args.request_id) != 32 or any(c not in '0123456789abcdef' for c in args.request_id):
        parser.error('request_id must be the returned 32-character hexadecimal ID')
    directory = QUEUE / args.request_id
    request = read(directory / 'request.json')
    if args.operation == 'show':
        print(json.dumps(request, indent=2))
        return 0
    if not args.agent_id:
        parser.error('--agent-id must identify the actual cloud agent')
    if args.operation == 'bind':
        if request['state'] != 'awaiting_agent' or request.get('agent_id'):
            raise ValueError('Cloud request already bound or ended')
        request.update(state='running', agent_id=args.agent_id, bound_at=time.time())
        write(directory / 'request.json', request)
    else:
        if request.get('agent_id') != args.agent_id:
            raise ValueError('Only the bound cloud actor can complete this request')
        response = {'request_id': args.request_id, 'agent_id': args.agent_id,
                    'state': 'cancelled' if args.operation == 'cancelled' else 'completed',
                    'returncode': args.returncode, 'completed_at': time.time()}
        if args.operation == 'complete':
            final = Path(request['final_output'])
            response['final_sha256'] = hashlib.sha256(final.read_bytes()).hexdigest()
        write(directory / 'response.json', response)
        request.update(state=response['state'], completed_at=response['completed_at'])
        write(directory / 'request.json', request)
    print(json.dumps({'request_id': args.request_id, 'state': request['state']}))
    return 0

if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
