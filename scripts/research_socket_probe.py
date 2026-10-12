"""Run in the agent's command context to measure its actual localhost policy."""
import json
import socket
import threading
import time
from urllib.request import ProxyHandler, build_opener


def probe():
    started = time.monotonic()
    checks = {}
    listener = None
    worker = None
    try:
        listener = socket.socket()
        checks['socket_creation'] = {'outcome': 'passed'}
        listener.settimeout(2)
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        checks['loopback_binding'] = {'outcome': 'passed'}

        def serve():
            try:
                connection, _ = listener.accept()
                with connection:
                    connection.settimeout(2)
                    connection.recv(4096)
                    connection.sendall(b'HTTP/1.1 200 OK\r\nContent-Length: 10\r\nConnection: close\r\n\r\ntool-check')
            except OSError:
                pass

        worker = threading.Thread(target=serve, daemon=True)
        worker.start()
        url = f'http://127.0.0.1:{listener.getsockname()[1]}'
        with build_opener(ProxyHandler({})).open(url, timeout=2) as response:
            if response.read() != b'tool-check':
                raise OSError('Unexpected loopback HTTP reply')
        checks['http_round_trip'] = {'outcome': 'passed'}
    except OSError as error:
        name = next(name for name in ('socket_creation', 'loopback_binding', 'http_round_trip') if name not in checks)
        checks[name] = {'outcome': 'blocked', 'error': str(error),
                        'next_step': 'Use supervised research_preflight/start_test; check the effective command network policy'}
    finally:
        if listener is not None:
            listener.close()
        if worker is not None:
            worker.join(timeout=2)
    for name in ('socket_creation', 'loopback_binding', 'http_round_trip'):
        checks.setdefault(name, {'outcome': 'unknown', 'reason': 'An earlier check was blocked'})
    return {'kind': 'tool_check', 'context': 'agent_command', 'checks': checks,
            'outcome': 'passed' if all(check['outcome'] == 'passed' for check in checks.values()) else 'blocked',
            'wall_s': time.monotonic() - started}


if __name__ == '__main__':
    result = probe()
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['outcome'] == 'passed' else 1)
