"""RPC to a single existing browser page, with explicit identity and saved steering context."""
import asyncio
import json
from pathlib import Path
import uuid

from websockets.asyncio.server import serve
from .services import free_port


class BrowserConnection:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.socket = None
        self.page = None
        self.pending = {}
        self.contexts = []
        self.server = None
        self.path = '/' + uuid.uuid4().hex

    async def start(self):
        port = free_port()
        self.server = await serve(self.receive, '127.0.0.1', port, max_size=16 * 1024 * 1024)
        self.url = f'ws://127.0.0.1:{port}{self.path}'
        return self

    async def receive(self, socket):
        if socket.request.path != self.path or self.socket is not None:
            await socket.close(code=1008, reason='Select exactly one research page')
            return
        self.socket = socket
        try:
            async for message in socket:
                data = json.loads(message)
                if data.get('type') == 'ready':
                    self.page = data
                elif data.get('type') == 'context':
                    self.save_context(data['result'])
                elif data.get('id') in self.pending:
                    future = self.pending[data['id']]
                    if future.done():
                        continue
                    if data.get('error'):
                        future.set_exception(RuntimeError(data['error']))
                    else:
                        future.set_result(data['result'])
        finally:
            self.socket = self.page = None
            for future in self.pending.values():
                if not future.done():
                    future.set_exception(RuntimeError('Browser page disconnected; state was not restored'))

    def save_context(self, context):
        self.contexts.append(context)
        with (self.directory / 'browser-context.jsonl').open('a') as output:
            output.write(json.dumps(context, allow_nan=False) + '\n')

    async def call(self, action, args, page_id):
        if not self.socket or not self.page:
            raise RuntimeError('Open the supervised browser URL first')
        if page_id != self.page['page_id']:
            raise ValueError('Page identity changed; inspect browser_status and select the page explicitly')
        request_id = uuid.uuid4().hex
        future = asyncio.get_running_loop().create_future()
        self.pending[request_id] = future
        try:
            await self.socket.send(json.dumps({'id': request_id, 'action': action, 'args': args}))
            result = await asyncio.wait_for(future, timeout=60)
            if action == 'capture_context':
                self.save_context(result)
            if action not in ('observe', 'capabilities', 'interventions', 'capture_context'):
                with (self.directory / 'browser-actions.jsonl').open('a') as output:
                    output.write(json.dumps({'page_id': page_id, 'action': action, 'args': args,
                                             'result': result}, allow_nan=False) + '\n')
            return result
        finally:
            self.pending.pop(request_id, None)

    def status(self):
        return {'backend': 'browser-js', 'connected': self.page is not None,
                'page': {'page_id': self.page['page_id'], 'backend': 'browser-js'} if self.page else None,
                'captured_contexts': self.contexts,
                'context_artifact': str(self.directory / 'browser-context.jsonl')}

    async def close(self):
        if self.server:
            self.server.close()
            await self.server.wait_closed()
