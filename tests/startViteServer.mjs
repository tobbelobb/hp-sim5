import { createServer } from 'vite';
import path from 'path';
import { createViteServerOptions } from './viteServerOptions.mjs';

const projectRoot = process.argv[2] ? path.resolve(process.argv[2]) : process.cwd();
const server = await createServer(createViteServerOptions(projectRoot));
await server.listen();
const port = server.httpServer.address().port;
console.log('PORT:' + port);
process.stdin.resume();
process.on('SIGTERM', () => {
  server.close().then(() => process.exit(0));
});
