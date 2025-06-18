import { createServer } from 'vite';
import path from 'path';

const projectRoot = process.argv[2] ? path.resolve(process.argv[2]) : process.cwd();
const server = await createServer({
  root: projectRoot,
  configFile: false,
  logLevel: 'error',
  server: { port: 0, host: '127.0.0.1' },
  optimizeDeps: { entries: [] }
});
await server.listen();
const port = server.httpServer.address().port;
console.log('PORT:' + port);
process.stdin.resume();
process.on('SIGTERM', () => {
  server.close().then(() => process.exit(0));
});
