// Registers an in-process stand-in for the bare `ws` package so the real bridge
// module can be exercised without opening a socket. Import this first:
//   node --import ./ws-stub/ws-register.mjs <script>
import { registerHooks } from 'node:module';

registerHooks({
  resolve(specifier, context, nextResolve) {
    if (specifier === 'ws') {
      return { url: new URL('./fake-ws.mjs', import.meta.url).href, shortCircuit: true };
    }
    return nextResolve(specifier, context);
  },
});
