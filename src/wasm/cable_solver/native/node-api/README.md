# Node-API C headers

These four unmodified headers come from Node.js v23.11.1. They are the complete
C header dependency of `../node_bridge.c`, which explicitly targets Node-API 8.
They require only standard C library headers, not V8, libuv or the rest of the
Node development package.

The native build uses this directory by default. This keeps fresh checkouts
buildable offline with a runtime-only Node installation. An explicit
`NODE_INCLUDE_DIR` overrides this directory.

[Node-API's ABI stability](https://nodejs.org/docs/latest-v23.x/api/n-api.html#implications-of-abi-stability)
allows the same declarations to be used across Node versions supporting API 8.
The SDK release does not pin the runtime version.

[UPSTREAM.json](UPSTREAM.json) records the official release archive, its verified
SHA-256 and each header's SHA-256. The archive was checked against Node's release
`SHASUMS256.txt` before extracting the files. [LICENSE](LICENSE) contains the
Node.js MIT license. When updating these headers, preserve them unchanged,
refresh the provenance and run the native build and header regression tests.
