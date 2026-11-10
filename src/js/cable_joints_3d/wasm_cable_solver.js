export async function compileCableSolverWasm(bytes) {
  return WebAssembly.compile(bytes);
}
