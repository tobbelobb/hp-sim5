import { parse as parseUsda } from "@kroxilon/usda-parser";

/**
 * Open() – the JavaScript twin of Usd.Stage.Open().
 * Accepts either a file path/URL or an in-memory USDA string and
 * returns a Stage-like object with GetPrimAtPath(), Traverse(), and the raw AST.
 */
export async function Open(pathOrSource) {
  const source = isUsdText(pathOrSource)
    ? pathOrSource                              // already the file text
    : await fetch(pathOrSource).then(r => r.text());

  const ast = parseUsda(source);
  const primIndex = indexPrims(ast.statements);

  return {
    /**
     * A very simplified GetPrimAtPath that returns the raw parser node.
     * For a richer API, you might wrap the node in a Prim-like class.
     */
    GetPrimAtPath(path) {
      return primIndex[path] ?? null;
    },

    /** Simple breadth-first traversal generator over [path, prim] pairs. */
    *Traverse() {
      for (const path in primIndex) {
        yield [path, primIndex[path]];
      }
    },

    /** The raw AST from @kroxilon/usda-parser. */
    ast,
  };
}

/* ---------- helpers ---------------------------------------------------- */
function isUsdText(str) {
  // Very cheap test for "#usda 1.0" header.
  return str.startsWith("#usda") || str.startsWith("def ") || str.includes("def ");
}

function indexPrims(statements, parent = "", out = {}) {
  for (const stmt of statements ?? []) {
    if (stmt.type === "definition") {
      const path = `${parent}/${stmt.name}`.replace("//", "/");
      out[path] = stmt;
      if (stmt.statements) {
        indexPrims(stmt.statements, path, out);
      }
    }
  }
  return out;
}
