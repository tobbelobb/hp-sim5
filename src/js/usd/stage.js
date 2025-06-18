import { parse as parseUsda } from "@kroxilon/usda-parser";

/**
 * Open() – the JavaScript twin of Usd.Stage.Open().
 * Accepts either a file path/URL or an in-memory USDA string and
 * returns a Stage instance with GetPrimAtPath(), Traverse(), etc.
 */
export async function Open(pathOrSource) {
  const source = isUsdText(pathOrSource)
    ? pathOrSource                              // already the file text
    : await fetch(pathOrSource).then(r => r.text());

  const ast = parseUsda(source);                // low-level PEG parse :contentReference[oaicite:0]{index=0}
  return new Stage(ast, pathOrSource);
}

/* ---------- Stage – a *very* small subset of the real USD API ---------- */
class Stage {
  constructor(ast, identifier = "<memory>") {
    this.identifier = identifier;
    this.ast        = ast;                      // raw @kroxilon/usda-parser AST
    this._index     = indexPrims(ast.statements);
  }

  /** Mirror of UsdStage.GetPrimAtPath(). Returns the AST node or null. */
  GetPrimAtPath(path) {
    return this._index[path] ?? null;
  }

  /** Simple breadth-first traversal generator (like stage.Traverse()). */
  *Traverse() {
    for (const [path, prim] of Object.entries(this._index)) {
      yield [path, prim];
    }
  }
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
      indexPrims(stmt.statements, path, out);
    }
  }
  return out;
}
