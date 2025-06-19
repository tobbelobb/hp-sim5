import { parse as parseUsda } from "@kroxilon/usda-parser";

/**
 * Open() – the JavaScript twin of Usd.Stage.Open().
 * Accepts either a file path/URL or an in-memory USDA string and
 * returns a Stage-like object with GetPrimAtPath(), Traverse(), and the raw AST.
 */
export async function Open(pathOrSource) {
  const source = isUsdText(pathOrSource)
    ? pathOrSource                              // already the file text
    : await fetch(pathOrSource).then(async (r) => {
        if (!r.ok) {
          throw new Error(`Failed to fetch ${r.url}: ${r.status} ${r.statusText}. Check that the file path is correct and the server is configured to serve it.`);
        }
        const text = await r.text();
        // A common error is for the server to return an HTML 404 page.
        // This provides a better error message than the parser's syntax error.
        if (text.trim().startsWith("<")) {
          throw new Error(
            `Fetch for ${r.url} returned HTML, not a USDA file. Check that the file path is correct.`
          );
        }
        return text;
      });

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


// --- USD Parsing Helpers ---
export function getAttribute(primNode, attrName) {
    if (!primNode || !primNode.statements) return null;
    const decl = primNode.statements.find(s => s.type === 'declaration' && s.reference === attrName);
    return decl ? decl.value : null;
}

export function getRelationship(primNode, relName) {
    const value = getAttribute(primNode, relName);
    if (!value) return [];
    // The new parser returns an object or an array of objects for relationships
    if (Array.isArray(value)) {
        // It's a list of relationships
        return value.map(v => v.importPath).filter(Boolean);
    }
    if (value.importPath) {
        // It's a single relationship
        return [value.importPath];
    }
    return [];
}

export function getChildren(primNode) {
    return primNode?.statements?.filter(s => s.type === 'definition') ?? [];
}

export function getChild(primNode, name) {
    return primNode?.statements?.find(s => s.type === 'definition' && s.name === name);
}

export function materialProperties(stage, prim) {
    const materialPath = getRelationship(prim, 'material:binding')[0];
    if (!materialPath) return { color: null, friction: null, restitution: null };

    const matPrim = stage.GetPrimAtPath(materialPath);
    if (!matPrim) return { color: null, friction: null, restitution: null };

    const shaderPrim = getChild(matPrim, "Shader");
    let color = null;
    if (shaderPrim) {
        const colorVal = getAttribute(shaderPrim, "inputs:diffuseColor");
        if (colorVal) {
            const toHex = c => Math.round(c * 255).toString(16).padStart(2, '0');
            color = `#${toHex(colorVal[0])}${toHex(colorVal[1])}${toHex(colorVal[2])}`;
        }
    }

    const friction = getAttribute(matPrim, "physics:staticFriction");
    const restitution = getAttribute(matPrim, "physics:restitution");

    return { color, friction, restitution };
}

