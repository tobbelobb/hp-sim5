## 1  Installation

```bash
npm i @kroxilon/usda-parser   # or:  yarn add @kroxilon/usda-parser
```

---

## 2  Hello world

```ts
import { parse } from "@kroxilon/usda-parser";

const txt = `#usda 1.0\n\ndef Cube \"MyCube\" { double size = 2 }`;

const ast = parse(txt);
console.dir(ast, { depth: Infinity });
```

<details>
<summary>Result (trimmed)</summary>

```jsonc
{
  "version": 1,
  "descriptor": null,
  "statements": [
    {
      "type": "definition",        // ← DefinitionNode
      "defType": "Cube",
      "name": "MyCube",
      "statements": [
        {
          "type": "declaration",   // ← DeclarationNode
          "defineType": "double",
          "reference": "size",
          "value": 2
        }
      ]
    }
  ]
}
```

</details>

---

## 3  API surface

| Function / type                      | Description                                                                                  |
| ------------------------------------ | -------------------------------------------------------------------------------------------- |
| `parse(source: string): ParseResult` | Convert a USDA *text* file to a **ParseResult** AST (plain JS objects).                      |
| <code>type ParseResult</code>        | Root node: `{ version: number; descriptor?: DescriptorNode; statements: Node[] }`            |
| <code>type Node</code>               | Union of<br>`DefinitionNode` · `DeclarationNode` · `AssignmentNode` · `CommentNode`          |
| `DefinitionNode`                     | `{ type:"definition"; defType:string; name:string; statements:Node[] }`                      |
| `DeclarationNode`                    | `{ type:"declaration"; defineType:string; reference:string; value:any }`                     |
| `AssignmentNode`                     | `{ type:"assignment"; identifier:string; value:any }`                                        |
| `DescriptorNode`                     | Optional paren‑block that precedes a prim or the stage, holds `assignments` + `description`. |

> ⚠️  The parser **does not evaluate** or compose USD layers; it only gives you the concrete syntax tree.

---

## 4  Common helpers (copy‑paste)

```ts
/** Walk every node depth‑first */
export function* traverse(node: { statements?: any[] }) {
  for (const child of node.statements ?? []) {
    yield child;
    if (child.statements) yield* traverse(child);
  }
}

/** Index prims by absolute path “/World/Cube” */
export function indexPrims(root: ParseResult) {
  const map: Record<string, any> = {};
  function visit(n: any, path = "") {
    if (n.type === "definition") {
      const p = `${path}/${n.name}`.replace(/\/+/g, "/");
      map[p] = n;
      n.statements?.forEach(s => visit(s, p));
    }
  }
  root.statements.forEach(s => visit(s));
  return map;
}
```

---

## 5  Turning it into a “Stage” (Python‑style)

```ts
import { parse } from "@kroxilon/usda-parser";

export async function Open(input: string | URL | Blob) {
  const text = typeof input === "string" && (input.startsWith("#usda") || input.includes("def "))
    ? input                      // already source
    : await fetch(input).then(r => r.text());

  const ast    = parse(text);
  const idx    = indexPrims(ast);   // from §4

  return {
    /** Return the raw AST node for a prim path or `null`. */
    GetPrimAtPath: (p: string) => idx[p] ?? null,
    /** Breadth‑first iterator over [path,node] pairs. */
    Traverse: function* () { for (const k in idx) yield [k, idx[k]] as const; },
    /** Expose the parse result for advanced queries */
    ast,
  } as const;
}
```

Now you can write:

```ts
const stage = await Open("/assets/scene.usda");
const cube  = stage.GetPrimAtPath("/World/MyCube");
```

---

## 6  Docstrings for AI models (TS‑doc / JSDoc)

```ts
/**
 * Parse a USDA string into an AST describing the file’s concrete syntax.
 * @param source USDA text (must start with `#usda` or a `def` keyword).
 * @returns {ParseResult} – see types above. Note: this **does not** resolve
 *          references, payloads, or layer composition.
 */
export declare function parse(source: string): ParseResult;
```

Place snippets like the above next to your wrapper so LLMs have context while
analyzing your repo.

* Keep descriptions **short & atomic** (<= 3 sentences).
* Mention edge‑cases explicitly (e.g. *"USDC not supported"*).
* Where relevant, add tiny usage examples in the docstring body.

---

## 7  Further reading

* Original README with full example output (GitHub)
  [https://github.com/Kroxilon/usda-parser](https://github.com/Kroxilon/usda-parser)
* USD specification: [https://openusd.org/release/spec\_usda.html](https://openusd.org/release/spec_usda.html)
* Tests inside `@kroxilon/usda-parser/test` show plenty of real‑world files.
