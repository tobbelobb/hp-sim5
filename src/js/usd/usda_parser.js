import {
  parse as parseUsdaPeggy,
  SyntaxError as UsdaSyntaxError,
} from '@kroxilon/usda-parser/lib/usda-parser.peggy.js';

export function parseUsdaAst(source) {
  return parseUsdaPeggy(source);
}

export { UsdaSyntaxError };
