module.exports = {
  presets: [['@babel/preset-env', { targets: { node: 'current' } }]],
  env: { test: { plugins: [({ types: t }) => ({ visitor: {
    // Jest runs transformed modules as CommonJS. Preserve module-relative
    // artifact lookup when the production source uses import.meta.url.
    MemberExpression(path) {
      if (t.isMetaProperty(path.node.object) && path.node.object.meta.name === 'import'
          && t.isIdentifier(path.node.property, { name: 'url' })) {
        path.replaceWith(t.memberExpression(t.callExpression(t.memberExpression(
          t.callExpression(t.identifier('require'), [t.stringLiteral('node:url')]),
          t.identifier('pathToFileURL')), [t.identifier('__filename')]), t.identifier('href')));
      }
    },
  } })] } },
};
