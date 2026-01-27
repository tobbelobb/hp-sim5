export default {
  testEnvironment: 'node',
  transform: {
    '^.+\\.[jt]sx?$': 'babel-jest',
    '^.+\\.mjs$': 'babel-jest',
  },
  transformIgnorePatterns: [
    '/node_modules/(?!(\\@kroxilon\\/usda-parser))'
  ]
};
