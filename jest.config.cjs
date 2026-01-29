module.exports = {
  testEnvironment: 'node',
  testMatch: [
    '<rootDir>/tests/**/*.test.js',
    '<rootDir>/autocal/control/tests/behaviors/**/*.test.mjs',
    '<rootDir>/autocal/control/tests/cli/**/*.test.mjs',
    '<rootDir>/autocal/control/tests/primitives/**/*.test.mjs',
  ],
  transform: {
    '^.+\\.[jt]sx?$': 'babel-jest',
    '^.+\\.mjs$': 'babel-jest',
  },
  transformIgnorePatterns: [
    '/node_modules/(?!(\\@kroxilon\\/usda-parser))'
  ],
};
