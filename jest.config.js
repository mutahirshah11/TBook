module.exports = {
  preset: 'ts-jest',
  testEnvironment: 'jsdom',
  moduleFileExtensions: ['ts', 'tsx', 'js', 'jsx', 'json', 'node'],
  setupFilesAfterEnv: ['<rootDir>/src/components/ChatbotUI/setupTests.ts'],
  testMatch: [
    '<rootDir>/src/components/ChatbotUI/**/*.(test|spec).(ts|tsx|js|jsx)',
    '<rootDir>/src/components/ChatbotUI/tests/**/*.(test|spec).(ts|tsx|js|jsx)'
  ],
  transform: {
    '^.+\\.(ts|tsx)$': 'ts-jest',
    '^.+\\.(js|jsx)$': 'babel-jest'
  },
  moduleNameMapper: {
    '\\.(css|less|scss|sass)$': 'identity-obj-proxy',
    '^@components/(.*)$': '<rootDir>/src/components/$1',
    '^@utils/(.*)$': '<rootDir>/src/utils/$1',
    '^@types/(.*)$': '<rootDir>/src/types/$1'
  },
  collectCoverageFrom: [
    'src/components/ChatbotUI/**/*.{ts,tsx}',
    '!src/components/ChatbotUI/**/*.d.ts',
    '!src/components/ChatbotUI/**/node_modules/**'
  ],
  coverageThreshold: {
    global: {
      branches: 80,
      functions: 80,
      lines: 80,
      statements: 80
    }
  }
};