# Quickstart: Docusaurus RAG Chatbot UI

## Prerequisites
- Node.js 18+
- Docusaurus 2.x project
- Access to FastAPI backend with RAG endpoints
- OpenAI ChatKit compatible environment

## Installation

1. **Install dependencies**:
```bash
npm install @openai/chat-components react react-dom
# If not already installed
npm install @docusaurus/core @docusaurus/module-type-aliases @docusaurus/types
```

2. **Create the ChatbotUI component directory**:
```bash
mkdir -p src/components/ChatbotUI/{components,utils,types,tests}
```

3. **Set up the basic component structure**:
```bash
# Component files
touch src/components/ChatbotUI/components/{ChatWindow,Message,InputArea,FloatingButton,ThemeProvider}.tsx
# Utility files
touch src/components/ChatbotUI/utils/{sessionManager,textSelection,apiClient}.ts
# Type definitions
touch src/components/ChatbotUI/types/{index,api}.ts
# Test files
touch src/components/ChatbotUI/tests/{ChatWindow,Message,InputArea,FloatingButton,sessionManager,textSelection}.test.tsx
```

## Basic Setup

1. **Create the main ChatWindow component**:
```tsx
// src/components/ChatbotUI/components/ChatWindow.tsx
import React, { useState, useEffect } from 'react';
import { ConversationSession, Message } from '../types';

interface ChatWindowProps {
  isOpen: boolean;
  onClose: () => void;
  selectedText?: string;
}

const ChatWindow: React.FC<ChatWindowProps> = ({ isOpen, onClose, selectedText }) => {
  const [session, setSession] = useState<ConversationSession | null>(null);
  const [messages, setMessages] = useState<Message[]>([]);

  // Implementation will follow TDD approach with tests first

  if (!isOpen) return null;

  return (
    <div className="chat-window">
      {/* Chat interface implementation */}
    </div>
  );
};

export default ChatWindow;
```

2. **Integrate with Docusaurus**:
```tsx
// docs/src/theme/ChatbotProvider.tsx
import React, { createContext, useContext } from 'react';
import FloatingButton from '../../src/components/ChatbotUI/components/FloatingButton';

const ChatbotContext = createContext<any>(null);

export const ChatbotProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  return (
    <ChatbotContext.Provider value={{}}>
      {children}
      <FloatingButton />
    </ChatbotContext.Provider>
  );
};
```

3. **Update Docusaurus config** to use the provider in your `src/theme/Layout` or appropriate wrapper.

## Running Tests

1. **Unit tests**:
```bash
npm run test -- --testPathPattern=ChatbotUI
```

2. **Integration tests**:
```bash
npm run test:integration -- --testPathPattern=chatbot
```

## Development Workflow (TDD)

1. Write failing tests for each component before implementation
2. Implement minimal code to pass tests
3. Refactor while keeping tests passing
4. Repeat for each feature incrementally

Example test-first approach:
```tsx
// First write the test
test('ChatWindow renders with initial state', () => {
  const { getByRole } = render(<ChatWindow isOpen={true} onClose={jest.fn()} />);
  expect(getByRole('dialog')).toBeInTheDocument();
});

// Then implement the component to make the test pass
```

## Environment Configuration

Set the following environment variables:
```
REACT_APP_API_BASE_URL=http://localhost:8000  # Your FastAPI backend URL
REACT_APP_CHAT_TIMEOUT=30000  # 30 second timeout for chat requests
```

## Next Steps

1. Implement the core components following the data model
2. Write comprehensive tests for each component
3. Integrate with the existing FastAPI backend
4. Test accessibility compliance (WCAG 2.1 AA)
5. Verify performance targets (responses within 2 seconds)