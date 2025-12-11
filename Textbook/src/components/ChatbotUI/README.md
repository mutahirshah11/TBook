# Chatbot UI Component

This directory contains the React components for the Docusaurus RAG Chatbot interface.

## Structure

```
src/components/ChatbotUI/
├── components/      # React UI components
│   ├── ChatWindow.tsx        # Main chat interface window
│   ├── Message.tsx           # Individual message display
│   ├── InputArea.tsx         # Message input and submission
│   ├── FloatingButton.tsx    # Floating chat trigger button
│   └── ThemeProvider.tsx     # Theme management for light/dark modes
├── utils/          # Utility functions
│   ├── sessionManager.ts     # Session state management
│   ├── textSelection.ts      # Selected text handling
│   └── apiClient.ts          # API communication layer
├── types/          # TypeScript type definitions
│   ├── index.ts              # Main type definitions
│   └── api.ts                # API-related types
├── tests/          # Test files (TDD approach)
│   ├── ChatWindow.test.tsx
│   ├── Message.test.tsx
│   ├── InputArea.test.tsx
│   ├── FloatingButton.test.tsx
│   ├── sessionManager.test.ts
│   └── textSelection.test.ts
├── ChatbotUI.css   # Styles for the chatbot components
├── ChatbotProvider.tsx  # Provider component that manages the chatbot state
└── README.md
```

## Features

- Floating chat widget that appears on all book pages
- Scrollable conversation history with clear user/AI differentiation
- Real-time response streaming from the RAG system
- Session management within browser session
- Selected-text queries for context-aware responses
- Responsive design for desktop, tablet, and mobile
- Thematic consistency with book's light/dark modes
- Full accessibility support (WCAG 2.1 AA)

## Integration

The chatbot is integrated into the Docusaurus site through:
- Root component at `src/theme/Root.tsx` which wraps the entire app
- Automatic text selection detection across all pages
- Floating button that appears on all routes
- Session persistence across page navigation

## Development

This component follows Test-Driven Development principles. All tests must pass before any code changes are made:

1. Write tests for new functionality before implementing
2. Run tests to confirm they fail initially
3. Implement minimal code to pass tests
4. Refactor while keeping tests passing

## Usage

The chatbot is automatically available on all pages of the Docusaurus site. No additional setup is required after installation.