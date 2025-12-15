import React from 'react';
import { render, screen } from '@testing-library/react';
import ChatWindow from '../components/ChatWindow';
import { sessionManager } from '../utils/sessionManager';

// Mock the session manager
jest.mock('../utils/sessionManager', () => ({
  sessionManager: {
    getSession: jest.fn(),
    createSession: jest.fn(),
    addMessage: jest.fn(),
    updateMessage: jest.fn(),
    getMessages: jest.fn(),
    clearSession: jest.fn(),
    isSessionActive: jest.fn(),
  },
}));

describe('SC-001: Chat Interface Display Test', () => {
  const mockGetSession = sessionManager.getSession as jest.MockedFunction<any>;
  const mockCreateSession = sessionManager.createSession as jest.MockedFunction<any>;

  beforeEach(() => {
    jest.clearAllMocks();

    // Default mock implementations
    mockGetSession.mockReturnValue(null);
    mockCreateSession.mockReturnValue({
      id: 'test-session-123',
      messages: [],
      createdAt: new Date(),
      lastActiveAt: new Date(),
      isActive: true,
    });
  });

  test('Verify chat interface appears with clear input area and message history display', () => {
    render(<ChatWindow isOpen={true} onClose={jest.fn()} />);

    // Verify the chat window is displayed
    expect(screen.getByLabelText('Chat with Book Assistant')).toBeInTheDocument();

    // Verify the header is present
    expect(screen.getByText('Book Assistant')).toBeInTheDocument();

    // Verify the close button is present
    expect(screen.getByLabelText('Close chat')).toBeInTheDocument();

    // Verify the input area is present and clearly marked
    const inputArea = screen.getByLabelText('Chat input area');
    expect(inputArea).toBeInTheDocument();

    // Verify the message input field is present and clearly marked
    const messageInput = screen.getByLabelText('Type your message');
    expect(messageInput).toBeInTheDocument();
    expect(messageInput).toHaveAttribute('role', 'textbox');
    expect(messageInput).toHaveAttribute('aria-multiline', 'true');

    // Verify the send button is present
    expect(screen.getByLabelText('Send message')).toBeInTheDocument();

    // Verify the messages container is present
    const messagesContainer = screen.getByLabelText('Conversation history');
    expect(messagesContainer).toBeInTheDocument();
    expect(messagesContainer).toHaveAttribute('role', 'log');

    // Verify the empty state message is present when no messages exist
    expect(screen.getByText('Ask me anything about the book content!')).toBeInTheDocument();

    // Verify the messages container has the proper accessibility attributes
    expect(messagesContainer).toHaveAttribute('aria-live', 'polite');

    // Verify the overall structure of the chat window
    const chatWindow = screen.getByLabelText('Chat with Book Assistant');
    expect(chatWindow).toHaveClass('chat-window');

    // Check that the header section exists
    const header = screen.getByRole('banner');
    expect(header).toBeInTheDocument();

    // Check that the input container exists
    const inputContainer = screen.getByRole('complementary');
    expect(inputContainer).toBeInTheDocument();
  });

  test('Verify message history displays properly when messages exist', () => {
    // Mock a session with existing messages
    const mockSessionWithMessages = {
      id: 'test-session-123',
      messages: [
        {
          id: 'msg-1',
          content: 'Hello, this is a test message',
          sender: 'user',
          timestamp: new Date(),
          status: 'delivered'
        },
        {
          id: 'msg-2',
          content: 'This is the AI response',
          sender: 'ai',
          timestamp: new Date(),
          sourceReferences: ['/chapter-1']
        }
      ],
      createdAt: new Date(),
      lastActiveAt: new Date(),
      isActive: true,
    };

    mockGetSession.mockReturnValue(mockSessionWithMessages);

    render(<ChatWindow isOpen={true} onClose={jest.fn()} />);

    // Verify the messages are displayed
    expect(screen.getByText('Hello, this is a test message')).toBeInTheDocument();
    expect(screen.getByText('This is the AI response')).toBeInTheDocument();

    // Verify the message history container still exists
    const messagesContainer = screen.getByLabelText('Conversation history');
    expect(messagesContainer).toBeInTheDocument();

    // Verify the messages are in a list structure
    const messageList = screen.getByRole('list');
    expect(messageList).toBeInTheDocument();
    expect(messageList).toHaveAttribute('aria-label', 'Messages');
  });
});