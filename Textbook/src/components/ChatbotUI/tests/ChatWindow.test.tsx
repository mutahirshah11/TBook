import React from 'react';
import { render, screen, waitFor, fireEvent } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import ChatWindow from '../components/ChatWindow';
import { sessionManager } from '../utils/sessionManager';
import { apiClient } from '../utils/apiClient';

// Mock the API client
jest.mock('../utils/apiClient', () => ({
  apiClient: {
    sendMessage: jest.fn(),
    streamMessage: jest.fn(),
    createSession: jest.fn(),
    getSession: jest.fn(),
  },
}));

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

describe('ChatWindow Component Integration Tests', () => {
  const mockSendMessage = apiClient.sendMessage as jest.MockedFunction<any>;
  const mockStreamMessage = apiClient.streamMessage as jest.MockedFunction<any>;
  const mockCreateSession = apiClient.createSession as jest.MockedFunction<any>;
  const mockGetSession = apiClient.getSession as jest.MockedFunction<any>;

  const mockGetSessionFunc = sessionManager.getSession as jest.MockedFunction<any>;
  const mockCreateSessionFunc = sessionManager.createSession as jest.MockedFunction<any>;
  const mockAddMessage = sessionManager.addMessage as jest.MockedFunction<any>;
  const mockUpdateMessage = sessionManager.updateMessage as jest.MockedFunction<any>;

  beforeEach(() => {
    jest.clearAllMocks();

    // Default mock implementations
    mockGetSessionFunc.mockReturnValue(null);
    mockCreateSessionFunc.mockReturnValue({
      id: 'test-session-123',
      messages: [],
      createdAt: new Date(),
      lastActiveAt: new Date(),
      isActive: true,
    });
  });

  test('renders chat window when isOpen is true', () => {
    render(<ChatWindow isOpen={true} onClose={jest.fn()} />);

    expect(screen.getByLabelText('Chat with Book Assistant')).toBeInTheDocument();
    expect(screen.getByRole('banner')).toBeInTheDocument();
    expect(screen.getByText('Book Assistant')).toBeInTheDocument();
    expect(screen.getByLabelText('Close chat')).toBeInTheDocument();
    expect(screen.getByLabelText('Chat input area')).toBeInTheDocument();
  });

  test('does not render chat window when isOpen is false', () => {
    render(<ChatWindow isOpen={false} onClose={jest.fn()} />);

    expect(screen.queryByLabelText('Chat with Book Assistant')).not.toBeInTheDocument();
  });

  test('initializes session when opened', async () => {
    const mockSession = {
      id: 'test-session-123',
      messages: [],
      createdAt: new Date(),
      lastActiveAt: new Date(),
      isActive: true,
    };

    mockGetSessionFunc.mockReturnValue(mockSession);

    render(<ChatWindow isOpen={true} onClose={jest.fn()} />);

    await waitFor(() => {
      expect(mockGetSessionFunc).toHaveBeenCalled();
    });
  });

  test('sends message and updates UI correctly', async () => {
    const user = userEvent.setup();

    // Mock session data
    const mockSession = {
      id: 'test-session-123',
      messages: [],
      createdAt: new Date(),
      lastActiveAt: new Date(),
      isActive: true,
    };

    mockGetSessionFunc.mockReturnValue(mockSession);

    // Mock API response
    mockStreamMessage.mockImplementation(async (data, onChunk) => {
      onChunk('This is a test response', true);
    });

    render(<ChatWindow isOpen={true} onClose={jest.fn()} />);

    // Find the input area and send a message
    const input = screen.getByLabelText('Type your message');
    const sendButton = screen.getByLabelText('Send message');

    await user.type(input, 'Hello, this is a test message');
    await user.click(sendButton);

    // Wait for the message to be processed
    await waitFor(() => {
      expect(mockAddMessage).toHaveBeenCalledTimes(2); // User message + AI response
    });

    // Check that the message was sent via the API
    expect(mockStreamMessage).toHaveBeenCalledWith(
      expect.objectContaining({
        message: 'Hello, this is a test message',
        sessionId: 'test-session-123',
      }),
      expect.any(Function)
    );
  });

  test('displays error when message sending fails', async () => {
    const user = userEvent.setup();

    // Mock session data
    const mockSession = {
      id: 'test-session-123',
      messages: [],
      createdAt: new Date(),
      lastActiveAt: new Date(),
      isActive: true,
    };

    mockGetSessionFunc.mockReturnValue(mockSession);

    // Mock API error
    mockStreamMessage.mockRejectedValue(new Error('Network error'));

    render(<ChatWindow isOpen={true} onClose={jest.fn()} />);

    // Find the input area and send a message
    const input = screen.getByLabelText('Type your message');
    const sendButton = screen.getByLabelText('Send message');

    await user.type(input, 'Hello, this is a test message');
    await user.click(sendButton);

    // Wait for the error to be displayed
    await waitFor(() => {
      expect(screen.getByText('Network error')).toBeInTheDocument();
    });
  });

  test('shows loading state when sending message', async () => {
    const user = userEvent.setup();

    // Mock session data
    const mockSession = {
      id: 'test-session-123',
      messages: [],
      createdAt: new Date(),
      lastActiveAt: new Date(),
      isActive: true,
    };

    mockGetSessionFunc.mockReturnValue(mockSession);

    // Mock API to delay response
    mockStreamMessage.mockImplementation(async (data, onChunk) => {
      await new Promise(resolve => setTimeout(resolve, 100));
      onChunk('This is a test response', true);
    });

    render(<ChatWindow isOpen={true} onClose={jest.fn()} />);

    // Find the input area and send a message
    const input = screen.getByLabelText('Type your message');
    const sendButton = screen.getByLabelText('Send message');

    await user.type(input, 'Hello, this is a test message');
    await user.click(sendButton);

    // Check that the input is disabled during loading
    expect(sendButton).toBeDisabled();
  });

  test('closes chat window when close button is clicked', async () => {
    const mockOnClose = jest.fn();
    const user = userEvent.setup();

    render(<ChatWindow isOpen={true} onClose={mockOnClose} />);

    const closeButton = screen.getByLabelText('Close chat');
    await user.click(closeButton);

    expect(mockOnClose).toHaveBeenCalledTimes(1);
  });

  test('displays selected text preview when provided', () => {
    render(
      <ChatWindow
        isOpen={true}
        onClose={jest.fn()}
        selectedText="This is some selected text from the page"
      />
    );

    expect(screen.getByText('Selected: "This is some selected text from the page"')).toBeInTheDocument();
  });

  test('handles streaming response correctly', async () => {
    const user = userEvent.setup();

    // Mock session data
    const mockSession = {
      id: 'test-session-123',
      messages: [],
      createdAt: new Date(),
      lastActiveAt: new Date(),
      isActive: true,
    };

    mockGetSessionFunc.mockReturnValue(mockSession);

    // Mock streaming API response with multiple chunks
    mockStreamMessage.mockImplementation(async (data, onChunk) => {
      onChunk('This is the first part of the ', false);
      await new Promise(resolve => setTimeout(resolve, 10));
      onChunk('response that comes in chunks.', true);
    });

    render(<ChatWindow isOpen={true} onClose={jest.fn()} />);

    // Find the input area and send a message
    const input = screen.getByLabelText('Type your message');
    const sendButton = screen.getByLabelText('Send message');

    await user.type(input, 'Hello, stream a long response');
    await user.click(sendButton);

    // Wait for the streaming to complete
    await waitFor(() => {
      expect(mockUpdateMessage).toHaveBeenCalled();
    });
  });
});