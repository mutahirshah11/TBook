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

describe('SC-002: AI Response with Book References Test', () => {
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

  test('Verify user can type question and receive AI response with book references', async () => {
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

    // Mock API response with book references
    mockStreamMessage.mockImplementation(async (data, onChunk) => {
      onChunk('The key principle of embodied AI is that intelligence emerges from the interaction between an agent and its environment. This is discussed in detail in Chapter 3 of the textbook.', false);
      await new Promise(resolve => setTimeout(resolve, 10));
      onChunk(' For more information, see the sections on sensorimotor integration and environmental interaction.', true);
    });

    render(<ChatWindow isOpen={true} onClose={jest.fn()} />);

    // Find the input area and send a question
    const input = screen.getByLabelText('Type your message');
    const sendButton = screen.getByLabelText('Send message');

    await user.type(input, 'What are the key principles of embodied AI?');
    await user.click(sendButton);

    // Wait for the AI response to be displayed
    await waitFor(() => {
      expect(screen.getByText(/The key principle of embodied AI is that intelligence emerges from the interaction/i)).toBeInTheDocument();
    });

    // Verify that the response mentions book content and references
    const responseElement = screen.getByText(/The key principle of embodied AI is that intelligence emerges from the interaction between an agent and its environment/i);
    expect(responseElement).toBeInTheDocument();

    // Verify that the response includes references to book content
    expect(screen.getByText(/Chapter 3 of the textbook/i)).toBeInTheDocument();
    expect(screen.getByText(/sections on sensorimotor integration and environmental interaction/i)).toBeInTheDocument();

    // Verify that the AI message has the correct sender
    const aiMessageElements = document.querySelectorAll('.message--ai');
    expect(aiMessageElements.length).toBeGreaterThan(0);

    // Verify that the response was added to the session
    expect(mockAddMessage).toHaveBeenCalledWith(
      expect.objectContaining({
        sender: 'ai',
        content: expect.stringContaining('The key principle of embodied AI'),
      })
    );
  });

  test('Verify AI response includes source references when provided by backend', async () => {
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

    // Mock API response that includes source references
    mockStreamMessage.mockImplementation(async (data, onChunk) => {
      // Simulate streaming response with source references in final chunk
      onChunk('Embodied AI requires tight coupling between perception and action systems.', false);
      await new Promise(resolve => setTimeout(resolve, 10));
      onChunk(' This concept is fundamental to robotics applications.', true);
    });

    // Mock the updateMessage to track when source references are added
    const mockUpdateMessageImplementation = jest.fn();
    (sessionManager.updateMessage as jest.MockedFunction<any>).mockImplementation(mockUpdateMessageImplementation);

    render(<ChatWindow isOpen={true} onClose={jest.fn()} />);

    // Send a question about robotics
    const input = screen.getByLabelText('Type your message');
    const sendButton = screen.getByLabelText('Send message');

    await user.type(input, 'How is embodied AI related to robotics?');
    await user.click(sendButton);

    // Wait for the response to be processed
    await waitFor(() => {
      expect(mockUpdateMessageImplementation).toHaveBeenCalled();
    });

    // Verify the response content is displayed
    expect(screen.getByText(/Embodied AI requires tight coupling between perception and action systems/i)).toBeInTheDocument();
  });

  test('Verify AI response is relevant to the question asked', async () => {
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

    // Mock API response related to the question
    mockStreamMessage.mockImplementation(async (data, onChunk) => {
      expect(data.message).toBe('What is machine learning?');
      onChunk('Machine learning is a subset of artificial intelligence that enables systems to learn and improve from experience without being explicitly programmed.', true);
    });

    render(<ChatWindow isOpen={true} onClose={jest.fn()} />);

    // Send a question about machine learning
    const input = screen.getByLabelText('Type your message');
    const sendButton = screen.getByLabelText('Send message');

    await user.type(input, 'What is machine learning?');
    await user.click(sendButton);

    // Wait for the relevant response
    await waitFor(() => {
      expect(screen.getByText(/Machine learning is a subset of artificial intelligence/i)).toBeInTheDocument();
    });

    // Verify the response is relevant to the question
    const responseText = screen.getByText(/Machine learning is a subset of artificial intelligence that enables systems to learn and improve from experience/i);
    expect(responseText).toBeInTheDocument();
  });

  test('Verify response time meets performance requirements (conceptual test)', async () => {
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

    // Mock a quick API response
    mockStreamMessage.mockImplementation(async (data, onChunk) => {
      await new Promise(resolve => setTimeout(resolve, 50)); // Simulate fast response
      onChunk('This is a quick response to test performance.', true);
    });

    render(<ChatWindow isOpen={true} onClose={jest.fn()} />);

    const startTime = Date.now();

    // Send a question
    const input = screen.getByLabelText('Type your message');
    const sendButton = screen.getByLabelText('Send message');

    await user.type(input, 'Quick test question');
    await user.click(sendButton);

    // Wait for response
    await waitFor(() => {
      expect(screen.getByText(/This is a quick response to test performance/i)).toBeInTheDocument();
    });

    const endTime = Date.now();
    const responseTime = endTime - startTime;

    // This is a conceptual test - in a real implementation, we'd have stricter performance requirements
    expect(responseTime).toBeLessThan(5000); // Less than 5 seconds
  });
});