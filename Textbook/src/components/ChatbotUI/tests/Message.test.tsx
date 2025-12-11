import React from 'react';
import { render, screen } from '@testing-library/react';
import Message from '../components/Message';
import { Message as MessageType } from '../types';

// Mock message data for testing
const mockUserMessage: MessageType = {
  id: '1',
  content: 'Hello, this is a user message',
  sender: 'user',
  timestamp: new Date('2025-12-11T10:00:00Z'),
  status: 'delivered',
};

const mockAiMessage: MessageType = {
  id: '2',
  content: 'Hello, this is an AI response',
  sender: 'ai',
  timestamp: new Date('2025-12-11T10:01:00Z'),
  sourceReferences: ['/chapter-1', '/chapter-2'],
};

describe('Message Component', () => {
  test('renders user message correctly', () => {
    render(<Message message={mockUserMessage} />);

    // Check that the message content is rendered
    expect(screen.getByText('Hello, this is a user message')).toBeInTheDocument();

    // Check that user-specific classes are applied
    const messageElement = screen.getByText('Hello, this is a user message').closest('.message');
    expect(messageElement).toHaveClass('message--user');

    // Check that timestamp is rendered
    expect(screen.getByText('10:00')).toBeInTheDocument();

    // Check that status is rendered
    expect(screen.getByText('delivered')).toBeInTheDocument();
  });

  test('renders AI message correctly', () => {
    render(<Message message={mockAiMessage} />);

    // Check that the message content is rendered
    expect(screen.getByText('Hello, this is an AI response')).toBeInTheDocument();

    // Check that AI-specific classes are applied
    const messageElement = screen.getByText('Hello, this is an AI response').closest('.message');
    expect(messageElement).toHaveClass('message--ai');

    // Check that timestamp is rendered
    expect(screen.getByText('10:01')).toBeInTheDocument();
  });

  test('renders message without status correctly', () => {
    const messageWithoutStatus = {
      ...mockUserMessage,
      status: undefined,
    };

    render(<Message message={messageWithoutStatus} />);

    // Check that the message content is rendered
    expect(screen.getByText('Hello, this is a user message')).toBeInTheDocument();

    // Check that timestamp is rendered
    expect(screen.getByText('10:00')).toBeInTheDocument();

    // Ensure status element is not rendered when status is undefined
    expect(screen.queryByText('delivered')).not.toBeInTheDocument();
  });

  test('applies correct CSS classes based on message status', () => {
    const messageWithErrorStatus = {
      ...mockUserMessage,
      status: 'error',
    };

    render(<Message message={messageWithErrorStatus} />);

    const messageElement = screen.getByText('Hello, this is a user message').closest('.message');
    expect(messageElement).toHaveClass('message--error');
  });

  test('formats timestamp correctly', () => {
    const messageWithSpecificTime = {
      ...mockUserMessage,
      timestamp: new Date('2025-12-11T15:30:45Z'),
    };

    render(<Message message={messageWithSpecificTime} />);

    // Check that the time is formatted as HH:MM (e.g., 15:30)
    expect(screen.getByText('15:30')).toBeInTheDocument();
  });

  test('renders message with context', () => {
    const messageWithContext = {
      ...mockUserMessage,
      context: 'Some selected text context',
    };

    render(<Message message={messageWithContext} />);

    // The context should not be directly visible in the message content
    expect(screen.getByText('Hello, this is a user message')).toBeInTheDocument();
    // The context might be used internally but not displayed directly
  });
});