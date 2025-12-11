import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import InputArea from '../components/InputArea';

describe('InputArea Component', () => {
  const mockOnSend = jest.fn();

  beforeEach(() => {
    mockOnSend.mockClear();
  });

  test('renders input area with placeholder', () => {
    render(<InputArea onSend={mockOnSend} />);

    const textarea = screen.getByPlaceholderText('Type your question...');
    expect(textarea).toBeInTheDocument();
    expect(textarea).toHaveAttribute('rows', '1');

    const sendButton = screen.getByLabelText('Send message');
    expect(sendButton).toBeInTheDocument();
  });

  test('allows user to type in the input area', async () => {
    const user = userEvent.setup();
    render(<InputArea onSend={mockOnSend} />);

    const textarea = screen.getByPlaceholderText('Type your question...');

    await user.type(textarea, 'Hello, this is a test message');
    expect(textarea).toHaveValue('Hello, this is a test message');
  });

  test('sends message when send button is clicked', async () => {
    const user = userEvent.setup();
    render(<InputArea onSend={mockOnSend} />);

    const textarea = screen.getByPlaceholderText('Type your question...');
    const sendButton = screen.getByLabelText('Send message');

    await user.type(textarea, 'Test message');
    await user.click(sendButton);

    expect(mockOnSend).toHaveBeenCalledWith('Test message');
    expect(textarea).toHaveValue('');
  });

  test('sends message when Enter key is pressed (without Shift)', async () => {
    const user = userEvent.setup();
    render(<InputArea onSend={mockOnSend} />);

    const textarea = screen.getByPlaceholderText('Type your question...');

    await user.type(textarea, 'Test message{enter}');

    // Wait for the event to be processed
    await waitFor(() => {
      expect(mockOnSend).toHaveBeenCalledWith('Test message');
    });

    expect(textarea).toHaveValue('');
  });

  test('does not send message when Shift+Enter is pressed', async () => {
    const user = userEvent.setup();
    render(<InputArea onSend={mockOnSend} />);

    const textarea = screen.getByPlaceholderText('Type your question...');

    await user.type(textarea, 'Test message{shift}{enter}');

    expect(mockOnSend).not.toHaveBeenCalled();
    expect(textarea).toHaveValue('Test message');
  });

  test('disables send button when input is empty', () => {
    render(<InputArea onSend={mockOnSend} />);

    const sendButton = screen.getByLabelText('Send message');
    expect(sendButton).toBeDisabled();

    const textarea = screen.getByPlaceholderText('Type your question...');
    expect(textarea).toHaveValue('');
  });

  test('enables send button when input has text', async () => {
    const user = userEvent.setup();
    render(<InputArea onSend={mockOnSend} />);

    const textarea = screen.getByPlaceholderText('Type your question...');
    const sendButton = screen.getByLabelText('Send message');

    expect(sendButton).toBeDisabled();

    await user.type(textarea, 'A');
    expect(sendButton).not.toBeDisabled();

    // Clear the input to test button gets disabled again
    await user.clear(textarea);
    expect(sendButton).toBeDisabled();
  });

  test('respects disabled prop', () => {
    render(<InputArea onSend={mockOnSend} disabled={true} />);

    const textarea = screen.getByPlaceholderText('Type your question...');
    const sendButton = screen.getByLabelText('Send message');

    expect(textarea).toBeDisabled();
    expect(sendButton).toBeDisabled();
  });

  test('uses custom placeholder when provided', () => {
    render(<InputArea onSend={mockOnSend} placeholder="Ask anything..." />);

    const textarea = screen.getByPlaceholderText('Ask anything...');
    expect(textarea).toBeInTheDocument();
  });

  test('adjusts textarea height based on content', async () => {
    const user = userEvent.setup();
    render(<InputArea onSend={mockOnSend} />);

    const textarea = screen.getByPlaceholderText('Type your question...');

    // Type a long message to make the textarea expand
    const longMessage = 'This is a long message that should cause the textarea to expand.\n'.repeat(10);
    await user.type(textarea, longMessage);

    // Check that the height has increased
    expect(textarea).toHaveStyle('height: 150px'); // Max height is 150px
  });

  test('does not send empty messages', async () => {
    const user = userEvent.setup();
    render(<InputArea onSend={mockOnSend} />);

    const textarea = screen.getByPlaceholderText('Type your question...');
    const sendButton = screen.getByLabelText('Send message');

    // Try to send an empty message
    await user.click(sendButton);
    expect(mockOnSend).not.toHaveBeenCalled();

    // Try to send a whitespace-only message
    await user.type(textarea, '   ');
    await user.click(sendButton);
    expect(mockOnSend).not.toHaveBeenCalled();

    // Should only send non-empty messages
    await user.clear(textarea);
    await user.type(textarea, 'Valid message');
    await user.click(sendButton);
    expect(mockOnSend).toHaveBeenCalledWith('Valid message');
  });
});