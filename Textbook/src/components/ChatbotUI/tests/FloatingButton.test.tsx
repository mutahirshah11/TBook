import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import FloatingButton from '../components/FloatingButton';

describe('FloatingButton Component', () => {
  const mockOnClick = jest.fn();

  beforeEach(() => {
    mockOnClick.mockClear();
  });

  test('renders with default closed state', () => {
    render(<FloatingButton onClick={mockOnClick} />);

    const button = screen.getByLabelText('Open chat');
    expect(button).toBeInTheDocument();

    // Check that the closed state icon is present
    const closedIcon = screen.getByTestId('closed-icon');
    expect(closedIcon).toBeInTheDocument();
  });

  test('renders with open state when isOpen is true', () => {
    render(<FloatingButton onClick={mockOnClick} isOpen={true} />);

    const button = screen.getByLabelText('Close chat');
    expect(button).toBeInTheDocument();

    // Check that the open state icon is present
    const openIcon = screen.getByTestId('open-icon');
    expect(openIcon).toBeInTheDocument();
  });

  test('calls onClick handler when clicked', async () => {
    const user = userEvent.setup();
    render(<FloatingButton onClick={mockOnClick} />);

    const button = screen.getByLabelText('Open chat');
    await user.click(button);

    expect(mockOnClick).toHaveBeenCalledTimes(1);
  });

  test('respects disabled prop', () => {
    render(<FloatingButton onClick={mockOnClick} disabled={true} />);

    const button = screen.getByLabelText('Open chat');
    expect(button).toBeDisabled();

    // Try clicking the disabled button
    fireEvent.click(button);
    expect(mockOnClick).not.toHaveBeenCalled();
  });

  test('does not call onClick when disabled', async () => {
    const user = userEvent.setup();
    render(<FloatingButton onClick={mockOnClick} disabled={true} />);

    const button = screen.getByLabelText('Open chat');
    await user.click(button);

    expect(mockOnClick).not.toHaveBeenCalled();
  });

  test('applies correct CSS classes based on state', () => {
    const { rerender } = render(<FloatingButton onClick={mockOnClick} isOpen={false} />);

    const button = screen.getByLabelText('Open chat');
    expect(button).toHaveClass('floating-button--closed');
    expect(button).not.toHaveClass('floating-button--open');

    rerender(<FloatingButton onClick={mockOnClick} isOpen={true} />);

    const openButton = screen.getByLabelText('Close chat');
    expect(openButton).toHaveClass('floating-button--open');
    expect(openButton).not.toHaveClass('floating-button--closed');
  });

  test('applies disabled CSS class when disabled', () => {
    render(<FloatingButton onClick={mockOnClick} disabled={true} />);

    const button = screen.getByLabelText('Open chat');
    expect(button).toHaveClass('floating-button--disabled');
  });

  test('has proper accessibility attributes', () => {
    render(<FloatingButton onClick={mockOnClick} />);

    const button = screen.getByLabelText('Open chat');
    expect(button).toHaveAttribute('role', 'button');
    expect(button).toHaveAttribute('aria-label', 'Open chat');
    expect(button).toHaveAttribute('tabIndex', '0');
  });

  test('changes aria-label based on state', () => {
    const { rerender } = render(<FloatingButton onClick={mockOnClick} isOpen={false} />);

    let button = screen.getByLabelText('Open chat');
    expect(button).toHaveAttribute('aria-label', 'Open chat');

    rerender(<FloatingButton onClick={mockOnClick} isOpen={true} />);

    button = screen.getByLabelText('Close chat');
    expect(button).toHaveAttribute('aria-label', 'Close chat');
  });
});