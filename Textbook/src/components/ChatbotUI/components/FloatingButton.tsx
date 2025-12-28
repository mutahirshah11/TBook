import React from 'react';
import clsx from 'clsx';
import { MessageSquare, X } from 'lucide-react';

interface FloatingButtonProps {
  onClick: () => void;
  isOpen?: boolean;
  disabled?: boolean;
}

const FloatingButton: React.FC<FloatingButtonProps> = ({ onClick, isOpen = false, disabled = false }) => {
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      className={clsx('floating-button', {
        'floating-button--open': isOpen,
      })}
      aria-label={isOpen ? "Close chat" : "Open chat"}
    >
        {isOpen ? <X size={28} /> : <MessageSquare size={28} />}
    </button>
  );
};

export default FloatingButton;