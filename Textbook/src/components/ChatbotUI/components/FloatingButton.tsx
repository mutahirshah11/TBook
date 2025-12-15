import React from 'react';
import clsx from 'clsx';

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
        'floating-button--closed': !isOpen,
        'floating-button--disabled': disabled,
      })}
      aria-label={isOpen ? "Close chat" : "Open chat"}
    >
      <div className="floating-button__content">
        {isOpen ? (
          <svg
            data-testid="open-icon"
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path d="M6 18L18 6M6 6L18 18" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" />
          </svg>
        ) : (
          <svg
            data-testid="closed-icon"
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path d="M8 12H16M12 8V16M21 12C21 16.0732 16.9706 20 12 20C7.02944 20 3 16.0732 3 12C3 7.9268 7.02944 4 12 4C16.9706 4 21 7.9268 21 12Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" />
          </svg>
        )}
      </div>
    </button>
  );
};

export default FloatingButton;