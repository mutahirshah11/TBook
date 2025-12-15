import React, { useState, useRef, KeyboardEvent } from 'react';
import clsx from 'clsx';

interface InputAreaProps {
  onSend: (message: string, selectedText?: string) => void;
  disabled?: boolean;
  placeholder?: string;
}

const InputArea: React.FC<InputAreaProps> = ({ onSend, disabled = false, placeholder = "Type your question..." }) => {
  const [inputValue, setInputValue] = useState<string>('');
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  const handleSend = () => {
    const trimmedValue = inputValue.trim();
    if (trimmedValue && !disabled) {
      onSend(trimmedValue);
      setInputValue('');
      if (textareaRef.current) {
        textareaRef.current.style.height = 'auto';
      }
    }
  };

  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const adjustTextareaHeight = () => {
    const textarea = textareaRef.current;
    if (textarea) {
      textarea.style.height = 'auto';
      textarea.style.height = `${Math.min(textarea.scrollHeight, 150)}px`;
    }
  };

  return (
    <div className="input-area" role="form" aria-label="Chat input area">
      <div className="input-area__container">
        <textarea
          ref={textareaRef}
          value={inputValue}
          onChange={(e) => {
            setInputValue(e.target.value);
            adjustTextareaHeight();
          }}
          onKeyDown={handleKeyDown}
          placeholder={placeholder}
          disabled={disabled}
          className={clsx('input-area__textarea', {
            'input-area__textarea--disabled': disabled,
          })}
          rows={1}
          aria-label="Type your message"
          aria-required="true"
          aria-invalid={disabled ? "true" : "false"}
          role="textbox"
          aria-multiline="true"
        />
        <button
          onClick={handleSend}
          disabled={disabled || !inputValue.trim()}
          className={clsx('input-area__send-button', {
            'input-area__send-button--disabled': disabled || !inputValue.trim(),
          })}
          aria-label="Send message"
          title="Send message"
        >
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">
            <path d="M22 2L11 13M22 2L15 22L11 13M11 13L2 9L22 2" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" />
          </svg>
        </button>
      </div>
    </div>
  );
};

export default InputArea;