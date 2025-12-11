import React from 'react';
import clsx from 'clsx';
import { Message as MessageType } from '../types';

interface MessageProps {
  message: MessageType;
}

const Message: React.FC<MessageProps> = ({ message }) => {
  const isUser = message.sender === 'user';

  return (
    <div
      className={clsx('message', `message--${message.sender}`, {
        'message--sending': message.status === 'sending',
        'message--error': message.status === 'error',
      })}
      role="listitem"
      aria-label={`${message.sender} message: ${message.content.substring(0, 50)}${message.content.length > 50 ? '...' : ''}`}
    >
      <div
        className={clsx('message__container', {
          'message__container--user': isUser,
          'message__container--ai': !isUser,
        })}
        tabIndex={0}
      >
        <div
          className={clsx('message__content', {
            'message__content--user': isUser,
            'message__content--ai': !isUser,
          })}
          role="paragraph"
        >
          {message.content}
        </div>
        <div className="message__meta" aria-label={`Sent at ${message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}`}>
          <time
            className="message__timestamp"
            dateTime={message.timestamp.toISOString()}
          >
            {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
          </time>
          {message.status && (
            <span
              className={`message__status message__status--${message.status}`}
              aria-label={`Status: ${message.status}`}
            >
              {message.status}
            </span>
          )}
        </div>
      </div>
    </div>
  );
};

export default Message;