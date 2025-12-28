import React from 'react';
import clsx from 'clsx';
import { Message as MessageType } from '../types';
import ReactMarkdown from 'react-markdown';
import { User, Bot } from 'lucide-react';

interface MessageProps {
  message: MessageType;
}

const Message: React.FC<MessageProps> = ({ message }) => {
  const isUser = message.sender === 'user';

  return (
    <div className={clsx('message', `message--${message.sender}`)}>
      <div className={clsx('message__container', {
        'message__container--user': isUser,
        'message__container--ai': !isUser,
      })}>
        <div className="message__content">
            {isUser ? (
                // User messages are usually short, just text
                <p style={{margin:0}}>{message.content}</p>
            ) : (
                // AI messages need markdown
                <ReactMarkdown>{message.content}</ReactMarkdown>
            )}
        </div>
        
        <div className="message__meta">
            {/* 
               We could show timestamps here, but often clean is better. 
               Only show status for user messages if needed (e.g. error) 
            */}
            {isUser && message.status === 'error' && (
                <span style={{color: '#f87171', fontWeight: 'bold'}}>Failed to send</span>
            )}
        </div>
      </div>
    </div>
  );
};

export default Message;