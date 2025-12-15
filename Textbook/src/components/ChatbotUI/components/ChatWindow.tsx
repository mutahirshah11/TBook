import React, { useState, useEffect, useRef, useCallback } from 'react';
import clsx from 'clsx';
import Message from './Message';
import InputArea from './InputArea';
import { Message as MessageType, ConversationSession } from '../types';
import { apiClient } from '../utils/apiClient';
import { sessionManager } from '../utils/sessionManager';

interface ChatWindowProps {
  isOpen: boolean;
  onClose: () => void;
  selectedText?: string;
}

const ChatWindow: React.FC<ChatWindowProps> = ({ isOpen, onClose, selectedText }) => {
  const [session, setSession] = useState<ConversationSession | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const chatContainerRef = useRef<HTMLDivElement>(null);

  // Initialize session
  useEffect(() => {
    if (isOpen) {
      const existingSession = sessionManager.getSession();
      if (existingSession) {
        setSession(existingSession);
      } else {
        // Create a new session if none exists
        sessionManager.createSession();
        setSession(sessionManager.getSession());
      }
    }
  }, [isOpen]);

  // Scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [session?.messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSendMessage = useCallback(async (message: string, contextText?: string) => {
    if (!session) return;

    setIsLoading(true);
    setError(null);

    try {
      // Create user message object
      const userMessageId = `msg_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
      const userMessage: MessageType = {
        id: userMessageId,
        content: message,
        sender: 'user',
        timestamp: new Date(),
        context: contextText || selectedText || null,
        status: 'sending',
      };

      // Add the user message to the session immediately
      sessionManager.addMessage(userMessage);
      setSession(sessionManager.getSession());

      // Create a temporary AI message to hold the streaming response
      const aiMessageId = `msg_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
      let aiMessage: MessageType = {
        id: aiMessageId,
        content: '', // Start with empty content
        sender: 'ai',
        timestamp: new Date(),
        sourceReferences: [],
      };

      // Add the temporary AI message to the session
      sessionManager.addMessage(aiMessage);
      setSession(sessionManager.getSession());

      // Stream the response from the backend
      await apiClient.streamMessage({
        message,
        sessionId: session.id,
        selectedText: contextText || selectedText,
        context: {
          currentPage: typeof window !== 'undefined' ? window.location.pathname : '',
        },
      }, (chunk, isFinal) => {
        // Update the AI message content with the new chunk
        aiMessage = {
          ...aiMessage,
          content: aiMessage.content + chunk,
          timestamp: new Date(), // Update timestamp to now
        };

        // If it's the final chunk, we can also update source references if provided
        if (isFinal) {
          // In a real implementation, the final chunk might contain source references
          // For now, we'll just update the message as is
        }

        // Update the AI message in the session
        sessionManager.updateMessage(aiMessageId, aiMessage);
        setSession(sessionManager.getSession());
      });

      // Update the user message status to delivered
      sessionManager.updateMessage(userMessageId, { status: 'delivered' });
      setSession(sessionManager.getSession());
    } catch (err) {
      console.error('Error sending message:', err);
      setError(err instanceof Error ? err.message : 'Failed to send message');

      // Update the user message status to error
      if (session?.messages.length > 0) {
        const lastMessage = session.messages[session.messages.length - 1];
        if (lastMessage.sender === 'user' && lastMessage.status === 'sending') {
          sessionManager.updateMessage(lastMessage.id, { status: 'error' });
          setSession(sessionManager.getSession());
        }
      }
    } finally {
      setIsLoading(false);
    }
  }, [session, selectedText]);

  // Handle sending initial message with selected text if available
  useEffect(() => {
    if (selectedText && session) {
      // Optionally prompt the user to ask about the selected text
      // For now, we'll just store it for the next message
    }
  }, [selectedText, session]);

  if (!isOpen) return null;

  return (
    <div
      className="chat-window"
      role="dialog"
      aria-modal="true"
      aria-label="Chat with Book Assistant"
      aria-describedby="chat-window-description"
    >
      <div id="chat-window-description" className="sr-only">
        Interactive chat interface for asking questions about the book content
      </div>

      <div className="chat-window__header" role="banner">
        <h3 className="chat-window__title">Book Assistant</h3>
        <button
          onClick={onClose}
          className="chat-window__close-button"
          aria-label="Close chat"
          title="Close chat"
        >
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">
            <path d="M6 18L18 6M6 6L18 18" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" />
          </svg>
        </button>
      </div>

      <div
        ref={chatContainerRef}
        className="chat-window__messages-container"
        role="log"
        aria-live="polite"
        aria-label="Conversation history"
      >
        {session?.messages && session.messages.length > 0 ? (
          <div role="list" aria-label="Messages">
            {session.messages.map((message) => (
              <Message key={message.id} message={message} />
            ))}
          </div>
        ) : (
          <div className="chat-window__empty-state" role="status" aria-label="No messages yet">
            <p>Ask me anything about the book content!</p>
            {selectedText && (
              <p className="chat-window__selected-text-preview" aria-label={`Selected text: ${selectedText}`}>
                Selected: "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"
              </p>
            )}
          </div>
        )}
        <div ref={messagesEndRef} aria-hidden="true" />
      </div>

      {error && (
        <div
          className="chat-window__error"
          role="alert"
          aria-live="assertive"
          aria-label={`Error: ${error}`}
        >
          {error}
        </div>
      )}

      <div className="chat-window__input-container" role="complementary">
        <InputArea
          onSend={handleSendMessage}
          disabled={isLoading}
          placeholder={selectedText ? "Ask about the selected text..." : "Type your question..."}
        />
      </div>
    </div>
  );
};

export default ChatWindow;