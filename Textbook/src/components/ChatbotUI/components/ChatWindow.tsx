import React, { useState, useEffect, useRef, useCallback } from 'react';
import clsx from 'clsx';
import Message from './Message';
import InputArea from './InputArea';
import { Message as MessageType, ConversationSession } from '../types';
import { apiClient } from '../utils/apiClient';
import { sessionManager } from '../utils/sessionManager';
import { Bot, X, Trash2, Sparkles, MessageSquare } from 'lucide-react';

interface ChatWindowProps {
  isOpen: boolean;
  onClose: () => void;
  selectedText?: string;
}

const SUGGESTIONS = [
  "Explain ROS2 nodes",
  "What is URDF?",
  "How does Gazebo work?",
  "Inverse Kinematics basics"
];

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
        sessionManager.createSession();
        setSession(sessionManager.getSession());
      }
    }
  }, [isOpen]);

  useEffect(() => {
    scrollToBottom();
  }, [session?.messages, isLoading]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleClearChat = () => {
    if (window.confirm("Clear conversation history?")) {
        sessionManager.clearSession();
        sessionManager.createSession();
        setSession(sessionManager.getSession());
    }
  };

  const handleSendMessage = useCallback(async (message: string, contextText?: string) => {
    if (!session) return;

    setIsLoading(true);
    setError(null);

    try {
      const userMessageId = `msg_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
      const userMessage: MessageType = {
        id: userMessageId,
        content: message,
        sender: 'user',
        timestamp: new Date(),
        context: contextText || selectedText || null,
        status: 'sending',
      };

      sessionManager.addMessage(userMessage);
      setSession(sessionManager.getSession());

      const aiMessageId = `msg_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
      let aiMessage: MessageType = {
        id: aiMessageId,
        content: '',
        sender: 'ai',
        timestamp: new Date(),
        sourceReferences: [],
      };

      sessionManager.addMessage(aiMessage);
      setSession(sessionManager.getSession());

      await apiClient.streamMessage({
        message,
        sessionId: session.id,
        selectedText: contextText || selectedText,
        context: {
          currentPage: typeof window !== 'undefined' ? window.location.pathname : '',
        },
      }, (chunk, isFinal) => {
        aiMessage = {
          ...aiMessage,
          content: aiMessage.content + chunk,
          timestamp: new Date(),
        };
        sessionManager.updateMessage(aiMessageId, aiMessage);
        setSession(sessionManager.getSession());
      });

      sessionManager.updateMessage(userMessageId, { status: 'delivered' });
      setSession(sessionManager.getSession());
    } catch (err) {
      console.error('Error sending message:', err);
      setError(err instanceof Error ? err.message : 'Failed to send message');
      
      // Remove the empty AI message on error if it's still empty
      const currentSession = sessionManager.getSession();
      const lastMsg = currentSession?.messages[currentSession.messages.length - 1];
      if (lastMsg?.sender === 'ai' && !lastMsg.content) {
          // Ideally remove it, but for now we just leave it or show error in it
      }
    } finally {
      setIsLoading(false);
    }
  }, [session, selectedText]);

  if (!isOpen) return null;

  return (
    <div className="chat-window" role="dialog" aria-label="AI Assistant">
      <div className="chat-window__header">
        <div className="chat-window__title-container">
          <div className="chat-window__icon">
            <Bot size={20} />
          </div>
          <div>
            <h3 className="chat-window__title">AI Assistant</h3>
            <div style={{display:'flex', alignItems:'center', fontSize:'0.75rem', color:'#94a3b8'}}>
                <span className="chat-window__status-dot"></span>
                <span style={{marginLeft:'6px'}}>Online</span>
            </div>
          </div>
        </div>
        <div className="chat-window__actions">
            <button onClick={handleClearChat} className="chat-window__action-button" title="Clear Chat">
                <Trash2 size={18} />
            </button>
            <button onClick={onClose} className="chat-window__action-button" title="Close">
                <X size={20} />
            </button>
        </div>
      </div>

      <div ref={chatContainerRef} className="chat-window__messages-container">
        {session?.messages && session.messages.length > 0 ? (
          <>
            {session.messages.map((message) => (
              <Message key={message.id} message={message} />
            ))}
            {isLoading && (
               <div className="message message--ai">
                   <div className="message__container message__container--ai">
                       <div className="typing-indicator">
                           <div className="typing-dot"></div>
                           <div className="typing-dot"></div>
                           <div className="typing-dot"></div>
                       </div>
                   </div>
               </div>
            )}
          </>
        ) : (
          <div className="chat-window__empty-state">
            <div className="empty-state__icon">
                <Sparkles size={32} />
            </div>
            <h3>How can I help you today?</h3>
            <p style={{fontSize: '0.9rem', maxWidth: '280px'}}>
                I can explain robotics concepts, analyze code, or help you with ROS2.
            </p>
            {selectedText && (
               <div className="chat-window__selected-text-preview">
                  <strong>Selected Context:</strong> "{selectedText.substring(0, 60)}..."
               </div>
            )}
            <div className="suggestion-chips">
                {SUGGESTIONS.map(s => (
                    <button key={s} className="suggestion-chip" onClick={() => handleSendMessage(s)}>
                        {s}
                    </button>
                ))}
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {error && (
        <div className="chat-window__error">
          ⚠️ {error}
        </div>
      )}

      <div className="chat-window__input-container">
        <InputArea
          onSend={handleSendMessage}
          disabled={isLoading}
          placeholder={selectedText ? "Ask about selection..." : "Type your question..."}
        />
      </div>
    </div>
  );
};

export default ChatWindow;