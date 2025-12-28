import React, { useState, useEffect } from 'react';
import FloatingButton from './components/FloatingButton';
import ChatWindow from './components/ChatWindow';
import './ChatbotUI.css';
import { useAuth } from '../Auth/AuthProvider';
import { toast } from 'react-toastify';

const ChatbotProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const { session, profile } = useAuth();
  
  // Strict Auth Check for Chatbot Visibility
  // User requested to allow access if signed in, regardless of onboarding
  const isAuthorized = !!session;

  // Handle text selection on the page
  useEffect(() => {
    let timeout: NodeJS.Timeout;

    const handleSelection = () => {
      const selectedText = window.getSelection()?.toString().trim();
      if (selectedText) {
        setSelectedText(selectedText);
      } else {
        // Clear selected text when user deselects
        timeout = setTimeout(() => {
          if (!window.getSelection()?.toString().trim()) {
            setSelectedText(null);
          }
        }, 300); // Small delay to ensure selection is cleared
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {
      clearTimeout(timeout);
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, []);

  const toggleChat = () => {
    if (!isAuthorized) {
        toast.warning("Signin to Use the Ragchatbot", {
            position: "bottom-right",
            theme: "colored" // Yellow/Warning theme usually
        });
        return;
    }
    setIsChatOpen(!isChatOpen);
  };

  const closeChat = () => {
    setIsChatOpen(false);
  };

  // Function to handle "Ask about selected text" action
  const handleAskAboutSelection = () => {
    if (!isAuthorized) {
        toast.warning("Signin to Use the Ragchatbot", {
            position: "bottom-right",
            theme: "colored"
        });
        return;
    }

    if (selectedText) {
      setIsChatOpen(true);
      // The ChatWindow will receive the selectedText prop
    }
  };

  // Add a global click handler to close the chat when clicking outside
  useEffect(() => {
    if (!isChatOpen) return;

    const handleClickOutside = (event: MouseEvent) => {
      const chatWindow = document.querySelector('.chat-window');
      const floatingButton = document.querySelector('.floating-button');

      if (
        chatWindow &&
        !chatWindow.contains(event.target as Node) &&
        !floatingButton?.contains(event.target as Node)
      ) {
        closeChat();
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isChatOpen]);

  return (
    <>
      {children}
      {/* Always show the button, but behavior depends on auth */}
      <FloatingButton
        onClick={toggleChat}
        isOpen={isChatOpen}
      />
      
      {/* Only render ChatWindow if open (and implicitly authorized) */}
      <ChatWindow
        isOpen={isChatOpen}
        onClose={closeChat}
        selectedText={selectedText || undefined}
      />
      
      {selectedText && (
        <div
        style={{
            position: 'fixed',
            bottom: '80px',
            right: '20px',
            zIndex: 1000,
            backgroundColor: '#1a73e8',
            color: 'white',
            padding: '8px 12px',
            borderRadius: '4px',
            fontSize: '14px',
            cursor: 'pointer',
            display: 'flex',
            alignItems: 'center',
            gap: '6px'
        }}
        onClick={handleAskAboutSelection}
        aria-label="Ask about selected text"
        >
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" style={{flexShrink: 0}}>
            <path d="M8 12H16M12 8V16M21 12C21 16.0732 16.9706 20 12 20C7.02944 20 3 16.0732 3 12C3 7.9268 7.02944 4 12 4C16.9706 4 21 7.9268 21 12Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" />
        </svg>
        Ask about selection
        </div>
      )}
    </>
  );
};

export default ChatbotProvider;