import React from 'react';
import ChatbotProvider from '../components/ChatbotUI/ChatbotProvider';

// Default theme Root component wrapper
export default function Root({ children }: { children: React.ReactNode }) {
  return <ChatbotProvider>{children}</ChatbotProvider>;
}