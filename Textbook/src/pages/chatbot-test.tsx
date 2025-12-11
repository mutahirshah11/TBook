import React from 'react';
import Layout from '@theme/Layout';

export default function ChatbotTestPage() {
  return (
    <Layout title="Chatbot Test" description="Test page for the chatbot integration">
      <main style={{ padding: '2rem' }}>
        <div style={{ maxWidth: '800px', margin: '0 auto' }}>
          <h1>Chatbot Integration Test</h1>
          <p>
            This page tests the integration of the RAG chatbot. You should see a floating chat button
            in the bottom right corner of the screen. Click it to open the chat interface.
          </p>

          <h2>Try selecting text</h2>
          <p>
            Try selecting some text on this page. When you select text, you should see a small
            "Ask about selection" button appear, allowing you to ask questions about the selected content.
          </p>

          <div style={{
            padding: '1rem',
            backgroundColor: '#f0f0f0',
            borderRadius: '4px',
            margin: '1rem 0'
          }}>
            <p><strong>Example text to select:</strong> Embodied AI is a fascinating field that combines
            robotics, machine learning, and cognitive science to create intelligent agents that interact
            with the physical world through sensors and actuators.</p>
          </div>

          <h2>Features</h2>
          <ul>
            <li>Floating chat button that appears on all pages</li>
            <li>Real-time streaming responses</li>
            <li>Selected text context queries</li>
            <li>Session management</li>
            <li>Responsive design</li>
            <li>Light/dark mode support</li>
            <li>Full accessibility compliance</li>
          </ul>
        </div>
      </main>
    </Layout>
  );
}