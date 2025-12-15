// Core entities based on the data model

export interface Message {
  id: string;
  content: string;
  sender: 'user' | 'ai';
  timestamp: Date;
  context?: string | null;
  status?: 'sending' | 'delivered' | 'error';
  sourceReferences?: string[];
}

export interface ConversationSession {
  id: string;
  messages: Message[];
  createdAt: Date;
  lastActiveAt: Date;
  isActive: boolean;
}

export interface ChatConfig {
  theme: 'light' | 'dark' | 'auto';
  isExpanded: boolean;
  position: { x: number; y: number };
  sessionTimeout: number; // in minutes
}

// Additional utility types
export type SenderType = 'user' | 'ai';
export type MessageStatus = 'sending' | 'delivered' | 'error';
export type ThemeType = 'light' | 'dark' | 'auto';

// API request/response types
export interface SendMessageRequest {
  message: string;
  sessionId: string;
  selectedText?: string;
  context?: {
    currentPage?: string;
  };
}

export interface SendMessageResponse {
  response: string;
  sources: string[];
  sessionId: string;
  contextUsed?: string;
}

export interface CreateSessionRequest {
  initialContext?: object;
}

export interface CreateSessionResponse {
  sessionId: string;
}

export interface GetSessionRequest {
  sessionId: string;
}

export interface GetSessionResponse {
  sessionId: string;
  isActive: boolean;
  messageCount: number;
  lastActiveAt: string; // ISO string format
}

export interface APIErrorResponse {
  error: string;
  message: string;
}

// Context types
export interface ChatContextType {
  session: ConversationSession | null;
  config: ChatConfig;
  sendMessage: (message: string, selectedText?: string) => Promise<void>;
  createSession: () => Promise<string>;
  updateConfig: (newConfig: Partial<ChatConfig>) => void;
  clearSession: () => void;
}