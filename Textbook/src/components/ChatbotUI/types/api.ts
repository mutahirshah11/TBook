// API-specific types that complement the general types in index.ts

// Request/Response types for the chat API
export interface ChatAPIRequest {
  endpoint: string;
  method: 'GET' | 'POST' | 'PUT' | 'DELETE';
  headers?: Record<string, string>;
  body?: any;
}

export interface ChatAPIResponse<T = any> {
  data: T;
  status: number;
  headers: Headers;
}

// Specific API endpoint request/response types
export interface SendMessageAPIRequest {
  message: string;
  sessionId: string;
  selectedText?: string;
  context?: {
    currentPage?: string;
  };
}

export interface SendMessageAPIResponse {
  response: string;
  sources: string[];
  sessionId: string;
  contextUsed?: string;
}

export interface CreateSessionAPIRequest {
  initialContext?: object;
}

export interface CreateSessionAPIResponse {
  sessionId: string;
}

export interface GetSessionAPIRequest {
  sessionId: string;
}

export interface GetSessionAPIResponse {
  sessionId: string;
  isActive: boolean;
  messageCount: number;
  lastActiveAt: string; // ISO string format
}

// Error response type
export interface APIErrorResponse {
  error: string;
  message: string;
  statusCode?: number;
}

// Configuration type for API client
export interface APIConfig {
  baseUrl: string;
  timeout: number;
  defaultHeaders?: Record<string, string>;
  retryAttempts?: number;
  retryDelay?: number;
}

// Streaming response types (for real-time responses)
export interface StreamingMessageChunk {
  content: string;
  isFinal: boolean;
  sources?: string[];
}

// Rate limiting response
export interface RateLimitResponse {
  error: 'RATE_LIMIT_EXCEEDED';
  message: string;
  retryAfter?: number; // seconds to wait before retrying
}

// Backend service status
export interface BackendStatus {
  isHealthy: boolean;
  responseTime: number; // in milliseconds
  lastChecked: Date;
}