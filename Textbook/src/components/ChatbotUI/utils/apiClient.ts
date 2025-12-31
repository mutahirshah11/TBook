import {
  SendMessageRequest,
  SendMessageResponse,
  CreateSessionRequest,
  CreateSessionResponse,
  GetSessionRequest,
  GetSessionResponse,
  APIErrorResponse
} from '../types';
import { config } from '../../../config';

// Configuration
const DEFAULT_TIMEOUT = 30000; // 30 seconds
const API_BASE_URL =
  (typeof process !== 'undefined' && process.env?.REACT_APP_API_BASE_URL) ||
  (typeof process !== 'undefined' && process.env?.API_BASE_URL) ||
  config.backendUrl;

// API Client class for handling communication with the backend
class ApiClient {
  private baseUrl: string;
  private timeout: number;

  constructor(baseUrl?: string, timeout?: number) {
    this.baseUrl = baseUrl || API_BASE_URL;
    this.timeout = timeout || DEFAULT_TIMEOUT;
  }

  // Helper method to create request options with timeout
  private async request<T>(endpoint: string, options: RequestInit = {}): Promise<T> {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.timeout);

    try {
      const response = await fetch(`${this.baseUrl}${endpoint}`, {
        ...options,
        signal: controller.signal,
        credentials: 'include', // Important: Send Better Auth session cookie
        headers: {
          'Content-Type': 'application/json',
          ...options.headers,
        },
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        const errorData: APIErrorResponse = await response.json().catch(() => ({
          error: 'UNKNOWN_ERROR',
          message: `HTTP Error: ${response.status} ${response.statusText}`,
        }));
        throw new Error(errorData.message || `HTTP Error: ${response.status} ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      clearTimeout(timeoutId);

      if (error instanceof TypeError && error.message.includes('fetch')) {
        throw new Error('Network error: Unable to connect to the server');
      }

      if (error.name === 'AbortError') {
        throw new Error('Request timeout: The server took too long to respond');
      }

      throw error;
    }
  }

  // Send a message to the RAG system
  async sendMessage(data: SendMessageRequest): Promise<SendMessageResponse> {
    // Transform the frontend request format to match backend expectations
    const backendPayload = {
      query_text: data.message,
      selected_text: data.selectedText || null,
      context_mode: data.selectedText ? 'selected-text' : 'full-book', // Use selected-text mode if text is provided
    };

    return this.request<SendMessageResponse>('/api/v1/chat', {
      method: 'POST',
      body: JSON.stringify(backendPayload),
      // No manual Authorization header needed, we use the session cookie
    });
  }

  // Stream response from the RAG system (for real-time responses)
  async streamMessage(data: SendMessageRequest, onChunk: (chunk: string, isFinal: boolean) => void): Promise<void> {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.timeout);

    try {
      // Transform the frontend request format to match backend expectations
      const backendPayload = {
        query_text: data.message,
        selected_text: data.selectedText || null,
        context_mode: data.selectedText ? 'selected-text' : 'full-book', // Use selected-text mode if text is provided
      };

      const response = await fetch(`${this.baseUrl}/api/v1/chat/stream`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'text/plain',
          'X-Stream': 'true', // Indicate that we want a streaming response
        },
        credentials: 'include', // Important: Send Better Auth session cookie
        body: JSON.stringify(backendPayload),
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        const errorData: APIErrorResponse = await response.json().catch(() => ({
          error: 'UNKNOWN_ERROR',
          message: `HTTP Error: ${response.status} ${response.statusText}`,
        }));
        throw new Error(errorData.message || `HTTP Error: ${response.status} ${response.statusText}`);
      }

      if (!response.body) {
        throw new Error('Response body is empty');
      }

      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      let buffer = '';

      try {
        while (true) {
          const { done, value } = await reader.read();
          if (done) break;

          buffer += decoder.decode(value, { stream: true });

          // Process the buffer looking for complete lines (Server-Sent Events format)
          let boundary = buffer.indexOf('\n');
          while (boundary !== -1) {
            const line = buffer.substring(0, boundary).trim();
            buffer = buffer.substring(boundary + 1);

            if (line.startsWith('data: ')) {
              // This is a Server-Sent Event data line
              const dataContent = line.substring(6); // Remove "data: " prefix
              try {
                const parsed = JSON.parse(dataContent);
                if (typeof parsed === 'object' && parsed.content !== undefined) {
                  if (parsed.status === 'error') {
                    onChunk(`Error: ${parsed.content}`, true);
                  } else {
                    onChunk(parsed.content, parsed.status === 'complete');
                  }
                } else {
                  onChunk(dataContent, false);
                }
              } catch (e) {
                // If it's not JSON, treat as plain text
                onChunk(dataContent, false);
              }
            } else if (line) {
              // Handle non-SSE format for compatibility
              try {
                const parsed = JSON.parse(line);
                if (typeof parsed === 'object' && parsed.content !== undefined) {
                  onChunk(parsed.content, parsed.isFinal || false);
                } else if (typeof parsed === 'string') {
                  onChunk(parsed, false);
                }
              } catch (e) {
                // If it's not JSON, treat as plain text
                onChunk(line, false);
              }
            }

            boundary = buffer.indexOf('\n');
          }
        }

        // Process any remaining data in buffer
        if (buffer.trim()) {
          // Handle remaining data - could be partial SSE line or JSON
          const trimmedBuffer = buffer.trim();
          if (trimmedBuffer.startsWith('data: ')) {
            const dataContent = trimmedBuffer.substring(6);
            try {
              const parsed = JSON.parse(dataContent);
              if (typeof parsed === 'object' && parsed.content !== undefined) {
                onChunk(parsed.content, true);
              } else {
                onChunk(dataContent, true);
              }
            } catch (e) {
              onChunk(dataContent, true);
            }
          } else {
            try {
              const parsed = JSON.parse(trimmedBuffer);
              if (typeof parsed === 'object' && parsed.content !== undefined) {
                onChunk(parsed.content, true);
              } else if (typeof parsed === 'string') {
                onChunk(trimmedBuffer, true);
              }
            } catch (e) {
              onChunk(trimmedBuffer, true);
            }
          }
        }
      } finally {
        reader.releaseLock();
      }
    } catch (error) {
      clearTimeout(timeoutId);

      if (error instanceof TypeError && error.message.includes('fetch')) {
        throw new Error('Network error: Unable to connect to the server');
      }

      if (error.name === 'AbortError') {
        throw new Error('Request timeout: The server took too long to respond');
      }

      throw error;
    }
  }

  // Create a new chat session
  // Note: Session management is handled client-side in the browser
  // This method is a placeholder to match the interface but doesn't make a server call
  async createSession(data?: CreateSessionRequest): Promise<CreateSessionResponse> {
    // Client-side session management, no server call needed
    return {
      sessionId: `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
    };
  }

  // Get session details
  // Note: Session management is handled client-side in the browser
  // This method is a placeholder to match the interface but doesn't make a server call
  async getSession(data: GetSessionRequest): Promise<GetSessionResponse> {
    // Client-side session management, no server call needed
    return {
      sessionId: data.sessionId,
      isActive: true,
      messageCount: 0, // This would need to be tracked client-side
      lastActiveAt: new Date().toISOString(),
    };
  }
}

// Create a singleton instance
const apiClient = new ApiClient();

export { ApiClient, apiClient };
export default apiClient;