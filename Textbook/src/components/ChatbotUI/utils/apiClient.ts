import {
  SendMessageRequest,
  SendMessageResponse,
  CreateSessionRequest,
  CreateSessionResponse,
  GetSessionRequest,
  GetSessionResponse,
  APIErrorResponse
} from '../types';

// Configuration
const DEFAULT_TIMEOUT = 30000; // 30 seconds
const API_BASE_URL =
  (typeof process !== 'undefined' && process.env?.REACT_APP_API_BASE_URL) ||
  (typeof process !== 'undefined' && process.env?.API_BASE_URL) ||
  'http://localhost:8000';

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
    return this.request<SendMessageResponse>('/api/chat/send', {
      method: 'POST',
      body: JSON.stringify(data),
    });
  }

  // Stream response from the RAG system (for real-time responses)
  async streamMessage(data: SendMessageRequest, onChunk: (chunk: string, isFinal: boolean) => void): Promise<void> {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.timeout);

    try {
      const response = await fetch(`${this.baseUrl}/api/chat/send`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'text/plain',
          'X-Stream': 'true', // Indicate that we want a streaming response
        },
        body: JSON.stringify({
          ...data,
          stream: true, // Explicitly request streaming
        }),
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

          // Process the buffer looking for complete chunks
          let boundary = buffer.indexOf('\n');
          while (boundary !== -1) {
            const chunk = buffer.substring(0, boundary);
            buffer = buffer.substring(boundary + 1);

            if (chunk.trim()) {
              try {
                // Try to parse as JSON (Server-Sent Events format or JSON lines)
                const parsed = JSON.parse(chunk);
                if (typeof parsed === 'object' && parsed.content !== undefined) {
                  onChunk(parsed.content, parsed.isFinal || false);
                } else if (typeof parsed === 'string') {
                  onChunk(parsed, false);
                }
              } catch (e) {
                // If it's not JSON, treat as plain text
                onChunk(chunk, false);
              }
            }

            boundary = buffer.indexOf('\n');
          }
        }

        // Process any remaining data in buffer
        if (buffer.trim()) {
          try {
            const parsed = JSON.parse(buffer);
            if (typeof parsed === 'object' && parsed.content !== undefined) {
              onChunk(parsed.content, parsed.isFinal || true);
            } else if (typeof parsed === 'string') {
              onChunk(buffer, true);
            }
          } catch (e) {
            onChunk(buffer, true);
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
  async createSession(data?: CreateSessionRequest): Promise<CreateSessionResponse> {
    return this.request<CreateSessionResponse>('/api/chat/session', {
      method: 'POST',
      body: JSON.stringify(data || {}),
    });
  }

  // Get session details
  async getSession(data: GetSessionRequest): Promise<GetSessionResponse> {
    const params = new URLSearchParams({ sessionId: data.sessionId });
    return this.request<GetSessionResponse>(`/api/chat/session?${params}`, {
      method: 'GET',
    });
  }
}

// Create a singleton instance
const apiClient = new ApiClient();

export { ApiClient, apiClient };
export default apiClient;