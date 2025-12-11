import { ConversationSession, Message } from '../types';

const SESSION_STORAGE_KEY = 'chatbot-session';
const SESSION_TIMEOUT = 24 * 60 * 60 * 1000; // 24 hours in milliseconds

class SessionManager {
  private static instance: SessionManager;
  private currentSession: ConversationSession | null = null;

  private constructor() {
    this.loadSessionFromStorage();
  }

  public static getInstance(): SessionManager {
    if (!SessionManager.instance) {
      SessionManager.instance = new SessionManager();
    }
    return SessionManager.instance;
  }

  // Load session from browser storage
  private loadSessionFromStorage(): void {
    try {
      const sessionData = sessionStorage.getItem(SESSION_STORAGE_KEY);
      if (sessionData) {
        const parsed = JSON.parse(sessionData);
        // Validate session structure and check if it's still active
        if (this.isValidSession(parsed)) {
          // Convert string dates back to Date objects
          this.currentSession = {
            ...parsed,
            createdAt: new Date(parsed.createdAt),
            lastActiveAt: new Date(parsed.lastActiveAt),
            messages: parsed.messages.map((msg: any) => ({
              ...msg,
              timestamp: new Date(msg.timestamp),
            })),
          };
        } else {
          // If session is invalid, clear it from storage
          this.clearSession();
        }
      }
    } catch (error) {
      console.error('Error loading session from storage:', error);
      this.clearSession();
    }
  }

  // Save session to browser storage
  private saveSessionToStorage(): void {
    if (this.currentSession) {
      try {
        sessionStorage.setItem(SESSION_STORAGE_KEY, JSON.stringify(this.currentSession));
      } catch (error) {
        console.error('Error saving session to storage:', error);
      }
    }
  }

  // Validate session structure and check if it's still active
  private isValidSession(session: any): boolean {
    if (!session || !session.id || !session.createdAt || !session.messages) {
      return false;
    }

    // Check if session is older than 24 hours
    const sessionAge = Date.now() - new Date(session.createdAt).getTime();
    if (sessionAge > SESSION_TIMEOUT) {
      return false;
    }

    // Check if session has more than 100 messages (performance constraint)
    if (session.messages.length > 100) {
      return false;
    }

    return true;
  }

  // Create a new session
  public createSession(): ConversationSession {
    const sessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    this.currentSession = {
      id: sessionId,
      messages: [],
      createdAt: new Date(),
      lastActiveAt: new Date(),
      isActive: true,
    };

    this.saveSessionToStorage();
    return this.currentSession;
  }

  // Get the current session
  public getSession(): ConversationSession | null {
    return this.currentSession;
  }

  // Add a message to the current session
  public addMessage(message: Message): void {
    if (!this.currentSession) {
      this.createSession();
    }

    if (this.currentSession) {
      // Limit messages to 100 for performance (as per validation rules)
      if (this.currentSession.messages.length >= 100) {
        // Remove the oldest message if we're at the limit
        this.currentSession.messages.shift();
      }

      this.currentSession.messages.push(message);
      this.currentSession.lastActiveAt = new Date();
      this.saveSessionToStorage();
    }
  }

  // Update a message in the current session
  public updateMessage(messageId: string, updates: Partial<Message>): void {
    if (!this.currentSession) return;

    const messageIndex = this.currentSession.messages.findIndex(msg => msg.id === messageId);
    if (messageIndex !== -1) {
      this.currentSession.messages[messageIndex] = {
        ...this.currentSession.messages[messageIndex],
        ...updates,
      };
      this.currentSession.lastActiveAt = new Date();
      this.saveSessionToStorage();
    }
  }

  // Clear the current session
  public clearSession(): void {
    this.currentSession = null;
    try {
      sessionStorage.removeItem(SESSION_STORAGE_KEY);
    } catch (error) {
      console.error('Error clearing session from storage:', error);
    }
  }

  // Check if the session is still active
  public isSessionActive(): boolean {
    if (!this.currentSession) return false;

    const now = new Date();
    const lastActive = this.currentSession.lastActiveAt;
    const timeSinceLastActive = now.getTime() - lastActive.getTime();

    // Consider session inactive if no activity for 30 minutes (configurable)
    const INACTIVITY_TIMEOUT = 30 * 60 * 1000; // 30 minutes
    return timeSinceLastActive < INACTIVITY_TIMEOUT;
  }

  // Get all messages in the current session
  public getMessages(): Message[] {
    return this.currentSession?.messages || [];
  }
}

// Create a singleton instance
const sessionManager = SessionManager.getInstance();

export { SessionManager, sessionManager };
export default sessionManager;