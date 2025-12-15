/**
 * Utility functions for handling text selection on the page
 */
class TextSelectionUtil {
  /**
   * Get the currently selected text on the page
   * @returns The selected text or null if no text is selected
   */
  static getSelectedText(): string | null {
    const selection = window.getSelection();
    if (!selection) return null;

    const selectedText = selection.toString().trim();
    return selectedText || null;
  }

  /**
   * Get the DOM range of the current selection
   * @returns The selected range or null if no selection exists
   */
  static getSelectedRange(): Range | null {
    const selection = window.getSelection();
    if (!selection || selection.rangeCount === 0) return null;

    return selection.getRangeAt(0);
  }

  /**
   * Check if there is any text currently selected on the page
   * @returns True if text is selected, false otherwise
   */
  static hasSelection(): boolean {
    const selectedText = this.getSelectedText();
    return selectedText !== null && selectedText.length > 0;
  }

  /**
   * Get the bounding rectangle of the current text selection
   * @returns The bounding rectangle or null if no selection exists
   */
  static getSelectionRect(): DOMRect | null {
    const range = this.getSelectedRange();
    if (!range) return null;

    return range.getBoundingClientRect();
  }

  /**
   * Clear the current text selection
   */
  static clearSelection(): void {
    const selection = window.getSelection();
    if (selection) {
      selection.removeAllRanges();
    }
  }

  /**
   * Get the context around the selected text (e.g., the paragraph or container)
   * @returns Context information about the selected text
   */
  static getSelectionContext(): {
    element: HTMLElement | null;
    textContent: string;
    containerType: string;
  } | null {
    const range = this.getSelectedRange();
    if (!range) return null;

    const commonAncestor = range.commonAncestorContainer as HTMLElement;
    let element: HTMLElement | null = commonAncestor;

    // If the common ancestor is a text node, get its parent element
    if (element.nodeType === Node.TEXT_NODE) {
      element = element.parentElement;
    }

    // Find the most relevant container (e.g., paragraph, div, etc.)
    while (element && element.parentElement) {
      const tagName = element.tagName.toLowerCase();
      if (['p', 'div', 'section', 'article', 'li', 'td', 'th'].includes(tagName)) {
        break;
      }
      element = element.parentElement;
    }

    if (!element) return null;

    return {
      element,
      textContent: element.textContent || '',
      containerType: element.tagName.toLowerCase(),
    };
  }

  /**
   * Create a function that can be attached to an event to capture selected text
   * @param callback - Function to call with the selected text
   * @returns Event handler function
   */
  static createSelectionHandler(
    callback: (selectedText: string, context?: any) => void
  ): (event: Event) => void {
    return (event: Event) => {
      // Wait for the selection to be completed (after mouse up or key up)
      setTimeout(() => {
        const selectedText = this.getSelectedText();
        if (selectedText) {
          const context = this.getSelectionContext();
          callback(selectedText, context);
        }
      }, 0);
    };
  }
}

// Export default instance and individual functions
export default TextSelectionUtil;

// Also export individual functions for convenience
export const {
  getSelectedText,
  getSelectedRange,
  hasSelection,
  getSelectionRect,
  clearSelection,
  getSelectionContext,
  createSelectionHandler
} = TextSelectionUtil;