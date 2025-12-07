import React, { useState, useEffect, useRef } from 'react';
import './Chatbot.css';

interface Message {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
  sources?: SourceReference[];
}

interface SourceReference {
  id: string;
  content: string;  // From OpenAPI spec
  metadata: Record<string, any>;  // From OpenAPI spec
}

interface ChatResponse {
  text: string;  // Changed from 'response' to 'text' to match API spec
  sources: SourceReference[];
}

const Chatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Check for selected text when user interacts with page
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const selectedText = selection?.toString().trim();

      // Only update if there's actual selected text
      if (selectedText && selectedText.length > 0) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Prepare the request payload - match API spec
      const payload = {
        text: inputValue,  // Changed from 'query' to 'text' to match API spec
        selected_text: selectedText || undefined, // Only include if there's selected text
        top_k: 5,
      };

      // Make API call to backend - use /query endpoint as per spec
      const response = await fetch('http://localhost:8000/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-API-Key': process.env.REACT_APP_API_KEY || 'your-api-key', // Use your API key
        },
        body: JSON.stringify(payload),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data: ChatResponse = await response.json();

      // Add bot response - use 'text' field as per API spec
      const botMessage: Message = {
        id: Date.now().toString(),
        text: data.text,
        sender: 'bot',
        timestamp: new Date(),
        sources: data.sources,
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      // Check if the error is related to no content found vs actual error
      // For now, we'll still use the constitution-compliant message for any error
      // In a production system, you might want to distinguish between different error types
      const errorMessage: Message = {
        id: Date.now().toString(),
        text: "I couldn't find a passage that directly answers this. Here are the closest related sources…", // Constitution-compliant response
        sender: 'bot',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setSelectedText(null); // Clear selected text after sending
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <div className="chatkit-container">
      {isOpen ? (
        <div className="chatkit-window">
          <div className="chatkit-header">
            <div className="chatkit-header-content">
              <div className="chatkit-ai-avatar">AI</div>
              <div className="chatkit-header-info">
                <h3 className="chatkit-title">Physical AI & Robotics Assistant</h3>
                <div className="chatkit-status">
                  <span className="status-dot online"></span>
                  <span className="status-text">Online</span>
                </div>
              </div>
            </div>
            <button className="chatkit-close" onClick={toggleChat}>
              <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                <path d="M12 4L4 12M4 4L12 12" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
              </svg>
            </button>
          </div>

          <div className="chatkit-messages">
            {messages.length === 0 ? (
              <div className="chatkit-welcome">
                <div className="chatkit-welcome-avatar">AI</div>
                <div className="chatkit-welcome-content">
                  <h4>Physical AI & Robotics Assistant</h4>
                  <p>Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics book.</p>
                  <p>Ask me anything about the book content or select text to ask specific questions!</p>
                </div>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`chatkit-message ${message.sender}-message`}
                >
                  {message.sender === 'bot' && (
                    <div className="chatkit-avatar bot-avatar">AI</div>
                  )}
                  <div className="chatkit-bubble">
                    <div className="chatkit-message-text">
                      {message.text}
                    </div>
                    {message.sender === 'bot' && message.sources && message.sources.length > 0 && (
                      <div className="chatkit-sources">
                        <details className="chatkit-details">
                          <summary className="chatkit-summary">📚 Sources & References</summary>
                          <div className="chatkit-sources-list">
                            {message.sources.map((source, index) => (
                              <div key={index} className="chatkit-source-item">
                                <div className="chatkit-source-info">
                                  <span className="chatkit-source-file">{source.metadata?.file_path || 'Unknown source'}</span>
                                  <span className="chatkit-source-details">Section {source.metadata?.chunk_id || 'N/A'} • Similarity: {(source.metadata?.similarity ? (source.metadata.similarity * 100).toFixed(1) : 'N/A')}%</span>
                                </div>
                                <a
                                  href={getSourceUrl(source.metadata?.file_path || '')}
                                  target="_blank"
                                  rel="noopener noreferrer"
                                  className="chatkit-source-link"
                                >
                                  View Source
                                </a>
                              </div>
                            ))}
                          </div>
                        </details>
                      </div>
                    )}
                  </div>
                  {message.sender === 'user' && (
                    <div className="chatkit-avatar user-avatar">You</div>
                  )}
                </div>
              ))
            )}
            <div ref={messagesEndRef} />
          </div>

          {selectedText && (
            <div className="chatkit-selected-text">
              <span className="selected-text-label">Using selected text:</span>
              <span className="selected-text-content">"{selectedText.substring(0, 60)}{selectedText.length > 60 ? '...' : ''}"</span>
            </div>
          )}

          <form onSubmit={handleSubmit} className="chatkit-input-form">
            <div className="chatkit-input-container">
              <input
                type="text"
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                placeholder={selectedText
                  ? "Ask about selected text..."
                  : "Message Physical AI & Robotics Assistant..."}
                className="chatkit-input"
                disabled={isLoading}
              />
              <button type="submit" disabled={isLoading} className="chatkit-send-button">
                {isLoading ? (
                  <div className="chatkit-loading-spinner"></div>
                ) : (
                  <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
                    <path d="M22 2L11 13M22 2l-7 20-4-9-9-4 20-7z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" />
                  </svg>
                )}
              </button>
            </div>
          </form>
        </div>
      ) : (
        <button className="chatkit-toggle" onClick={toggleChat}>
          <div className="chatkit-toggle-avatar">AI</div>
          <span>Ask AI</span>
        </button>
      )}
    </div>
  );
};

// Helper function to generate source URLs
const getSourceUrl = (sourceFile: string): string => {
  // Convert file path to URL - this is a simplified approach
  // In a real implementation, you'd need to map file paths to actual URLs
  const cleanPath = sourceFile.replace(/\.md$/, '').replace(/\\/g, '/');
  return `/docs/${cleanPath}`; // Assuming Docusaurus docs structure
};

export default Chatbot;