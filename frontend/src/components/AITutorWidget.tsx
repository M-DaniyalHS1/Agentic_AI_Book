import React, { useState, useRef, useEffect } from 'react';
import apiService, { ChatRequest, ChatResponse, Citation } from '../services/apiService';
import './AITutorWidget.css';

interface Message {
  id: string;
  type: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  confidence?: number;
  timestamp: Date;
}

const AITutorWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Load session from localStorage
  useEffect(() => {
    const savedSession = localStorage.getItem('ai_tutor_session_id');
    if (savedSession) {
      setSessionId(savedSession);
    }
  }, []);

  // Auto-scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Handle text selection in the page
  useEffect(() => {
    const handleTextSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 10 && isOpen) {
        setSelectedText(text);
      } else if (!text) {
        setSelectedText(null);
      }
    };

    document.addEventListener('selectionchange', handleTextSelection);
    return () => document.removeEventListener('selectionchange', handleTextSelection);
  }, [isOpen]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSendMessage = async (query?: string) => {
    const messageQuery = query || inputValue;
    if (!messageQuery.trim()) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      type: 'user',
      content: messageQuery,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const request: ChatRequest = {
        query: messageQuery,
        session_id: sessionId || undefined,
      };

      if (selectedText) {
        request.selected_text = selectedText;
        setSelectedText(null);
      }

      const response = await apiService.chat(request);

      // Save session ID
      if (response.session_id && !sessionId) {
        setSessionId(response.session_id);
        localStorage.setItem('ai_tutor_session_id', response.session_id);
      }

      // Add assistant message
      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        type: 'assistant',
        content: response.answer,
        citations: response.citations,
        confidence: response.confidence,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Chat error:', error);

      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        type: 'assistant',
        content:
          "I'm sorry, I encountered an error processing your request. Please make sure the backend server is running and try again.",
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleExplainSelected = () => {
    if (selectedText) {
      handleSendMessage(`Explain this: ${selectedText}`);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const clearChat = () => {
    setMessages([]);
    setSessionId(null);
    localStorage.removeItem('ai_tutor_session_id');
  };

  const formatCitationUrl = (citation: Citation) => {
    return `/docs/${citation.module_slug}/${citation.chapter_slug}`;
  };

  return (
    <>
      {/* Floating Action Button */}
      <button className="ai-tutor-fab" onClick={() => setIsOpen(!isOpen)} aria-label="AI Tutor">
        {isOpen ? (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <line x1="18" y1="6" x2="6" y2="18" />
            <line x1="6" y1="6" x2="18" y2="18" />
          </svg>
        ) : (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
        )}
      </button>

      {/* Selected Text Popup */}
      {selectedText && isOpen && (
        <div className="ai-tutor-selection-popup">
          <button onClick={handleExplainSelected} className="ai-tutor-explain-btn">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <circle cx="12" cy="12" r="10" />
              <path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3" />
              <line x1="12" y1="17" x2="12.01" y2="17" />
            </svg>
            Explain this
          </button>
        </div>
      )}

      {/* Chat Widget */}
      {isOpen && (
        <div className="ai-tutor-widget">
          {/* Header */}
          <div className="ai-tutor-header">
            <div className="ai-tutor-header-title">
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M12 2a2 2 0 0 1 2 2c0 .74-.4 1.39-1 1.73V7h1a7 7 0 0 1 7 7h1a1 1 0 0 1 1 1v3a1 1 0 0 1-1 1h-1v1a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-1H2a1 1 0 0 1-1-1v-3a1 1 0 0 1 1-1h1v-1a7 7 0 0 1 7-7h1V5.73c-.6-.34-1-.99-1-1.73a2 2 0 0 1 2-2z" />
              </svg>
              <span>AI Tutor</span>
            </div>
            <div className="ai-tutor-header-actions">
              <button onClick={clearChat} className="ai-tutor-clear-btn" title="Clear chat">
                <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <polyline points="3 6 5 6 21 6" />
                  <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2" />
                </svg>
              </button>
            </div>
          </div>

          {/* Messages */}
          <div className="ai-tutor-messages">
            {messages.length === 0 ? (
              <div className="ai-tutor-welcome">
                <div className="ai-tutor-welcome-icon">
                  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
                    <path d="M12 2a2 2 0 0 1 2 2c0 .74-.4 1.39-1 1.73V7h1a7 7 0 0 1 7 7h1a1 1 0 0 1 1 1v3a1 1 0 0 1-1 1h-1v1a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-1H2a1 1 0 0 1-1-1v-3a1 1 0 0 1 1-1h1v-1a7 7 0 0 1 7-7h1V5.73c-.6-.34-1-.99-1-1.73a2 2 0 0 1 2-2z" />
                  </svg>
                </div>
                <h3>Ask me anything about Physical AI!</h3>
                <p>I can help you understand concepts from the textbook.</p>
                <ul className="ai-tutor-suggestions">
                  <li onClick={() => handleSendMessage('What is ROS 2?')}>What is ROS 2?</li>
                  <li onClick={() => handleSendMessage('Explain digital twins')}>Explain digital twins</li>
                  <li onClick={() => handleSendMessage('How does VSLAM work?')}>How does VSLAM work?</li>
                </ul>
              </div>
            ) : (
              messages.map((message) => (
                <div key={message.id} className={`ai-tutor-message ${message.type}`}>
                  <div className="ai-tutor-message-content">
                    {message.content.split('\n').map((line, i) => (
                      <p key={i}>{line}</p>
                    ))}
                  </div>

                  {message.citations && message.citations.length > 0 && (
                    <div className="ai-tutor-citations">
                      <div className="ai-tutor-citations-title">
                        📚 Sources cited ({message.citations.length})
                      </div>
                      {message.citations.map((citation, idx) => (
                        <a
                          key={idx}
                          href={formatCitationUrl(citation)}
                          className="ai-tutor-citation"
                          target="_blank"
                          rel="noopener noreferrer"
                        >
                          <div className="ai-tutor-citation-source">
                            {citation.module_slug} → {citation.chapter_slug} → {citation.section_title}
                          </div>
                          <div className="ai-tutor-citation-snippet">{citation.content_snippet}</div>
                          <div className="ai-tutor-citation-score">
                            {(citation.similarity_score * 100).toFixed(0)}% match
                          </div>
                        </a>
                      ))}
                    </div>
                  )}

                  <div className="ai-tutor-message-meta">
                    {message.confidence && (
                      <span className="ai-tutor-confidence">
                        {(message.confidence * 100).toFixed(0)}% confidence
                      </span>
                    )}
                    <span className="ai-tutor-time">
                      {message.timestamp.toLocaleTimeString([], {
                        hour: '2-digit',
                        minute: '2-digit',
                      })}
                    </span>
                  </div>
                </div>
              ))
            )}

            {isLoading && (
              <div className="ai-tutor-message assistant">
                <div className="ai-tutor-loading">
                  <div className="ai-tutor-loading-dot"></div>
                  <div className="ai-tutor-loading-dot"></div>
                  <div className="ai-tutor-loading-dot"></div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className="ai-tutor-input-area">
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question about the textbook..."
              rows={2}
              maxLength={2000}
              disabled={isLoading}
            />
            <button
              onClick={() => handleSendMessage()}
              disabled={isLoading || !inputValue.trim()}
              className="ai-tutor-send-btn"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <line x1="22" y1="2" x2="11" y2="13" />
                <polygon points="22 2 15 22 11 13 2 9 22 2" />
              </svg>
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default AITutorWidget;
