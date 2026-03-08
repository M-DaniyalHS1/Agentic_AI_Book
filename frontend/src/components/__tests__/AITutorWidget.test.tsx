/**
 * Tests for AI Tutor Widget Component
 */
import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import AITutorWidget from '../AITutorWidget';

// Mock apiService
jest.mock('../apiService', () => ({
  __esModule: true,
  default: {
    chat: jest.fn(),
  },
}));

import apiService from '../apiService';

describe('AITutorWidget', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    (localStorage.getItem as jest.Mock).mockReturnValue(null);
  });

  it('should render FAB button initially', () => {
    render(<AITutorWidget />);

    const fabButton = screen.getByRole('button', { name: /ai tutor/i });
    expect(fabButton).toBeInTheDocument();
  });

  it('should open chat widget when FAB is clicked', () => {
    render(<AITutorWidget />);

    const fabButton = screen.getByRole('button', { name: /ai tutor/i });
    fireEvent.click(fabButton);

    expect(screen.getByText(/ask me anything about physical ai/i)).toBeInTheDocument();
    expect(screen.getByPlaceholderText(/ask a question about the textbook/i)).toBeInTheDocument();
  });

  it('should close chat widget when FAB is clicked again', () => {
    render(<AITutorWidget />);

    const fabButton = screen.getByRole('button', { name: /ai tutor/i });
    fireEvent.click(fabButton);
    fireEvent.click(fabButton);

    expect(screen.queryByPlaceholderText(/ask a question/i)).not.toBeInTheDocument();
  });

  it('should display welcome suggestions', () => {
    render(<AITutorWidget />);

    const fabButton = screen.getByRole('button', { name: /ai tutor/i });
    fireEvent.click(fabButton);

    expect(screen.getByText('What is ROS 2?')).toBeInTheDocument();
    expect(screen.getByText('Explain digital twins')).toBeInTheDocument();
    expect(screen.getByText('How does VSLAM work?')).toBeInTheDocument();
  });

  it('should send message when clicking suggestion', async () => {
    const mockResponse = {
      answer: 'ROS 2 is a robotics middleware.',
      citations: [],
      confidence: 0.8,
      fallback: false,
      session_id: 'test-session',
      response_time_ms: 100,
    };

    (apiService.chat as jest.Mock).mockResolvedValueOnce(mockResponse);

    render(<AITutorWidget />);

    const fabButton = screen.getByRole('button', { name: /ai tutor/i });
    fireEvent.click(fabButton);

    const suggestion = screen.getByText('What is ROS 2?');
    fireEvent.click(suggestion);

    await waitFor(() => {
      expect(screen.getByText('ROS 2 is a robotics middleware.')).toBeInTheDocument();
    });

    expect(apiService.chat).toHaveBeenCalledWith(
      expect.objectContaining({
        query: 'What is ROS 2?',
      })
    );
  });

  it('should send message when typing and pressing send', async () => {
    const mockResponse = {
      answer: 'This is a test response.',
      citations: [],
      confidence: 0.75,
      fallback: false,
      session_id: 'test-session',
      response_time_ms: 150,
    };

    (apiService.chat as jest.Mock).mockResolvedValueOnce(mockResponse);

    render(<AITutorWidget />);

    const fabButton = screen.getByRole('button', { name: /ai tutor/i });
    fireEvent.click(fabButton);

    const textarea = screen.getByPlaceholderText(/ask a question/i);
    fireEvent.change(textarea, { target: { value: 'Test question' } });

    const sendButton = screen.getByRole('button', { name: /send/i });
    fireEvent.click(sendButton);

    await waitFor(() => {
      expect(screen.getByText('Test question')).toBeInTheDocument();
    });

    expect(apiService.chat).toHaveBeenCalled();
  });

  it('should send message on Enter key press', async () => {
    const mockResponse = {
      answer: 'Response to enter key test.',
      citations: [],
      confidence: 0.8,
      fallback: false,
      session_id: 'test-session',
      response_time_ms: 100,
    };

    (apiService.chat as jest.Mock).mockResolvedValueOnce(mockResponse);

    render(<AITutorWidget />);

    const fabButton = screen.getByRole('button', { name: /ai tutor/i });
    fireEvent.click(fabButton);

    const textarea = screen.getByPlaceholderText(/ask a question/i);
    fireEvent.change(textarea, { target: { value: 'Test with Enter' } });
    fireEvent.keyDown(textarea, { key: 'Enter', code: 'Enter' });

    await waitFor(() => {
      expect(screen.getByText('Test with Enter')).toBeInTheDocument();
    });
  });

  it('should show loading state while waiting for response', async () => {
    (apiService.chat as jest.Mock).mockImplementation(
      () => new Promise((resolve) => setTimeout(resolve, 100))
    );

    render(<AITutorWidget />);

    const fabButton = screen.getByRole('button', { name: /ai tutor/i });
    fireEvent.click(fabButton);

    const textarea = screen.getByPlaceholderText(/ask a question/i);
    fireEvent.change(textarea, { target: { value: 'Loading test' } });

    const sendButton = screen.getByRole('button', { name: /send/i });
    fireEvent.click(sendButton);

    // Check for loading dots
    await waitFor(() => {
      const loadingDots = document.querySelectorAll('.ai-tutor-loading-dot');
      expect(loadingDots.length).toBeGreaterThan(0);
    });
  });

  it('should display error message on API failure', async () => {
    (apiService.chat as jest.Mock).mockRejectedValueOnce(new Error('API Error'));

    render(<AITutorWidget />);

    const fabButton = screen.getByRole('button', { name: /ai tutor/i });
    fireEvent.click(fabButton);

    const textarea = screen.getByPlaceholderText(/ask a question/i);
    fireEvent.change(textarea, { target: { value: 'Error test' } });

    const sendButton = screen.getByRole('button', { name: /send/i });
    fireEvent.click(sendButton);

    await waitFor(() => {
      expect(screen.getByText(/error processing your request/i)).toBeInTheDocument();
    });
  });

  it('should clear chat when clear button is clicked', async () => {
    const mockResponse = {
      answer: 'Test response',
      citations: [],
      confidence: 0.8,
      fallback: false,
      session_id: 'test-session',
      response_time_ms: 100,
    };

    (apiService.chat as jest.Mock).mockResolvedValueOnce(mockResponse);

    render(<AITutorWidget />);

    // Open and send message
    const fabButton = screen.getByRole('button', { name: /ai tutor/i });
    fireEvent.click(fabButton);

    const suggestion = screen.getByText('What is ROS 2?');
    fireEvent.click(suggestion);

    await waitFor(() => {
      expect(screen.getByText('Test response')).toBeInTheDocument();
    });

    // Clear chat
    const clearButton = screen.getByTitle('Clear chat');
    fireEvent.click(clearButton);

    expect(screen.queryByText('Test response')).not.toBeInTheDocument();
    expect(screen.getByText(/ask me anything/i)).toBeInTheDocument();
  });

  it('should save session ID to localStorage', async () => {
    const mockResponse = {
      answer: 'Test',
      citations: [],
      confidence: 0.8,
      fallback: false,
      session_id: 'new-session-123',
      response_time_ms: 100,
    };

    (apiService.chat as jest.Mock).mockResolvedValueOnce(mockResponse);

    render(<AITutorWidget />);

    const fabButton = screen.getByRole('button', { name: /ai tutor/i });
    fireEvent.click(fabButton);

    const textarea = screen.getByPlaceholderText(/ask a question/i);
    fireEvent.change(textarea, { target: { value: 'Session test' } });

    const sendButton = screen.getByRole('button', { name: /send/i });
    fireEvent.click(sendButton);

    await waitFor(() => {
      expect(localStorage.setItem).toHaveBeenCalledWith(
        'ai_tutor_session_id',
        'new-session-123'
      );
    });
  });

  it('should load existing session from localStorage', () => {
    (localStorage.getItem as jest.Mock).mockReturnValue('existing-session-456');

    render(<AITutorWidget />);

    expect(localStorage.getItem).toHaveBeenCalledWith('ai_tutor_session_id');
  });
});
