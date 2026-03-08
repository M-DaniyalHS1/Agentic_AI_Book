/**
 * Tests for API Service
 */
import apiService, { ChatRequest, ChatResponse } from '../apiService';

describe('ApiService', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  describe('chat', () => {
    it('should send chat request and return response', async () => {
      const mockResponse: ChatResponse = {
        answer: 'ROS 2 is a robotics middleware framework.',
        citations: [
          {
            module_slug: 'module-1',
            chapter_slug: 'ros2-architecture',
            section_slug: 'introduction',
            section_title: 'Introduction to ROS 2',
            content_snippet: 'ROS 2 is a flexible framework...',
            similarity_score: 0.85,
          },
        ],
        confidence: 0.85,
        fallback: false,
        session_id: 'test-session-123',
        response_time_ms: 150,
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => mockResponse,
      });

      const request: ChatRequest = {
        query: 'What is ROS 2?',
        session_id: 'test-session-123',
      };

      const response = await apiService.chat(request);

      expect(global.fetch).toHaveBeenCalledWith(
        'http://localhost:8000/api/tutor/chat',
        expect.objectContaining({
          method: 'POST',
          headers: expect.objectContaining({
            'Content-Type': 'application/json',
          }),
          body: JSON.stringify(request),
        })
      );

      expect(response).toEqual(mockResponse);
    });

    it('should handle chat with selected text', async () => {
      const mockResponse: ChatResponse = {
        answer: 'This explains ROS 2 nodes.',
        citations: [],
        confidence: 0.9,
        fallback: false,
        session_id: 'test-session',
        response_time_ms: 100,
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => mockResponse,
      });

      const request: ChatRequest = {
        query: 'Explain this',
        selected_text: 'ROS 2 nodes communicate through topics',
      };

      await apiService.chat(request);

      expect(global.fetch).toHaveBeenCalledWith(
        expect.any(String),
        expect.objectContaining({
          body: JSON.stringify(request),
        })
      );
    });

    it('should throw error on failed request', async () => {
      (global.fetch as jest.Mock).mockResolvedValueOnce({
        ok: false,
        json: async () => ({ detail: 'Internal server error' }),
      });

      const request: ChatRequest = {
        query: 'Test query',
      };

      await expect(apiService.chat(request)).rejects.toThrow('Internal server error');
    });
  });

  describe('getModules', () => {
    it('should fetch modules', async () => {
      const mockResponse = {
        modules: [
          {
            id: 1,
            title: 'Module 1: ROS 2',
            slug: 'module-1',
            description: 'Learn ROS 2',
            chapter_count: 5,
          },
        ],
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => mockResponse,
      });

      const response = await apiService.getModules();

      expect(global.fetch).toHaveBeenCalledWith(
        'http://localhost:8000/api/content/modules',
        expect.any(Object)
      );

      expect(response).toEqual(mockResponse);
    });
  });

  describe('search', () => {
    it('should search content with query', async () => {
      const mockResponse = {
        query: 'ROS 2',
        results: [
          {
            id: 1,
            title: 'ROS 2 Architecture',
            slug: 'ros2-architecture',
            snippet: 'ROS 2 is a framework...',
            chapter_slug: 'ros2-architecture',
            chapter_title: 'ROS 2 Architecture',
            module_slug: 'module-1',
            module_title: 'Module 1',
          },
        ],
        total: 1,
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => mockResponse,
      });

      const response = await apiService.search('ROS 2');

      expect(global.fetch).toHaveBeenCalledWith(
        expect.stringContaining('/api/content/search?q=ROS%202'),
        expect.any(Object)
      );

      expect(response).toEqual(mockResponse);
    });

    it('should include module filter in search', async () => {
      const mockResponse = {
        query: 'nodes',
        results: [],
        total: 0,
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => mockResponse,
      });

      await apiService.search('nodes', 10, 'module-1');

      expect(global.fetch).toHaveBeenCalledWith(
        expect.stringContaining('module_filter=module-1'),
        expect.any(Object)
      );
    });
  });

  describe('healthCheck', () => {
    it('should return health status', async () => {
      const mockResponse = {
        status: 'healthy',
        version: '1.0.0',
        services: {
          api: 'running',
          rag: 'ready',
        },
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => mockResponse,
      });

      const response = await apiService.healthCheck();

      expect(response).toEqual(mockResponse);
    });
  });
});
