/**
 * API Service for communicating with the backend
 * For Vercel full-stack deployment, uses relative URLs (same domain)
 */

// For Vercel full-stack: use relative path (API and frontend on same domain)
// For local/dev: use localhost:8001
// Note: Docusaurus doesn't support process.env in browser code
const API_BASE_URL = 'http://127.0.0.1:8001';

export interface ChatRequest {
  query: string;
  selected_text?: string;
  module?: string;
  chapter?: string;
  session_id?: string;
}

export interface Citation {
  module_slug: string;
  chapter_slug: string;
  section_slug: string;
  section_title: string;
  content_snippet: string;
  similarity_score: number;
}

export interface ChatResponse {
  answer: string;
  citations: Citation[];
  confidence: number;
  fallback: boolean;
  session_id: string;
  response_time_ms: number;
}

export interface SearchResult {
  id: number;
  title: string;
  slug: string;
  snippet: string;
  chapter_slug: string;
  chapter_title: string;
  module_slug: string;
  module_title: string;
}

export interface SearchResponse {
  query: string;
  results: SearchResult[];
  total: number;
}

export interface Module {
  id: number;
  title: string;
  slug: string;
  description: string;
  chapter_count: number;
}

export interface Chapter {
  id: number;
  title: string;
  slug: string;
  sidebar_position: number;
  section_count: number;
}

export interface Section {
  id: number;
  title: string;
  slug: string;
  content: string;
  content_html: string;
  word_count: number;
  chapter: {
    slug: string;
    title: string;
  };
  module: {
    slug: string;
    title: string;
  };
  navigation: {
    prev: { slug: string; title: string } | null;
    next: { slug: string; title: string } | null;
  };
}

class ApiService {
  private baseUrl: string;

  constructor() {
    this.baseUrl = API_BASE_URL;
  }

  private async request<T>(endpoint: string, options?: RequestInit): Promise<T> {
    const response = await fetch(`${this.baseUrl}${endpoint}`, {
      ...options,
      headers: {
        'Content-Type': 'application/json',
        ...options?.headers,
      },
    });

    if (!response.ok) {
      const error = await response.json().catch(() => ({ detail: 'Request failed' }));
      throw new Error(error.detail || `HTTP ${response.status}`);
    }

    return response.json();
  }

  // Chat/Tutor API
  async chat(request: ChatRequest): Promise<ChatResponse> {
    return this.request('/api/tutor/chat', {
      method: 'POST',
      body: JSON.stringify(request),
    });
  }

  async explainSelected(request: ChatRequest): Promise<ChatResponse> {
    return this.request('/api/tutor/explain-selected', {
      method: 'POST',
      body: JSON.stringify(request),
    });
  }

  async getSessionHistory(sessionId: string) {
    return this.request(`/api/tutor/session/${sessionId}`);
  }

  // Content API
  async getModules(): Promise<{ modules: Module[] }> {
    return this.request('/api/content/modules');
  }

  async getModule(moduleSlug: string) {
    return this.request(`/api/content/modules/${moduleSlug}`);
  }

  async getChapter(chapterSlug: string) {
    return this.request(`/api/content/chapters/${chapterSlug}`);
  }

  async getSection(sectionSlug: string): Promise<{ section: Section }> {
    return this.request(`/api/content/sections/${sectionSlug}`);
  }

  async search(query: string, limit: number = 10, moduleFilter?: string): Promise<SearchResponse> {
    const params = new URLSearchParams({
      q: query,
      limit: limit.toString(),
      ...(moduleFilter && { module_filter: moduleFilter }),
    });
    return this.request(`/api/content/search?${params}`);
  }

  // Health check
  async healthCheck() {
    return this.request('/health');
  }
}

export const apiService = new ApiService();
export default apiService;
