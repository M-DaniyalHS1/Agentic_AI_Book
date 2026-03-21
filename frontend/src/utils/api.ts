/**
 * API utility for making authenticated requests to the backend
 */

// Get API base URL - hardcoded for local development
// Using 'localhost' (not '127.0.0.1') for cookie domain consistency
const API_BASE_URL = 'http://localhost:8001';

/**
 * Make a fetch request with proper headers and credentials
 */
export async function fetchApi(
  endpoint: string,
  options: RequestInit = {}
): Promise<Response> {
  const url = `${API_BASE_URL}${endpoint}`;
  
  const config: RequestInit = {
    ...options,
    headers: {
      'Content-Type': 'application/json',
      ...(options.headers || {}),
    },
    credentials: 'include', // Include cookies for auth
  };

  const response = await fetch(url, config);
  return response;
}

/**
 * Parse JSON from response with error handling
 */
export async function parseResponse<T>(response: Response): Promise<T> {
  const data = await response.json();
  
  if (!response.ok) {
    throw new Error(
      data.detail?.message || data.message || `HTTP ${response.status}`
    );
  }
  
  return data;
}

/**
 * Combined helper for API calls with JSON parsing
 */
export async function apiRequest<T>(
  endpoint: string,
  options: RequestInit = {}
): Promise<T> {
  const response = await fetchApi(endpoint, options);
  return parseResponse<T>(response);
}
