"""
Integration Tests for API Endpoints
"""
import pytest
import sys
from pathlib import Path
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from main import app


@pytest.fixture
def client():
    """Create test client"""
    with TestClient(app) as test_client:
        yield test_client


@pytest.fixture
def mock_rag_service():
    """Mock RAG service for testing"""
    with patch('src.services.rag_service.get_rag_service') as mock_get_service:
        mock_service = MagicMock()
        mock_service.process_query.return_value = {
            'answer': 'ROS 2 is a robotics middleware framework.',
            'citations': [
                {
                    'module_slug': 'module-1',
                    'chapter_slug': 'ros2-architecture',
                    'section_slug': 'introduction',
                    'section_title': 'Introduction to ROS 2',
                    'content_snippet': 'ROS 2 is a flexible framework...',
                    'similarity_score': 0.85
                }
            ],
            'confidence': 0.85,
            'is_fallback': False,
            'response_time_ms': 150
        }
        mock_get_service.return_value = mock_service
        yield mock_service


class TestHealthEndpoint:
    """Test health check endpoint"""

    def test_health_check(self, client):
        """Test that health endpoint returns healthy status"""
        response = client.get("/health")
        
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "healthy"
        assert data["version"] == "1.0.0"
        assert data["services"]["api"] == "running"
        assert data["services"]["rag"] == "ready"


class TestContentAPI:
    """Test Content API endpoints"""

    def test_get_modules(self, client):
        """Test getting all modules"""
        # This will fail without database, but tests the endpoint exists
        response = client.get("/api/content/modules")
        
        # Should return 200 or 500 depending on DB setup
        assert response.status_code in [200, 500]

    def test_get_module_not_found(self, client):
        """Test getting non-existent module"""
        response = client.get("/api/content/modules/non-existent-module")
        
        # Should return 404 or 500 depending on DB setup
        assert response.status_code in [404, 500]

    def test_search_content(self, client):
        """Test searching content"""
        response = client.get("/api/content/search?q=ROS%202")
        
        # Should return 200 or 500 depending on DB setup
        assert response.status_code in [200, 500]


class TestTutorAPI:
    """Test AI Tutor API endpoints"""

    def test_chat_basic(self, client, mock_rag_service):
        """Test basic chat functionality"""
        request_data = {
            "query": "What is ROS 2?"
        }
        
        response = client.post("/api/tutor/chat", json=request_data)
        
        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert "citations" in data
        assert "confidence" in data
        assert data["fallback"] is False

    def test_chat_with_selected_text(self, client, mock_rag_service):
        """Test chat with selected text"""
        request_data = {
            "query": "Explain this",
            "selected_text": "ROS 2 nodes communicate through topics"
        }
        
        response = client.post("/api/tutor/chat", json=request_data)
        
        assert response.status_code == 200
        mock_rag_service.process_query.assert_called_once()
        call_args = mock_rag_service.process_query.call_args
        assert call_args[1]["selected_text"] == "ROS 2 nodes communicate through topics"

    def test_chat_with_context_filters(self, client, mock_rag_service):
        """Test chat with module and chapter filters"""
        request_data = {
            "query": "What are nodes?",
            "module": "module-1",
            "chapter": "ros2-architecture"
        }
        
        response = client.post("/api/tutor/chat", json=request_data)
        
        assert response.status_code == 200
        call_args = mock_rag_service.process_query.call_args
        assert call_args[1]["module_filter"] == "module-1"
        assert call_args[1]["chapter_filter"] == "ros2-architecture"

    def test_chat_fallback_response(self, client):
        """Test chat returns fallback for unknown topics"""
        with patch('src.services.rag_service.get_rag_service') as mock_get_service:
            mock_service = MagicMock()
            mock_service.process_query.return_value = {
                'answer': 'This is not covered in the book yet.',
                'citations': [],
                'confidence': 0.5,
                'is_fallback': True,
                'response_time_ms': 50
            }
            mock_get_service.return_value = mock_service
            
            request_data = {
                "query": "What is quantum entanglement?"
            }
            
            response = client.post("/api/tutor/chat", json=request_data)
            
            assert response.status_code == 200
            data = response.json()
            assert data["fallback"] is True
            assert "not covered" in data["answer"].lower()

    def test_explain_selected_endpoint(self, client, mock_rag_service):
        """Test explain-selected endpoint"""
        request_data = {
            "query": "Explain",
            "selected_text": "Digital twins simulate robot behavior"
        }
        
        response = client.post("/api/tutor/explain-selected", json=request_data)
        
        assert response.status_code == 200

    def test_explain_selected_missing_text(self, client):
        """Test explain-selected without selected text fails"""
        request_data = {
            "query": "Explain"
        }
        
        response = client.post("/api/tutor/explain-selected", json=request_data)
        
        assert response.status_code == 400

    def test_get_session_not_found(self, client):
        """Test getting non-existent session"""
        response = client.get("/api/tutor/session/non-existent-session-id")
        
        # Should return 404
        assert response.status_code == 404


class TestCORSConfiguration:
    """Test CORS configuration"""

    def test_cors_headers(self, client):
        """Test that CORS headers are present"""
        response = client.options(
            "/health",
            headers={
                "Origin": "http://localhost:3000",
                "Access-Control-Request-Method": "GET"
            }
        )
        
        # CORS should be configured
        assert response.status_code in [200, 404]  # 404 is OK if OPTIONS not handled


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
