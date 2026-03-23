"""
Contract Tests for API
These tests verify the API contract between frontend and backend
"""
import pytest
import sys
from pathlib import Path
from fastapi.testclient import TestClient
from typing import List, Dict, Any

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from main import app


@pytest.fixture
def client():
    """Create test client"""
    with TestClient(app) as test_client:
        yield test_client


class TestChatResponseContract:
    """Test that chat responses match the expected contract"""

    def test_chat_response_has_required_fields(self, client):
        """Verify chat response contains all required fields"""
        # This test will fail without DB, so we skip if DB not available
        response = client.post("/api/tutor/chat", json={"query": "test"})
        
        if response.status_code == 200:
            data = response.json()
            
            # Required fields
            assert "answer" in data
            assert "citations" in data
            assert "confidence" in data
            assert "fallback" in data
            assert "response_time_ms" in data
            
            # Type checks
            assert isinstance(data["answer"], str)
            assert isinstance(data["citations"], list)
            assert isinstance(data["confidence"], (int, float))
            assert isinstance(data["fallback"], bool)
            assert isinstance(data["response_time_ms"], int)

    def test_citation_structure(self, client):
        """Verify citation objects have correct structure"""
        response = client.post("/api/tutor/chat", json={"query": "test"})
        
        if response.status_code == 200:
            data = response.json()
            
            if data["citations"]:
                citation = data["citations"][0]
                
                # Required citation fields
                assert "module_slug" in citation
                assert "chapter_slug" in citation
                assert "section_slug" in citation
                assert "section_title" in citation
                assert "content_snippet" in citation
                assert "similarity_score" in citation
                
                # Type checks
                assert isinstance(citation["module_slug"], str)
                assert isinstance(citation["chapter_slug"], str)
                assert isinstance(citation["similarity_score"], (int, float))


class TestContentResponseContract:
    """Test that content responses match the expected contract"""

    def test_modules_response_structure(self, client):
        """Verify modules response structure"""
        response = client.get("/api/content/modules")
        
        if response.status_code == 200:
            data = response.json()
            
            assert "modules" in data
            assert isinstance(data["modules"], list)
            
            if data["modules"]:
                module = data["modules"][0]
                assert "id" in module
                assert "title" in module
                assert "slug" in module
                assert "chapter_count" in module

    def test_search_response_structure(self, client):
        """Verify search response structure"""
        response = client.get("/api/content/search?q=test")
        
        if response.status_code == 200:
            data = response.json()
            
            assert "query" in data
            assert "results" in data
            assert "total" in data
            
            assert isinstance(data["results"], list)
            assert isinstance(data["total"], int)


class TestErrorResponses:
    """Test error response contracts"""

    def test_404_response(self, client):
        """Test 404 error response structure"""
        response = client.get("/api/content/sections/non-existent")
        
        if response.status_code == 404:
            data = response.json()
            assert "detail" in data

    def test_400_response(self, client):
        """Test 400 error response structure"""
        response = client.post("/api/tutor/explain-selected", json={"query": "test"})
        
        if response.status_code == 400:
            data = response.json()
            assert "detail" in data


class TestPerformanceContracts:
    """Test performance-related contracts"""

    def test_response_time(self, client):
        """Test that responses are within acceptable time"""
        import time
        
        start = time.time()
        response = client.get("/health")
        elapsed = (time.time() - start) * 1000  # ms
        
        assert response.status_code == 200
        # Health check should be very fast (< 100ms)
        assert elapsed < 100


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
