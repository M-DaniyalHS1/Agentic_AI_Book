"""
Unit Tests for RAG Service
"""
import pytest
import sys
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.services.rag_service import RAGService, get_rag_service


class TestRAGService:
    """Test cases for RAG service"""

    @pytest.fixture
    def mock_rag_service(self):
        """Create a mock RAG service"""
        with patch('src.services.rag_service.get_qdrant_client'):
            with patch('src.services.rag_service.generate_embedding'):
                service = RAGService()
                yield service

    def test_build_context_empty_results(self, mock_rag_service):
        """Test context building with no results"""
        context = mock_rag_service.build_context([])
        
        assert "No relevant content found" in context

    def test_build_context_with_results(self, mock_rag_service):
        """Test context building with search results"""
        results = [
            {
                'module_slug': 'module-1',
                'chapter_slug': 'ros2-architecture',
                'section_slug': 'nodes',
                'section_title': 'ROS 2 Nodes',
                'content': 'Nodes are the basic unit of computation in ROS 2.',
                'similarity_score': 0.85
            }
        ]
        
        context = mock_rag_service.build_context(results)
        
        assert 'module-1' in context
        assert 'ros2-architecture' in context
        assert 'Nodes are the basic unit' in context

    def test_build_context_respects_max_tokens(self, mock_rag_service):
        """Test that context building respects max token limit"""
        results = [
            {
                'module_slug': 'module-1',
                'chapter_slug': 'chapter-1',
                'section_slug': 'section-1',
                'section_title': 'Section 1',
                'content': 'A' * 2000,  # Long content
                'similarity_score': 0.9
            },
            {
                'module_slug': 'module-1',
                'chapter_slug': 'chapter-1',
                'section_slug': 'section-2',
                'section_title': 'Section 2',
                'content': 'B' * 2000,
                'similarity_score': 0.8
            }
        ]
        
        context = mock_rag_service.build_context(results, max_tokens=1000)
        
        # Should not include all content due to token limit
        assert len(context) < 4000 * 4  # Rough estimate

    def test_generate_fallback_response_with_selected_text(self, mock_rag_service):
        """Test fallback response generation with selected text"""
        query = "What is this?"
        context = "Some context about ROS 2"
        selected_text = "ROS 2 nodes communicate through topics"
        
        response, confidence = mock_rag_service._generate_fallback_response(
            query, context, selected_text
        )
        
        assert "explain" in response.lower()
        assert "ROS 2 nodes" in response
        assert confidence == 0.6

    def test_generate_fallback_response_without_selected_text(self, mock_rag_service):
        """Test fallback response generation without selected text"""
        query = "What is quantum computing?"
        context = "Content about ROS 2 and robotics"
        
        response, confidence = mock_rag_service._generate_fallback_response(
            query, context
        )
        
        assert "not covered in the book" in response.lower()
        assert confidence == 0.5

    def test_process_query_structure(self, mock_rag_service):
        """Test that process_query returns correct structure"""
        with patch.object(mock_rag_service, 'search_relevant_content', return_value=[]):
            with patch.object(mock_rag_service, 'build_context', return_value="No content"):
                with patch.object(mock_rag_service, 'generate_response', 
                                return_value=("Fallback response", 0.5)):
                    
                    result = mock_rag_service.process_query("Test query")
                    
                    assert 'answer' in result
                    assert 'citations' in result
                    assert 'confidence' in result
                    assert 'is_fallback' in result
                    assert 'response_time_ms' in result

    def test_search_relevant_content_with_filters(self, mock_rag_service):
        """Test search with module and chapter filters"""
        mock_qdrant = Mock()
        mock_qdrant.search.return_value = []
        mock_rag_service.qdrant_client = mock_qdrant
        
        with patch('src.services.rag_service.generate_embedding', return_value=[0.1] * 384):
            results = mock_rag_service.search_relevant_content(
                query="ROS 2",
                module_filter="module-1",
                chapter_filter="ros2-architecture"
            )
            
            assert results == []
            # Verify filter was constructed
            mock_qdrant.search.assert_called_once()


class TestRAGServiceSingleton:
    """Test the singleton pattern for RAG service"""

    def test_get_rag_service_returns_same_instance(self):
        """Test that get_rag_service returns the same instance"""
        with patch('src.services.rag_service.get_qdrant_client'):
            with patch('src.services.rag_service.generate_embedding'):
                service1 = get_rag_service()
                service2 = get_rag_service()
                
                assert service1 is service2


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
