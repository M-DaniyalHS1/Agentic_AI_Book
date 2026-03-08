"""
Unit Tests for Embedding Utilities
"""
import pytest
import sys
from pathlib import Path

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.utils.embedding_utils import generate_embedding, generate_embeddings, get_embedding_model


class TestEmbeddingUtils:
    """Test cases for embedding utility functions"""

    def test_generate_embedding_returns_list(self):
        """Test that generate_embedding returns a list of floats"""
        text = "This is a test sentence about ROS 2."
        embedding = generate_embedding(text)
        
        assert isinstance(embedding, list)
        assert len(embedding) == 384  # all-MiniLM-L6-v2 dimension
        assert all(isinstance(x, float) for x in embedding)

    def test_generate_embedding_consistency(self):
        """Test that same text produces same embedding"""
        text = "Physical AI combines perception and action"
        embedding1 = generate_embedding(text)
        embedding2 = generate_embedding(text)
        
        assert embedding1 == embedding2

    def test_generate_embedding_different_texts(self):
        """Test that different texts produce different embeddings"""
        text1 = "ROS 2 is a robotics middleware"
        text2 = "Digital twins simulate robot behavior"
        
        embedding1 = generate_embedding(text1)
        embedding2 = generate_embedding(text2)
        
        # Embeddings should be different (not exactly equal)
        assert embedding1 != embedding2

    def test_generate_embeddings_batch(self):
        """Test batch embedding generation"""
        texts = [
            "Module 1 covers ROS 2 architecture",
            "Module 2 covers digital twins",
            "Module 3 covers AI robot brain"
        ]
        
        embeddings = generate_embeddings(texts)
        
        assert isinstance(embeddings, list)
        assert len(embeddings) == 3
        assert all(len(emb) == 384 for emb in embeddings)

    def test_generate_embedding_empty_string(self):
        """Test embedding generation with empty string"""
        embedding = generate_embedding("")
        
        assert isinstance(embedding, list)
        assert len(embedding) == 384

    def test_generate_embedding_long_text(self):
        """Test embedding with long text"""
        long_text = "ROS 2 nodes communicate through topics. " * 100
        embedding = generate_embedding(long_text)
        
        assert isinstance(embedding, list)
        assert len(embedding) == 384

    def test_get_embedding_model_singleton(self):
        """Test that model is cached"""
        model1 = get_embedding_model()
        model2 = get_embedding_model()
        
        assert model1 is model2


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
