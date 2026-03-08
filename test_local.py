"""
Test script to verify backend API works locally
"""
import asyncio
import sys
sys.path.insert(0, 'backend')

from src.utils.embedding_utils import generate_embedding
from src.utils.llm_utils import generate_response

async def test_embedding():
    """Test HuggingFace embedding generation"""
    print("Testing HuggingFace Embeddings...")
    try:
        embedding = await generate_embedding("Hello, this is a test!")
        print(f"  [OK] Generated embedding with {len(embedding)} dimensions")
        return True
    except Exception as e:
        print(f"  [ERROR] {e}")
        return False

async def test_llm():
    """Test Groq LLM generation"""
    print("Testing Groq LLM...")
    try:
        messages = [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": "Say hello in one sentence."}
        ]
        response = await generate_response(messages, max_tokens=50)
        print(f"  [OK] LLM Response: {response[:100]}...")
        return True
    except Exception as e:
        print(f"  [ERROR] {e}")
        return False

async def main():
    print("=" * 50)
    print("Testing FREE API Integrations")
    print("=" * 50)
    print()
    
    embedding_ok = await test_embedding()
    print()
    llm_ok = await test_llm()
    print()
    
    print("=" * 50)
    if embedding_ok and llm_ok:
        print("[SUCCESS] All tests passed!")
    else:
        print("[FAILED] Some tests failed. Check your API keys.")
    print("=" * 50)

if __name__ == "__main__":
    asyncio.run(main())
