"""
Embedding Utility Functions using OpenAI API
Generate vector embeddings for text using OpenAI's text-embedding-ada-002
This is serverless-compatible (no heavy ML dependencies)
"""
import os
import httpx
from typing import List
from dotenv import load_dotenv

load_dotenv()

# Model configuration
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")
OPENAI_EMBEDDING_MODEL = os.getenv("EMBEDDING_MODEL", "text-embedding-ada-002")
EMBEDDING_DIMENSION = 1536  # Dimension for text-embedding-ada-002

# OpenAI API endpoint
OPENAI_EMBEDDING_URL = "https://api.openai.com/v1/embeddings"


async def generate_embedding(text: str) -> List[float]:
    """
    Generate a single embedding vector for text using OpenAI API

    Args:
        text: The text to embed

    Returns:
        List of floats representing the embedding vector
    """
    if not OPENAI_API_KEY:
        raise ValueError("OPENAI_API_KEY environment variable is not set")

    async with httpx.AsyncClient() as client:
        response = await client.post(
            OPENAI_EMBEDDING_URL,
            headers={
                "Authorization": f"Bearer {OPENAI_API_KEY}",
                "Content-Type": "application/json"
            },
            json={
                "input": text,
                "model": OPENAI_EMBEDDING_MODEL
            },
            timeout=30.0
        )
        
        if response.status_code != 200:
            raise Exception(f"OpenAI API error: {response.status_code} - {response.text}")
        
        data = response.json()
        return data["data"][0]["embedding"]


async def generate_embeddings(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings for multiple texts in batch using OpenAI API

    Args:
        texts: List of texts to embed (max 2048 tokens per batch)

    Returns:
        List of embedding vectors
    """
    if not OPENAI_API_KEY:
        raise ValueError("OPENAI_API_KEY environment variable is not set")

    # OpenAI supports batch processing
    async with httpx.AsyncClient() as client:
        response = await client.post(
            OPENAI_EMBEDDING_URL,
            headers={
                "Authorization": f"Bearer {OPENAI_API_KEY}",
                "Content-Type": "application/json"
            },
            json={
                "input": texts,
                "model": OPENAI_EMBEDDING_MODEL
            },
            timeout=60.0
        )
        
        if response.status_code != 200:
            raise Exception(f"OpenAI API error: {response.status_code} - {response.text}")
        
        data = response.json()
        # Sort by index to maintain order
        embeddings = sorted(data["data"], key=lambda x: x["index"])
        return [e["embedding"] for e in embeddings]


async def generate_query_embedding(query: str) -> List[float]:
    """
    Generate embedding for a search/query text
    
    Args:
        query: The search query

    Returns:
        Embedding vector
    """
    return await generate_embedding(query)


# Synchronous versions for non-async contexts
def generate_embedding_sync(text: str) -> List[float]:
    """Synchronous version of generate_embedding"""
    import asyncio
    try:
        loop = asyncio.get_event_loop()
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
    return loop.run_until_complete(generate_embedding(text))


def generate_embeddings_sync(texts: List[str]) -> List[List[float]]:
    """Synchronous version of generate_embeddings"""
    import asyncio
    try:
        loop = asyncio.get_event_loop()
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
    return loop.run_until_complete(generate_embeddings(texts))
