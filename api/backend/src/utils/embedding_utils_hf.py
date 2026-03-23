"""
Embedding Utility Functions using HuggingFace Inference API
Free alternative to OpenAI embeddings
Uses sentence-transformers via HuggingFace API
"""
import os
import httpx
from typing import List
from dotenv import load_dotenv

load_dotenv()

# HuggingFace API Configuration
HUGGINGFACE_API_KEY = os.getenv("HUGGINGFACE_API_KEY", "")
HUGGINGFACE_EMBEDDING_MODEL = os.getenv(
    "EMBEDDING_MODEL", 
    "sentence-transformers/all-MiniLM-L6-v2"
)

# HuggingFace Inference API endpoint
HUGGINGFACE_API_URL = f"https://api-inference.huggingface.co/models/{HUGGINGFACE_EMBEDDING_MODEL}"

# Alternative: Free embedding APIs
# - https://huggingface.co/sentence-transformers/all-MiniLM-L6-v2
# - https://huggingface.co/BAAI/bge-small-en-v1.5


async def generate_embedding(text: str) -> List[float]:
    """
    Generate a single embedding vector for text using HuggingFace API

    Args:
        text: The text to embed

    Returns:
        List of floats representing the embedding vector
    """
    async with httpx.AsyncClient() as client:
        response = await client.post(
            HUGGINGFACE_API_URL,
            headers={
                "Authorization": f"Bearer {HUGGINGFACE_API_KEY}" if HUGGINGFACE_API_KEY else "",
                "Content-Type": "application/json"
            },
            json={
                "inputs": text,
                "options": {
                    "wait_for_model": True
                }
            },
            timeout=30.0
        )
        
        if response.status_code != 200:
            raise Exception(f"HuggingFace API error: {response.status_code} - {response.text}")
        
        data = response.json()
        # HuggingFace returns list of embeddings for batch, we want single
        if isinstance(data, list) and len(data) > 0:
            return data[0] if isinstance(data[0], list) else data
        return data


async def generate_embeddings(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings for multiple texts in batch using HuggingFace API

    Args:
        texts: List of texts to embed

    Returns:
        List of embedding vectors
    """
    async with httpx.AsyncClient() as client:
        response = await client.post(
            HUGGINGFACE_API_URL,
            headers={
                "Authorization": f"Bearer {HUGGINGFACE_API_KEY}" if HUGGINGFACE_API_KEY else "",
                "Content-Type": "application/json"
            },
            json={
                "inputs": texts,
                "options": {
                    "wait_for_model": True
                }
            },
            timeout=60.0
        )
        
        if response.status_code != 200:
            raise Exception(f"HuggingFace API error: {response.status_code} - {response.text}")
        
        data = response.json()
        return data


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
