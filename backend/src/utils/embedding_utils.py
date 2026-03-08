"""
Unified Embedding Utility - Supports Multiple Free Providers
Providers: HuggingFace, Groq (coming soon), OpenAI (optional)
"""
import os
import httpx
from typing import List
from dotenv import load_dotenv

load_dotenv()

# Provider selection
EMBEDDING_PROVIDER = os.getenv("EMBEDDING_PROVIDER", "huggingface").lower()

# HuggingFace Configuration
HUGGINGFACE_API_KEY = os.getenv("HUGGINGFACE_API_KEY", "")
HUGGINGFACE_EMBEDDING_MODEL = os.getenv(
    "HUGGINGFACE_EMBEDDING_MODEL",
    "sentence-transformers/all-MiniLM-L6-v2"
)
# Updated HuggingFace Inference API endpoint (2024+)
HUGGINGFACE_API_URL = f"https://router.huggingface.co/hf-inference/models/{HUGGINGFACE_EMBEDDING_MODEL}"

# OpenAI Configuration (optional fallback)
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")
OPENAI_EMBEDDING_MODEL = os.getenv("OPENAI_EMBEDDING_MODEL", "text-embedding-ada-002")
OPENAI_EMBEDDING_URL = "https://api.openai.com/v1/embeddings"

# Embedding dimensions by model
EMBEDDING_DIMENSIONS = {
    "sentence-transformers/all-MiniLM-L6-v2": 384,
    "BAAI/bge-small-en-v1.5": 384,
    "text-embedding-ada-002": 1536,
}


def get_embedding_dimension() -> int:
    """Get embedding dimension based on selected model"""
    if EMBEDDING_PROVIDER == "huggingface":
        return EMBEDDING_DIMENSIONS.get(HUGGINGFACE_EMBEDDING_MODEL, 384)
    elif EMBEDDING_PROVIDER == "openai":
        return EMBEDDING_DIMENSIONS.get(OPENAI_EMBEDDING_MODEL, 1536)
    return 384


async def generate_embedding(text: str) -> List[float]:
    """
    Generate embedding using selected provider

    Args:
        text: The text to embed

    Returns:
        List of floats representing the embedding vector
    """
    if EMBEDDING_PROVIDER == "huggingface":
        return await _generate_embedding_huggingface(text)
    elif EMBEDDING_PROVIDER == "openai":
        return await _generate_embedding_openai(text)
    else:
        raise ValueError(f"Unknown embedding provider: {EMBEDDING_PROVIDER}")


async def _generate_embedding_huggingface(text: str) -> List[float]:
    """Generate embedding using HuggingFace Inference API"""
    async with httpx.AsyncClient() as client:
        response = await client.post(
            HUGGINGFACE_API_URL,
            headers={
                "Authorization": f"Bearer {HUGGINGFACE_API_KEY}" if HUGGINGFACE_API_KEY else "",
                "Content-Type": "application/json"
            },
            json={
                "inputs": text,
            },
            timeout=30.0
        )
        
        if response.status_code != 200:
            raise Exception(f"HuggingFace API error: {response.status_code} - {response.text}")
        
        data = response.json()
        # Handle different response formats
        if isinstance(data, dict) and "embeddings" in data:
            return data["embeddings"][0]
        elif isinstance(data, list) and len(data) > 0:
            return data[0] if isinstance(data[0], list) else data
        elif isinstance(data, dict) and "data" in data:
            return data["data"][0]
        return data


async def _generate_embedding_openai(text: str) -> List[float]:
    """Generate embedding using OpenAI API"""
    if not OPENAI_API_KEY:
        raise ValueError("OPENAI_API_KEY is required for OpenAI embeddings")
    
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
    """Generate embeddings for multiple texts"""
    if EMBEDDING_PROVIDER == "huggingface":
        return await _generate_embeddings_huggingface(texts)
    elif EMBEDDING_PROVIDER == "openai":
        return await _generate_embeddings_openai(texts)
    raise ValueError(f"Unknown embedding provider: {EMBEDDING_PROVIDER}")


async def _generate_embeddings_huggingface(texts: List[str]) -> List[List[float]]:
    """Batch generate embeddings using HuggingFace"""
    async with httpx.AsyncClient() as client:
        response = await client.post(
            HUGGINGFACE_API_URL,
            headers={
                "Authorization": f"Bearer {HUGGINGFACE_API_KEY}" if HUGGINGFACE_API_KEY else "",
                "Content-Type": "application/json"
            },
            json={
                "inputs": texts,
                "options": {"wait_for_model": True}
            },
            timeout=60.0
        )
        
        if response.status_code != 200:
            raise Exception(f"HuggingFace API error: {response.status_code} - {response.text}")
        
        return response.json()


async def _generate_embeddings_openai(texts: List[str]) -> List[List[float]]:
    """Batch generate embeddings using OpenAI"""
    if not OPENAI_API_KEY:
        raise ValueError("OPENAI_API_KEY is required for OpenAI embeddings")
    
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
        embeddings = sorted(data["data"], key=lambda x: x["index"])
        return [e["embedding"] for e in embeddings]


async def generate_query_embedding(query: str) -> List[float]:
    """Generate embedding for search query"""
    return await generate_embedding(query)


# Synchronous versions
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
