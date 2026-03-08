"""
LLM Utility - Free AI Models via Groq and HuggingFace
Supports: Groq (free Llama), HuggingFace Inference API, OpenAI (optional)
"""
import os
import httpx
from typing import Optional, List, Dict
from dotenv import load_dotenv

load_dotenv()

# Provider selection
LLM_PROVIDER = os.getenv("LLM_PROVIDER", "groq").lower()

# Groq Configuration (FREE - Llama models)
GROQ_API_KEY = os.getenv("GROQ_API_KEY", "")
GROQ_MODEL = os.getenv("GROQ_MODEL", "llama-3.1-8b-instant")
GROQ_API_URL = "https://api.groq.com/openai/v1/chat/completions"

# HuggingFace Configuration
HUGGINGFACE_API_KEY = os.getenv("HUGGINGFACE_API_KEY", "")
HUGGINGFACE_LLM_MODEL = os.getenv("HUGGINGFACE_LLM_MODEL", "meta-llama/Meta-Llama-3-8B-Instruct")
HUGGINGFACE_API_URL = f"https://api-inference.huggingface.co/models/{HUGGINGFACE_LLM_MODEL}/v1/chat/completions"

# OpenAI Configuration (optional fallback)
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")
OPENAI_MODEL = os.getenv("OPENAI_MODEL", "gpt-3.5-turbo")
OPENAI_CHAT_URL = "https://api.openai.com/v1/chat/completions"


async def generate_response(
    messages: List[Dict[str, str]],
    max_tokens: int = 500,
    temperature: float = 0.7
) -> str:
    """
    Generate LLM response using selected provider

    Args:
        messages: List of message dicts with 'role' and 'content'
        max_tokens: Maximum tokens in response
        temperature: Sampling temperature

    Returns:
        Generated response text
    """
    if LLM_PROVIDER == "groq":
        return await _generate_response_groq(messages, max_tokens, temperature)
    elif LLM_PROVIDER == "huggingface":
        return await _generate_response_huggingface(messages, max_tokens, temperature)
    elif LLM_PROVIDER == "openai":
        return await _generate_response_openai(messages, max_tokens, temperature)
    else:
        raise ValueError(f"Unknown LLM provider: {LLM_PROVIDER}")


async def _generate_response_groq(
    messages: List[Dict[str, str]],
    max_tokens: int = 500,
    temperature: float = 0.7
) -> str:
    """Generate response using Groq API (FREE - fast Llama models)"""
    if not GROQ_API_KEY:
        raise ValueError("GROQ_API_KEY is required. Get free key at https://console.groq.com")
    
    async with httpx.AsyncClient() as client:
        response = await client.post(
            GROQ_API_URL,
            headers={
                "Authorization": f"Bearer {GROQ_API_KEY}",
                "Content-Type": "application/json"
            },
            json={
                "model": GROQ_MODEL,
                "messages": messages,
                "max_tokens": max_tokens,
                "temperature": temperature
            },
            timeout=30.0
        )
        
        if response.status_code != 200:
            raise Exception(f"Groq API error: {response.status_code} - {response.text}")
        
        data = response.json()
        return data["choices"][0]["message"]["content"]


async def _generate_response_huggingface(
    messages: List[Dict[str, str]],
    max_tokens: int = 500,
    temperature: float = 0.7
) -> str:
    """Generate response using HuggingFace Inference API"""
    if not HUGGINGFACE_API_KEY:
        raise ValueError("HUGGINGFACE_API_KEY is required")
    
    async with httpx.AsyncClient() as client:
        response = await client.post(
            HUGGINGFACE_API_URL,
            headers={
                "Authorization": f"Bearer {HUGGINGFACE_API_KEY}",
                "Content-Type": "application/json"
            },
            json={
                "model": HUGGINGFACE_LLM_MODEL,
                "messages": messages,
                "max_tokens": max_tokens,
                "temperature": temperature
            },
            timeout=60.0
        )
        
        if response.status_code != 200:
            raise Exception(f"HuggingFace API error: {response.status_code} - {response.text}")
        
        data = response.json()
        return data["choices"][0]["message"]["content"]


async def _generate_response_openai(
    messages: List[Dict[str, str]],
    max_tokens: int = 500,
    temperature: float = 0.7
) -> str:
    """Generate response using OpenAI API"""
    if not OPENAI_API_KEY:
        raise ValueError("OPENAI_API_KEY is required")
    
    async with httpx.AsyncClient() as client:
        response = await client.post(
            OPENAI_CHAT_URL,
            headers={
                "Authorization": f"Bearer {OPENAI_API_KEY}",
                "Content-Type": "application/json"
            },
            json={
                "model": OPENAI_MODEL,
                "messages": messages,
                "max_tokens": max_tokens,
                "temperature": temperature
            },
            timeout=30.0
        )
        
        if response.status_code != 200:
            raise Exception(f"OpenAI API error: {response.status_code} - {response.text}")
        
        data = response.json()
        return data["choices"][0]["message"]["content"]


# Synchronous versions
def generate_response_sync(
    messages: List[Dict[str, str]],
    max_tokens: int = 500,
    temperature: float = 0.7
) -> str:
    """Synchronous version of generate_response"""
    import asyncio
    try:
        loop = asyncio.get_event_loop()
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
    return loop.run_until_complete(generate_response(messages, max_tokens, temperature))
