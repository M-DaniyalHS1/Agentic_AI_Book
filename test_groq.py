"""
Quick test of Groq API (LLM) only
"""
import asyncio
import os
import httpx

async def test_groq():
    # Load .env
    from dotenv import load_dotenv
    load_dotenv('backend/.env')
    
    api_key = os.getenv("GROQ_API_KEY")
    model = os.getenv("GROQ_MODEL", "llama-3.1-8b-instant")
    
    print(f"Testing Groq API with model: {model}")
    
    async with httpx.AsyncClient() as client:
        response = await client.post(
            "https://api.groq.com/openai/v1/chat/completions",
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json"
            },
            json={
                "model": model,
                "messages": [
                    {"role": "system", "content": "You are a helpful assistant."},
                    {"role": "user", "content": "Say hello in one sentence."}
                ],
                "max_tokens": 50
            },
            timeout=30.0
        )
        
        if response.status_code == 200:
            data = response.json()
            print(f"[SUCCESS] Groq API works!")
            print(f"Response: {data['choices'][0]['message']['content']}")
            return True
        else:
            print(f"[ERROR] {response.status_code}: {response.text}")
            return False

if __name__ == "__main__":
    result = asyncio.run(test_groq())
    if result:
        print("\nGroq LLM is ready to use!")
    else:
        print("\nCheck your GROQ_API_KEY in backend/.env")
