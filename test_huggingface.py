"""
Quick test of HuggingFace Embeddings
"""
import asyncio
import os
import httpx

async def test_huggingface():
    # Load .env
    from dotenv import load_dotenv
    load_dotenv('backend/.env')
    
    api_key = os.getenv("HUGGINGFACE_API_KEY")
    model = os.getenv("HUGGINGFACE_EMBEDDING_MODEL", "sentence-transformers/all-MiniLM-L6-v2")
    
    # Use the new HuggingFace Inference API (serverless)
    api_url = f"https://router.huggingface.co/hf-inference/models/{model}"
    
    print(f"Testing HuggingFace Embeddings with model: {model}")
    
    async with httpx.AsyncClient() as client:
        response = await client.post(
            api_url,
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json"
            },
            json={
                "inputs": "Hello, this is a test sentence for embedding.",
            },
            timeout=30.0
        )
        
        print(f"Status: {response.status_code}")
        
        if response.status_code == 200:
            data = response.json()
            print(f"[SUCCESS] HuggingFace API works!")
            
            # Parse response
            if isinstance(data, list) and len(data) > 0:
                embedding = data[0] if isinstance(data[0], list) else data
                print(f"Embedding dimensions: {len(embedding)}")
                print(f"First 5 values: {embedding[:5]}")
                return True
            else:
                print(f"Response: {data}")
                return True
        else:
            print(f"[ERROR] {response.status_code}: {response.text}")
            return False

if __name__ == "__main__":
    result = asyncio.run(test_huggingface())
    if result:
        print("\nHuggingFace Embeddings are ready to use!")
    else:
        print("\nCheck your HUGGINGFACE_API_KEY in backend/.env")
        print("Try a different model from: https://huggingface.co/models?pipeline_tag=feature-extraction")
