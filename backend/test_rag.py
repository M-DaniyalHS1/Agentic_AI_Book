#!/usr/bin/env python
"""Test script for RAG chatbot"""
import requests
import json

url = "http://localhost:8000/api/tutor/chat"
payload = {"query": "What is ROS 2?"}

print(f"Testing RAG chatbot with query: '{payload['query']}'")
print("=" * 60)

try:
    response = requests.post(url, json=payload, timeout=30)
    response.raise_for_status()
    
    data = response.json()
    
    print("\n✅ Response received!\n")
    print(f"Answer: {data['answer'][:500]}...")
    print(f"\nConfidence: {data['confidence']}")
    print(f"Fallback: {data['fallback']}")
    print(f"Response Time: {data['response_time_ms']}ms")
    print(f"\nCitations: {len(data['citations'])} sources")
    
    if data['citations']:
        print("\nTop citation:")
        cit = data['citations'][0]
        print(f"  Module: {cit['module_slug']}")
        print(f"  Chapter: {cit['chapter_slug']}")
        print(f"  Section: {cit['section_title']}")
        print(f"  Score: {cit['similarity_score']}")
        
except requests.exceptions.ConnectionError:
    print("❌ Error: Cannot connect to backend server at http://localhost:8000")
    print("   Make sure the backend is running: python -m uvicorn main:app --reload")
except requests.exceptions.Timeout:
    print("❌ Error: Request timed out")
except Exception as e:
    print(f"❌ Error: {e}")

print("\n" + "=" * 60)
