#!/usr/bin/env python3
"""
Railway startup script for FastAPI backend
Generates Prisma client and starts uvicorn
"""
import subprocess
import os
import sys

def main():
    print("Starting FastAPI backend on Railway...")
    
    # Generate Prisma client
    print("Generating Prisma client...")
    result = subprocess.run(["prisma", "generate"], capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Warning: Prisma generate failed: {result.stderr}")
    else:
        print("Prisma client generated successfully")
    
    # Get port from environment (Railway sets PORT)
    port = int(os.environ.get("PORT", 8000))
    print(f"Starting uvicorn on port {port}...")
    
    # Start uvicorn
    os.execvp("uvicorn", ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", str(port)])

if __name__ == "__main__":
    main()
