#!/usr/bin/env python3
"""
Prisma generate script for Docker build.
Uses the generator module directly to avoid shell command issues.
"""
import asyncio
import sys
import os

print("=" * 50, flush=True)
print("Starting Prisma Generate", flush=True)
print("=" * 50, flush=True)

# Check if schema exists
schema_path = "./prisma/schema.prisma"
print(f"Checking schema at: {os.path.abspath(schema_path)}", flush=True)
if not os.path.exists(schema_path):
    print(f"ERROR: Schema not found at {schema_path}", flush=True)
    sys.exit(1)
print("Schema found!", flush=True)

# Try to run generate using prisma.generator module
print("\nUsing prisma.generator module directly...", flush=True)

try:
    from prisma.generator import generator
    
    async def run_generate():
        # Get the generator instance
        gen = generator.Generator()
        # Run the generation
        await gen.generate()
        return 0
    
    result = asyncio.run(run_generate())
    print("\nSUCCESS: Prisma client generated!", flush=True)
    sys.exit(0)
    
except Exception as e:
    print(f"ERROR: Generator failed with: {e}", flush=True)
    import traceback
    traceback.print_exc()
    sys.exit(1)
