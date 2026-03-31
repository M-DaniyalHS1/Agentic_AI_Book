#!/usr/bin/env python3
"""
Prisma generate script for Docker build.
Uses prisma.generator package internal API.
"""
import sys
import os
import asyncio

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

# Debug: show prisma package location
import prisma
print(f"\nPrisma package location: {prisma.__file__}", flush=True)

# Try using prisma.cli.main
print("\nTrying prisma.cli.main approach...", flush=True)
try:
    from prisma.cli import main
    # Simulate command line args
    sys.argv = ['prisma', 'generate']
    main.main()
    print("\nSUCCESS: Prisma client generated!", flush=True)
    sys.exit(0)
except Exception as e:
    print(f"prisma.cli.main failed: {e}", flush=True)
    import traceback
    traceback.print_exc()

# Try using generator directly with proper API
print("\nTrying generator.run approach...", flush=True)
try:
    from prisma.generator import generator
    
    async def run():
        # Run the generator with schema path
        await generator.run(
            schema=schema_path,
            data=None
        )
    
    asyncio.run(run())
    print("\nSUCCESS: Prisma client generated!", flush=True)
    sys.exit(0)
except Exception as e:
    print(f"generator.run failed: {e}", flush=True)
    import traceback
    traceback.print_exc()

# Last resort: try to find and run the binary
print("\nTrying to find generator binary...", flush=True)
import site
import subprocess
import glob

site_packages = site.getsitepackages()[0]
prisma_dir = os.path.join(site_packages, "prisma")

# Find all files
found = []
for root, dirs, files in os.walk(prisma_dir):
    for f in files:
        full_path = os.path.join(root, f)
        found.append(full_path)

print(f"Found {len(found)} files in prisma package", flush=True)
for f in found[:20]:  # Show first 20
    print(f"  {f}", flush=True)

# Look for any executable
for root, dirs, files in os.walk(prisma_dir):
    for f in files:
        if not f.endswith('.py'):
            full_path = os.path.join(root, f)
            print(f"Found non-Python file: {full_path}", flush=True)
            try:
                os.chmod(full_path, 0o755)
                result = subprocess.run([full_path, "--help"], capture_output=True, text=True, timeout=5)
                print(f"  It's executable! Output: {result.stdout[:100]}", flush=True)
            except Exception as ex:
                print(f"  Not executable or error: {ex}", flush=True)

print("\nERROR: All approaches failed!", flush=True)
sys.exit(1)
