#!/usr/bin/env python3
"""
Prisma generate script for Docker build.
Runs prisma generate and shows detailed output.
"""
import subprocess
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

# Run prisma generate
print("\nRunning: python -m prisma generate", flush=True)
result = subprocess.run(
    [sys.executable, "-m", "prisma", "generate"],
    capture_output=False,  # Don't capture - let output go to console
    text=True
)

print(f"\nReturn code: {result.returncode}", flush=True)

if result.returncode != 0:
    print("ERROR: Prisma generate failed!", flush=True)
    sys.exit(1)

print("\nSUCCESS: Prisma client generated!", flush=True)
sys.exit(0)
