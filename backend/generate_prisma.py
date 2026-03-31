#!/usr/bin/env python3
"""
Prisma generate script for Docker build.
"""
import subprocess
import sys

print("=" * 50, flush=True)
print("Starting Prisma Generate", flush=True)
print("=" * 50, flush=True)

result = subprocess.run(
    [sys.executable, "-m", "prisma", "generate"],
    capture_output=False,
    text=True
)

if result.returncode != 0:
    print("ERROR: Prisma generate failed!", flush=True)
    sys.exit(1)

print("\nSUCCESS: Prisma client generated!", flush=True)
sys.exit(0)
