#!/usr/bin/env python3
"""
Prisma generate script for Docker build.
Uses subprocess with full path to generator binary.
"""
import subprocess
import sys
import os
import site

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

# Find the prisma generator binary
site_packages = site.getsitepackages()[0]
generator_bin = os.path.join(site_packages, "prisma", "generator", "bin", "prisma-client-python")

print(f"\nLooking for generator at: {generator_bin}", flush=True)
if not os.path.exists(generator_bin):
    print(f"ERROR: Generator binary not found at {generator_bin}", flush=True)
    # Try to find it
    import glob
    found = glob.glob(os.path.join(site_packages, "prisma", "generator", "bin", "*"))
    print(f"Files in generator/bin: {found}", flush=True)
    sys.exit(1)
print(f"Generator binary found!", flush=True)

# Run prisma generate with explicit generator path
print("\nRunning prisma generate with explicit generator...", flush=True)
result = subprocess.run(
    [generator_bin, "--schema", schema_path],
    capture_output=False,
    text=True
)

print(f"\nReturn code: {result.returncode}", flush=True)
if result.returncode != 0:
    print("ERROR: Prisma generate failed!", flush=True)
    sys.exit(1)

print("\nSUCCESS: Prisma client generated!", flush=True)
sys.exit(0)
