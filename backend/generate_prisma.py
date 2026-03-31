#!/usr/bin/env python3
"""
Prisma generate script for Docker build.
Searches for the generator binary and runs it.
"""
import subprocess
import sys
import os
import site
import glob

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

# Find site-packages
site_packages = site.getsitepackages()[0]
print(f"\nSite packages: {site_packages}", flush=True)

# Search for prisma generator binary
print("\nSearching for prisma generator binary...", flush=True)
prisma_dir = os.path.join(site_packages, "prisma")
print(f"Prisma dir: {prisma_dir}", flush=True)

# Find all executables in prisma package
found_bins = []
for root, dirs, files in os.walk(prisma_dir):
    for f in files:
        if "prisma" in f.lower() or "generator" in f.lower():
            full_path = os.path.join(root, f)
            found_bins.append(full_path)
            print(f"  Found: {full_path}", flush=True)

if not found_bins:
    print("ERROR: No prisma/generator binaries found!", flush=True)
    sys.exit(1)

# Try to find the actual generator executable
generator_bin = None
for path in found_bins:
    if "client" in path.lower() or os.path.basename(path).startswith("prisma"):
        generator_bin = path
        break

if not generator_bin:
    generator_bin = found_bins[0]

print(f"\nUsing generator: {generator_bin}", flush=True)

# Make sure it's executable
os.chmod(generator_bin, 0o755)

# Run prisma generate
print(f"\nRunning: {generator_bin} --schema {schema_path}", flush=True)
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
