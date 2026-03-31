# Dockerfile for Railway deployment
# Uses Python 3.11 (compatible with all packages)
# BUILD: v15 - cache bust with ARG

FROM python:3.11-slim

WORKDIR /app

# Cache buster - change this value to force rebuild
ARG CACHE_BUST=20260331-v15

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    postgresql-client \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install Node.js for Prisma CLI
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash - \
    && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

# Copy only requirements first (better caching)
COPY requirements-railway.txt .

# Install Python dependencies (includes prisma package for generator)
RUN pip install --no-cache-dir -r requirements-railway.txt

# Add Prisma generator bin to PATH so the binary can be found
ENV PATH="/usr/local/lib/python3.11/site-packages/prisma/generator/bin:$PATH"

# Copy generate_prisma.py script FIRST (before other files to bust cache)
COPY generate_prisma.py /app/generate_prisma.py
RUN ls -la /app/generate_prisma.py && head -20 /app/generate_prisma.py

# Copy application code
COPY . .

# Generate Prisma client using the generate_prisma.py script
RUN python /app/generate_prisma.py

# Expose port (Railway sets PORT env var)
EXPOSE 8000

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD python -c "import httpx; httpx.get('http://localhost:8000/api/health')" || exit 1

# Start application
CMD ["python", "start.py"]
