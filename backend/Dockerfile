# Dockerfile for Railway deployment
# Uses Python 3.11 (compatible with all packages)
# BUILD: v20 - correct provider prisma-client-py

FROM python:3.11-slim

WORKDIR /app

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

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements-railway.txt

# Copy generate script and application code
COPY generate_prisma.py /app/generate_prisma.py
COPY . .

# Generate Prisma client
RUN python /app/generate_prisma.py

# Expose port (Railway sets PORT env var)
EXPOSE 8000

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD python -c "import httpx; httpx.get('http://localhost:8000/api/health')" || exit 1

# Start application
CMD ["python", "start.py"]
