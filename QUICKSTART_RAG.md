# Quickstart: RAG Pipeline Setup

This guide walks you through setting up and testing the complete RAG (Retrieval-Augmented Generation) pipeline.

## Prerequisites

- Docker & Docker Compose installed
- Python 3.11+ with pip
- Node.js 18+ with npm
- OpenAI API key (or compatible LLM API)

## Step 1: Start Infrastructure

```bash
# From project root
docker-compose up -d
```

Verify services are running:

```bash
docker-compose ps
```

Expected output:
- `agent_book_postgres` - healthy
- `agent_book_qdrant` - healthy
- `agent_book_pgadmin` - (optional)

## Step 2: Configure Backend

```bash
cd backend

# Create virtual environment
python -m venv venv

# Activate (Windows)
venv\Scripts\activate
# Activate (macOS/Linux)
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Copy environment file
cp .env.example .env
```

Edit `.env`:

```env
DATABASE_URL=postgresql://agentbook:agentbook_secret@localhost:5432/agent_book_db
QDRANT_URL=http://localhost:6333
OPENAI_API_KEY=sk-your-key-here
OPENAI_BASE_URL=https://api.openai.com/v1
LLM_MODEL=gpt-3.5-turbo
EMBEDDING_MODEL=all-MiniLM-L6-v2
```

## Step 3: Initialize Database Tables

```bash
cd backend

# Start Python shell
python

# Run migrations (if using Alembic)
# from alembic.command import upgrade
# from alembic.config import Config
# alembic_cfg = Config("alembic.ini")
# upgrade(alembic_cfg, "head")
```

Or create tables directly:

```python
from src.database import engine, Base
Base.metadata.create_all(bind=engine)
```

## Step 4: Index Textbook Content in Qdrant

```bash
cd backend
python scripts/rebuild_embeddings.py
```

Expected output:
```
📚 Indexing textbook content from: ../frontend/docs
📄 Found 15 markdown files

📖 Processing: module-1/ros2-architecture.md
  ✓ Indexed: ROS 2 Architecture
  ✓ Indexed: Communication Patterns
  ...

==================================================
✅ Indexing complete!
   Indexed: 45 sections
   Errors: 0
==================================================
```

## Step 5: Start Backend Server

```bash
cd backend
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

Verify API is running:

```bash
curl http://localhost:8000/health
```

Expected response:
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "services": {
    "api": "running",
    "rag": "ready"
  }
}
```

## Step 6: Test RAG Pipeline Manually

### Test 1: Health Check

```bash
curl http://localhost:8000/health
```

### Test 2: Chat API

```bash
curl -X POST http://localhost:8000/api/tutor/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?"
  }'
```

Expected response:
```json
{
  "answer": "ROS 2 is a flexible framework...",
  "citations": [
    {
      "module_slug": "module-1",
      "chapter_slug": "ros2-architecture",
      "section_title": "Introduction to ROS 2",
      "similarity_score": 0.85
    }
  ],
  "confidence": 0.85,
  "fallback": false,
  "response_time_ms": 150
}
```

### Test 3: Search API

```bash
curl "http://localhost:8000/api/content/search?q=nodes"
```

### Test 4: Content API

```bash
curl http://localhost:8000/api/content/modules
```

## Step 7: Configure Frontend

```bash
cd frontend

# Install dependencies
npm install

# Copy environment file
cp .env.example .env
```

Edit `.env`:

```env
REACT_APP_API_URL=http://localhost:8000
```

## Step 8: Start Frontend

```bash
cd frontend
npm run start
```

Visit `http://localhost:3000`

## Step 9: Test AI Tutor in Browser

1. Open `http://localhost:3000`
2. Click the chat icon (bottom-right corner)
3. Ask a question: "What is ROS 2?"
4. Verify:
   - Answer appears
   - Citations are shown
   - Confidence score is displayed

5. Navigate to any textbook page
6. Select some text
7. Click "Explain this" popup
8. Verify explanation appears with citations

## Troubleshooting

### Qdrant Connection Error

```
Error: Connection refused to http://localhost:6333
```

**Solution**: Ensure Docker container is running:
```bash
docker-compose up -d qdrant
docker-compose logs qdrant
```

### Database Connection Error

```
sqlalchemy.exc.OperationalError: could not connect to server
```

**Solution**: Check PostgreSQL container:
```bash
docker-compose up -d postgres
docker-compose logs postgres
```

### No Citations in Response

**Possible causes**:
1. Content not indexed in Qdrant
2. Similarity threshold too high
3. Query embedding dimension mismatch

**Solution**:
```bash
# Rebuild embeddings
python scripts/rebuild_embeddings.py

# Check Qdrant collection
curl http://localhost:6333/collections/textbook_content
```

### LLM API Error

```
Error generating LLM response: Invalid API key
```

**Solution**: 
1. Verify `OPENAI_API_KEY` in `.env`
2. Check API key has credits
3. Test API directly:
```bash
curl https://api.openai.com/v1/models \
  -H "Authorization: Bearer $OPENAI_API_KEY"
```

### Frontend Can't Connect to Backend

**Solution**:
1. Verify backend is running on port 8000
2. Check `REACT_APP_API_URL` in frontend `.env`
3. Check CORS settings in `backend/main.py`

## Performance Optimization

### Adjust RAG Parameters

In `backend/src/services/rag_service.py`:

```python
TOP_K_RESULTS = 5          # Number of results to retrieve
SIMILARITY_THRESHOLD = 0.7 # Minimum similarity score
MAX_CONTEXT_TOKENS = 4000  # Max tokens in context
MAX_RESPONSE_TOKENS = 500  # Max tokens in response
```

### Use Self-Hosted LLM

For cost savings, use Ollama or vLLM:

```env
OPENAI_API_KEY=ollama
OPENAI_BASE_URL=http://localhost:11434/v1
LLM_MODEL=llama2:13b
```

## Next Steps

- ✅ RAG pipeline is working
- 📚 Add more textbook content to `frontend/docs/`
- 🧪 Run test suites: `pytest` and `npm test`
- 🚀 Deploy to Vercel
- 📊 Monitor usage and improve retrieval quality

## Additional Resources

- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [Docusaurus Documentation](https://docusaurus.io/)
- [LangChain Documentation](https://python.langchain.com/)
