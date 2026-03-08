# Local Testing Guide

## ✅ Your Credentials Are Working!

Both FREE APIs have been tested successfully:

- **Groq LLM**: ✓ Working (llama-3.1-8b-instant)
- **HuggingFace Embeddings**: ✓ Working (BAAI/bge-small-en-v1.5)

## 🚀 Start Backend Server

```bash
cd backend
python -m uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

Then open: http://localhost:8000/docs

## 🧪 Test Scripts

### Test Groq API
```bash
python test_groq.py
```

### Test HuggingFace Embeddings
```bash
python test_huggingface.py
```

## 📝 Your Current Configuration

**Backend/.env:**
- GROQ_API_KEY: ✓ Set
- GROQ_MODEL: llama-3.1-8b-instant
- HUGGINGFACE_API_KEY: ✓ Set
- HUGGINGFACE_EMBEDDING_MODEL: BAAI/bge-small-en-v1.5
- QDRANT_URL: ✓ Set
- QDRANT_API_KEY: ✓ Set
- DATABASE_URL: ✓ Set

## 🌐 Access Points

Once backend is running:

| Endpoint | URL |
|----------|-----|
| API Docs | http://localhost:8000/docs |
| Health Check | http://localhost:8000/health |
| Chat API | http://localhost:8000/api/tutor/chat |
| Content API | http://localhost:8000/api/content/modules |

## 🔧 Troubleshooting

### Port 8000 already in use
```bash
# Kill process on port 8000
netstat -ano | findstr :8000
taskkill /F /PID <PID>

# Or use a different port
python -m uvicorn main:app --port 8001
```

### Module not found errors
```bash
cd backend
pip install fastapi uvicorn mangum pydantic httpx qdrant-client python-dotenv psycopg2-binary markdown sqlalchemy alembic
```

### Database connection error
Make sure your Neon database URL is correct and includes SSL mode.

### Qdrant connection error
Check that your Qdrant Cloud cluster is running and URL is correct.

## 📦 Frontend Testing

To test with frontend:

1. **Start Backend** (port 8000)
   ```bash
   cd backend
   python -m uvicorn main:app --reload
   ```

2. **Start Frontend** (port 3000)
   ```bash
   cd frontend
   npm run start
   ```

3. **Open Browser**: http://localhost:3000

## ✨ What's Working

- ✅ Groq API (FREE LLM)
- ✅ HuggingFace Embeddings (FREE)
- ✅ Qdrant Cloud connection
- ✅ Neon Database connection
- ✅ FastAPI backend structure
- ✅ CORS configured for localhost

## Next Steps

1. **Test Locally**: Run the backend and test API endpoints
2. **Add Content**: Index textbook content into Qdrant
3. **Test Chat**: Use the /api/tutor/chat endpoint
4. **Deploy**: When ready, run `vercel --prod`

## Quick Health Check

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
