# Vercel Deployment Guide

## Problem: Lambda Size Limit

Vercel Serverless Functions have a **500 MB limit** for dependencies. Our ML dependencies (sentence-transformers, torch, etc.) exceed 7 GB.

## Solution: Separate Frontend and Backend

### Option 1: Frontend on Vercel + Backend on Railway/Render (Recommended)

#### Deploy Frontend to Vercel

```bash
# Deploy only the frontend
cd frontend
npm run build
vercel --prod
```

Configure in Vercel dashboard:
- **Build Command**: `cd frontend && npm install && npm run build`
- **Output Directory**: `frontend/build`
- **Install Command**: `cd frontend && npm install`

#### Deploy Backend to Railway

1. Create account at [railway.app](https://railway.app)
2. Create new project → Deploy from GitHub
3. Set root directory to `backend`
4. Add environment variables:
   - `DATABASE_URL`
   - `QDRANT_URL`
   - `OPENAI_API_KEY`

#### Deploy Backend to Render

1. Create account at [render.com](https://render.com)
2. New Web Service
3. Root directory: `backend`
4. Build command: `pip install -r requirements.txt`
5. Start command: `uvicorn main:app --host 0.0.0.0 --port $PORT`

### Option 2: Use Qdrant Cloud + OpenAI (Vercel Backend Possible)

If you want backend on Vercel, you must:

1. **Remove sentence-transformers** - Use OpenAI embeddings API instead
2. **Use Qdrant Cloud** - Don't bundle Qdrant
3. **Optimize dependencies** - Use `requirements-serverless.txt`

#### Steps:

```bash
# Update embedding utility to use API
# See backend/src/utils/embedding_utils_api.py

# Deploy with serverless requirements
vercel --prod
```

### Option 3: Docker Deployment (Best for Full Control)

Deploy entire stack to a VPS or cloud provider:

```bash
# Build and run with Docker
docker-compose -f docker-compose.prod.yml up -d
```

Recommended providers:
- **Hetzner** - Cheap VPS (~€5/month)
- **DigitalOcean** - Easy deployment
- **AWS EC2** - Scalable
- **Google Cloud Run** - Serverless containers

## Configuration for Each Option

### Option 1: Vercel + Railway

**Frontend (.env):**
```env
REACT_APP_API_URL=https://your-backend.railway.app
```

**Backend (Railway environment):**
```env
DATABASE_URL=postgresql://...
QDRANT_URL=https://your-qdrant.qdrant.tech
QDRANT_API_KEY=your-key
OPENAI_API_KEY=sk-...
```

### Option 2: Vercel Backend Only

**vercel.json:**
```json
{
  "builds": [{
    "src": "backend/vercel_handler.py",
    "use": "@vercel/python",
    "config": {
      "installCommand": "cd backend && pip install -r requirements-serverless.txt --target ."
    }
  }]
}
```

**backend/.env:**
```env
QDRANT_URL=https://cloud.qdrant.io
QDRANT_API_KEY=your-cloud-key
OPENAI_API_KEY=sk-...
EMBEDDING_MODEL=text-embedding-ada-002
```

## Recommended Architecture

```
┌─────────────────┐
│   Vercel CDN    │
│  (Static Site)  │
│   Docusaurus    │
└────────┬────────┘
         │
         │ API Calls
         ↓
┌─────────────────┐
│ Railway/Render  │
│  (FastAPI API)  │
│   RAG Service   │
└────────┬────────┘
         │
         ├──────┬──────────┐
         ↓      ↓          ↓
    ┌─────┐ ┌──────┐ ┌────────┐
    │Qdrant│ │OpenAI│ │Postgres│
    │Cloud │ │ API  │ │ (Neon) │
    └─────┘ └──────┘ └────────┘
```

## Quick Deploy Commands

### Frontend (Vercel)
```bash
vercel --prod
```

### Backend (Railway)
```bash
# Install Railway CLI
npm i -g @railway/cli

# Login and deploy
railway login
railway init
railway up
```

### Backend (Render)
```bash
# Connect GitHub repo to Render
# Configure in web UI
```

## Troubleshooting

### "Dependency size too large"
- Use `requirements-serverless.txt`
- Remove torch, sentence-transformers
- Use external embedding API

### "Cold starts too slow"
- Use Railway/Render (always-on)
- Enable Vercel Provisioned Concurrency
- Use lighter dependencies

### "CORS errors"
- Update `allow_origins` in `backend/main.py`
- Add your production URL

## Cost Comparison

| Option | Frontend | Backend | Total/Month |
|--------|----------|---------|-------------|
| Vercel + Railway | Free | $5 (Hobby) | $5 |
| Vercel + Render | Free | $7 (Basic) | $7 |
| VPS (Docker) | $5 | Included | $5 |
| Vercel Pro + Cloud | $20 | $15 (Qdrant) | $35+ |

## Next Steps

1. **Choose deployment option** (Option 1 recommended)
2. **Set up Qdrant Cloud** (free tier available)
3. **Deploy backend** to Railway/Render
4. **Deploy frontend** to Vercel
5. **Update API URLs** in frontend .env
6. **Test end-to-end**

## Resources

- [Vercel Serverless Functions](https://vercel.com/docs/functions)
- [Railway Documentation](https://docs.railway.app)
- [Render Documentation](https://render.com/docs)
- [Qdrant Cloud](https://cloud.qdrant.io)
