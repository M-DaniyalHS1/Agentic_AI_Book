# Vercel Full-Stack Deployment Guide

## Prerequisites - ALL FREE SERVICES

1. **Vercel Account**: Sign up at [vercel.com](https://vercel.com) - FREE
2. **Vercel CLI**: Install with `npm i -g vercel`
3. **Qdrant Cloud**: Set up at [cloud.qdrant.io](https://cloud.qdrant.io) - FREE 1GB tier
4. **Groq API**: Get FREE key at [console.groq.com](https://console.groq.com) - FREE Llama models
5. **HuggingFace**: Get FREE token at [huggingface.co/settings/tokens](https://huggingface.co/settings/tokens) - FREE 30k requests/month
6. **Neon PostgreSQL**: Set up at [neon.tech](https://neon.tech) - FREE 0.5GB tier

## Architecture

This deployment uses **full-stack on Vercel** with **100% FREE services**:
- **Frontend**: Docusaurus static site (served from CDN) - FREE
- **Backend**: FastAPI serverless functions (API routes) - FREE (100GB/month)
- **LLM**: Groq API (Llama 3 models) - FREE
- **Embeddings**: HuggingFace Inference API - FREE (30k requests/month)
- **Vector DB**: Qdrant Cloud - FREE (1GB)
- **Database**: Neon PostgreSQL - FREE (0.5GB)

## Step-by-Step Deployment

### Step 1: Get FREE API Keys

#### 1. Groq API Key (FREE LLM)
1. Go to [https://console.groq.com](https://console.groq.com)
2. Sign up / Login
3. Create API key
4. Copy the key (starts with `gsk_`)

#### 2. HuggingFace Token (FREE Embeddings)
1. Go to [https://huggingface.co/settings/tokens](https://huggingface.co/settings/tokens)
2. Sign up / Login
3. Create new token (type: "Read")
4. Copy the token (starts with `hf_`)

#### 3. Qdrant Cloud (FREE Vector DB)
1. Go to [https://cloud.qdrant.io](https://cloud.qdrant.io)
2. Sign up / Login
3. Create cluster (FREE tier)
4. Copy URL and API key

#### 4. Neon Database (FREE PostgreSQL)
1. Go to [https://neon.tech](https://neon.tech)
2. Sign up / Login
3. Create project
4. Copy connection string

### Step 2: Environment Variables

You need these values (ALL FREE):

```
# Groq (FREE LLM)
GROQ_API_KEY=gsk_your-groq-api-key
GROQ_MODEL=llama3-8b-8192

# HuggingFace (FREE Embeddings)
HUGGINGFACE_API_KEY=hf_your-huggingface-key
HUGGINGFACE_EMBEDDING_MODEL=sentence-transformers/all-MiniLM-L6-v2

# Qdrant Cloud (FREE Vector DB)
QDRANT_URL=https://your-cluster.qdrant.tech
QDRANT_API_KEY=your-qdrant-api-key

# Neon Database (FREE PostgreSQL)
DATABASE_URL=postgresql://user:password@host:port/database

# CORS
ALLOWED_ORIGINS=http://localhost:3000,https://your-app.vercel.app
```

### Step 2: Install Vercel CLI

```bash
npm install -g vercel
```

### Step 3: Login to Vercel

```bash
vercel login
```

### Step 4: Link Project to Vercel

```bash
vercel link
```

- Select "Create New Project"
- Name: `agent-book-factory`
- Scope: Your account

### Step 5: Set Environment Variables in Vercel

```bash
# Set each variable
vercel env add QDRANT_URL production
vercel env add QDRANT_API_KEY production
vercel env add OPENAI_API_KEY production
vercel env add EMBEDDING_MODEL production
vercel env add DATABASE_URL production
vercel env add ALLOWED_ORIGINS production
```

Or set them in the Vercel Dashboard:
1. Go to Project Settings > Environment Variables
2. Add each variable for "Production"

### Step 6: Deploy to Production

```bash
vercel --prod
```

### Step 7: Initialize Qdrant Collection

After deployment, trigger the health endpoint to initialize Qdrant:

```bash
curl https://your-app.vercel.app/health
```

### Step 8: Test the Deployment

1. Open `https://your-app.vercel.app` in browser
2. Test the chat functionality
3. Check API endpoints: `https://your-app.vercel.app/api/health`

## Troubleshooting

### "Module not found" errors

Make sure `api/index.py` exists and imports are correct.

### CORS errors

Update `ALLOWED_ORIGINS` in Vercel environment variables to include your production domain.

### "QDRANT_URL not set" errors

Ensure all environment variables are set in Vercel Dashboard.

### Function timeout errors

Increase timeout in `vercel.json`:
```json
"functions": {
  "api/*.py": {
    "maxDuration": 60
  }
}
```

### Large dependency size

The project uses `requirements-serverless.txt` which excludes heavy ML libraries. If you still face issues:
- Remove `qdrant-client` and use direct HTTP calls
- Use lighter OpenAI model

## Local Testing (Serverless Mode)

Test the serverless configuration locally:

```bash
# Install serverless dependencies
cd backend
pip install -r requirements-serverless.txt

# Create .env with production values
cp .env.example .env

# Test with Vercel dev server
vercel dev
```

## Rollback

If deployment fails:

```bash
# List deployments
vercel ls

# Rollback to previous
vercel rollback <deployment-url>
```

## Monitoring

- **Logs**: `vercel logs`
- **Dashboard**: Project > Deployments > Click deployment > Function Logs
- **Metrics**: Project > Analytics

## Cost Estimate - ALL FREE!

| Service | Free Tier | Paid (if needed) |
|---------|-----------|------------------|
| Vercel | 100GB/month | $20/month (Pro) |
| Groq API | Unlimited (currently free) | Future paid plans |
| HuggingFace | 30k requests/month | $9/month (Pro) |
| Qdrant Cloud | 1GB free | $15/month |
| Neon PostgreSQL | 0.5GB free | $19/month |

**Total: $0/month** (all services have generous free tiers)

## Free Tier Limits

- **Groq**: Currently free during beta, very generous rate limits
- **HuggingFace**: 30,000 requests/month (enough for ~1000 user queries)
- **Qdrant**: 1GB storage (~10,000 textbook sections)
- **Neon**: 0.5GB storage, 200 hours compute/month
- **Vercel**: 100GB bandwidth/month

For a school project or prototype, all free tiers should be sufficient!

## Commands Reference

```bash
# Deploy to preview
vercel

# Deploy to production
vercel --prod

# View logs
vercel logs

# List deployments
vercel ls

# Remove deployment
vercel rm <deployment-url>

# Pull environment variables
vercel env pull
```

## Next Steps

1. Set up custom domain in Vercel Dashboard
2. Configure automatic deployments from Git
3. Set up monitoring and alerts
4. Optimize function cold starts
5. Implement caching for embeddings

## Support

- [Vercel Documentation](https://vercel.com/docs)
- [FastAPI on Vercel](https://vercel.com/docs/functions/serverless-functions/runtimes/python)
- [Qdrant Cloud Docs](https://qdrant.tech/documentation/cloud/)
