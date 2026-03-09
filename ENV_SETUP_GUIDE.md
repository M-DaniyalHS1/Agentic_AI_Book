# Environment Variables Setup Guide

## Required Environment Variables for Vercel Deployment

Your RAG chatbot is **not working** because the backend API needs these environment variables to be set in Vercel.

---

## 🔑 Step 1: Get Your FREE API Keys

### 1. Groq API Key (FREE LLM) - 2 minutes

Groq provides **ultra-fast** Llama 3 inference, **completely free**.

1. Go to [https://console.groq.com](https://console.groq.com)
2. Click **"Sign Up"** (use Google/GitHub for quick signup)
3. After login, click **"API Keys"** in left sidebar
4. Click **"Create API Key"**
5. Give it a name (e.g., "agent-book-factory")
6. **Copy the key** (starts with `gsk_...`)
   - ⚠️ **Save it now!** You can't see it again.

**Model:** `llama3-8b-8192` or `llama-3.1-8b-instant`

---

### 2. HuggingFace Token (FREE Embeddings) - 2 minutes

HuggingFace provides free access to sentence-transformers models.

1. Go to [https://huggingface.co/settings/tokens](https://huggingface.co/settings/tokens)
2. Click **"Sign Up"** (use Google/GitHub)
3. After login, click **"New token"**
4. Give it a name (e.g., "agent-book-factory")
5. Select **Type: "Read"** (no write access needed)
6. Click **"Generate token"**
7. **Copy the token** (starts with `hf_...`)

**Model:** `sentence-transformers/all-MiniLM-L6-v2`

---

### 3. Qdrant Cloud (FREE Vector Database) - 3 minutes

Qdrant Cloud stores your textbook embeddings.

1. Go to [https://cloud.qdrant.io](https://cloud.qdrant.io)
2. Click **"Sign Up"** (use Google/GitHub)
3. After login, click **"Create Cluster"**
4. Select **Plan: "Free"** (1GB storage)
5. Choose a region (closest to you)
6. Click **"Create"**
7. Wait ~2 minutes for cluster to be ready
8. Click on your cluster
9. **Copy these values:**
   - **URL** (e.g., `https://xxxx-xxxx.cloud.qdrant.io`)
   - **API Key** (click "Show" to reveal)

---

### 4. Neon Database (FREE PostgreSQL) - 2 minutes

Neon provides serverless PostgreSQL for your app's data.

1. Go to [https://neon.tech](https://neon.tech)
2. Click **"Sign Up"** (use Google/GitHub)
3. After login, click **"Create project"**
4. Give it a name (e.g., "agent-book-factory")
5. Keep default settings
6. Click **"Create project"**
7. On the dashboard, find **"Connection string"**
8. **Copy the connection string** (looks like: `postgresql://user:password@host/neondb`)
   - Make sure to select **"Pooled"** connection for serverless

---

## 🚀 Step 2: Set Environment Variables in Vercel

### Option A: Using Vercel Dashboard (Recommended)

1. Go to your project in Vercel: [https://vercel.com/dashboard](https://vercel.com/dashboard)
2. Click on your project: **agent-book-factory**
3. Go to **Settings** → **Environment Variables**
4. Click **"Add New"** for each variable below:

```
GROQ_API_KEY=gsk_your-actual-key-here
GROQ_MODEL=llama3-8b-8192

HUGGINGFACE_API_KEY=hf_your-actual-token-here
HUGGINGFACE_EMBEDDING_MODEL=sentence-transformers/all-MiniLM-L6-v2

QDRANT_URL=https://your-cluster.qdrant.tech
QDRANT_API_KEY=your-actual-api-key

DATABASE_URL=postgresql://user:password@host:port/database

ALLOWED_ORIGINS=http://localhost:3000,https://agenticaibook-seven.vercel.app

LLM_PROVIDER=groq
```

5. Make sure to select **"Production"** and **"Preview"** for each variable
6. Click **"Save"**

### Option B: Using Vercel CLI

```bash
# Login to Vercel
vercel login

# Navigate to project directory
cd D:\class11\hackathon_1\agent_book_factory

# Set each environment variable
vercel env add GROQ_API_KEY production
vercel env add GROQ_MODEL production
vercel env add HUGGINGFACE_API_KEY production
vercel env add HUGGINGFACE_EMBEDDING_MODEL production
vercel env add QDRANT_URL production
vercel env add QDRANT_API_KEY production
vercel env add DATABASE_URL production
vercel env add ALLOWED_ORIGINS production
vercel env add LLM_PROVIDER production

# Deploy to apply changes
vercel --prod
```

---

## 📝 Step 3: Rebuild and Redeploy

After setting environment variables:

```bash
# Navigate to project
cd D:\class11\hackathon_1\agent_book_factory

# Push to trigger Vercel deployment
git add .
git commit -m "chore: update api/index.py with better path handling"
git push origin master

# Or redeploy manually
vercel --prod
```

---

## ✅ Step 4: Verify Deployment

### Test API Health Check

```bash
# Test health endpoint
curl https://agenticaibook-seven.vercel.app/api/health

# Should return:
# {"status":"healthy","version":"1.0.0","services":{"api":"running","rag":"ready"}}
```

### Test in Browser

1. Open: `https://agenticaibook-seven.vercel.app`
2. Look for **floating purple button** in bottom-right corner
3. Click the button to open chat widget
4. Ask: "What is ROS 2?"
5. You should get a response with citations

---

## 🔍 Troubleshooting

### Issue: `/api/health` returns 500 Error

**Cause:** Missing environment variables

**Fix:**
1. Check Vercel Dashboard → Settings → Environment Variables
2. Verify all variables are set for **Production**
3. Redeploy: `vercel --prod`

### Issue: AI Tutor button not visible

**Cause:** JavaScript error or CSS issue

**Fix:**
1. Open browser DevTools (F12)
2. Check **Console** tab for errors
3. Check **Elements** tab for `.ai-tutor-fab` element
4. Verify CSS is loaded (check `custom.css` import)

### Issue: Chat returns errors

**Cause:** Qdrant or Groq API key invalid

**Fix:**
1. Verify API keys are correct
2. Check Qdrant cluster is active
3. Test Groq API separately:
   ```bash
   curl https://api.groq.com/openai/v1/chat/completions \
     -H "Authorization: Bearer YOUR_GROQ_KEY" \
     -H "Content-Type: application/json" \
     -d '{"model":"llama3-8b-8192","messages":[{"role":"user","content":"Hello"}]}'
   ```

### Issue: No citations in responses

**Cause:** Content not indexed in Qdrant

**Fix:**
```bash
# Run content indexer locally
cd backend
python scripts/rebuild_embeddings.py
```

Then redeploy to Vercel.

---

## 📊 Environment Variables Reference

| Variable | Required | Example | Description |
|----------|----------|---------|-------------|
| `GROQ_API_KEY` | ✅ | `gsk_xxx` | Groq API key for LLM |
| `GROQ_MODEL` | ✅ | `llama3-8b-8192` | Groq model to use |
| `HUGGINGFACE_API_KEY` | ✅ | `hf_xxx` | HuggingFace token for embeddings |
| `HUGGINGFACE_EMBEDDING_MODEL` | ✅ | `sentence-transformers/all-MiniLM-L6-v2` | Embedding model |
| `QDRANT_URL` | ✅ | `https://xxx.qdrant.tech` | Qdrant Cloud URL |
| `QDRANT_API_KEY` | ✅ | `xxx` | Qdrant API key |
| `DATABASE_URL` | ✅ | `postgresql://...` | Neon PostgreSQL URL |
| `ALLOWED_ORIGINS` | ✅ | `http://localhost:3000,https://your-app.vercel.app` | CORS origins |
| `LLM_PROVIDER` | ✅ | `groq` | LLM provider (groq/huggingface/openai) |

---

## 🎯 Success Criteria

After setup, you should see:

- ✅ `/api/health` returns `{"status": "healthy"}`
- ✅ AI Tutor floating button visible (bottom-right, purple)
- ✅ Clicking button opens chat widget
- ✅ Chat responds with textbook-based answers
- ✅ Responses include citations with chapter/section links

---

**Last Updated:** 2026-03-09
**Deployment URL:** https://agenticaibook-seven.vercel.app
