# FREE API Setup Guide - 100% Free AI Services

This project uses **100% FREE AI services** for LLM and embeddings. No credit card required!

## 🎯 Free Services Used

| Service | Purpose | Free Tier | Sign Up |
|---------|---------|-----------|---------|
| **Groq** | LLM (Llama 3) | Unlimited (currently free) | [console.groq.com](https://console.groq.com) |
| **HuggingFace** | Embeddings | 30k requests/month | [huggingface.co](https://huggingface.co) |
| **Qdrant Cloud** | Vector DB | 1GB storage | [cloud.qdrant.io](https://cloud.qdrant.io) |
| **Neon** | PostgreSQL | 0.5GB storage | [neon.tech](https://neon.tech) |
| **Vercel** | Hosting | 100GB/month | [vercel.com](https://vercel.com) |

**Total Cost: $0/month** 🎉

---

## 📝 Step-by-Step Setup

### 1. Groq API Key (FREE LLM) - 2 minutes

Groq provides **ultra-fast** Llama 3 inference, currently **completely free**.

1. Go to [https://console.groq.com](https://console.groq.com)
2. Click **"Sign Up"** (use Google/GitHub for quick signup)
3. After login, click **"API Keys"** in left sidebar
4. Click **"Create API Key"**
5. Give it a name (e.g., "agent-book-factory")
6. **Copy the key** (starts with `gsk_...`)
   - ⚠️ Save it now! You can't see it again.
7. Click **"Done"**

**Model to use:** `llama3-8b-8192` (Llama 3, 8B parameters, 8192 context)

**Rate Limits:**
- Currently unlimited during beta
- Very generous: ~30 requests/minute

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
   - ⚠️ Save it! You'll need it for deployment.

**Model to use:** `sentence-transformers/all-MiniLM-L6-v2`

**Rate Limits:**
- 30,000 requests/month (free tier)
- Enough for ~1,000 user queries
- Resets every month

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

**Rate Limits:**
- 1GB storage (~10,000 textbook sections)
- 500 requests/minute

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

**Rate Limits:**
- 0.5GB storage
- 200 hours compute/month (enough for continuous operation)
- Auto-scales to zero when idle (saves resources)

---

## 🔧 Configure Your Project

### Create `.env` file

In the `backend` folder, create `.env`:

```bash
cd backend
cp .env.example .env
```

Edit `.env` and fill in your keys:

```env
# Groq (FREE LLM)
GROQ_API_KEY=gsk_your-actual-key-here
GROQ_MODEL=llama3-8b-8192

# HuggingFace (FREE Embeddings)
HUGGINGFACE_API_KEY=hf_your-actual-token-here
HUGGINGFACE_EMBEDDING_MODEL=sentence-transformers/all-MiniLM-L6-v2

# Qdrant Cloud (FREE Vector DB)
QDRANT_URL=https://your-cluster.qdrant.tech
QDRANT_API_KEY=your-actual-api-key

# Neon Database (FREE PostgreSQL)
DATABASE_URL=postgresql://user:password@host:port/database

# CORS
ALLOWED_ORIGINS=http://localhost:3000,https://your-app.vercel.app
```

---

## 🚀 Deploy to Vercel

### 1. Install Vercel CLI

```bash
npm install -g vercel
```

### 2. Login

```bash
vercel login
```

### 3. Link Project

```bash
vercel link
```

### 4. Set Environment Variables

```bash
vercel env add GROQ_API_KEY production
vercel env add HUGGINGFACE_API_KEY production
vercel env add QDRANT_URL production
vercel env add QDRANT_API_KEY production
vercel env add DATABASE_URL production
vercel env add ALLOWED_ORIGINS production
```

### 5. Deploy

```bash
vercel --prod
```

---

## ✅ Verify Deployment

1. **Health Check:** Visit `https://your-app.vercel.app/health`
   - Should show: `{"status": "healthy"}`

2. **Test Chat:** Open `https://your-app.vercel.app` and ask a question

3. **Check Logs:** Run `vercel logs` to see function execution

---

## 🔍 Troubleshooting

### "GROQ_API_KEY is required"

Make sure you set the environment variable in Vercel Dashboard.

### "HuggingFace API error: 401"

Check that your HuggingFace token is correct and starts with `hf_`.

### "Qdrant connection failed"

Verify your Qdrant URL includes `https://` and API key is correct.

### "Database connection error"

Make sure your Neon connection string includes username, password, and database name.

### "Rate limit exceeded"

- **HuggingFace**: Wait until next month or upgrade to Pro ($9/month)
- **Groq**: Currently unlimited, but may add limits in future
- **Qdrant**: 500 requests/minute should be enough

---

## 📊 Free Tier Capacity

For a **school project** or **prototype**:

| Service | Free Limit | Estimated Usage |
|---------|------------|-----------------|
| Groq | Unlimited | ~500 queries/day ✅ |
| HuggingFace | 30k/month | ~1,000 queries/day ✅ |
| Qdrant | 1GB | ~10,000 sections ✅ |
| Neon | 0.5GB | ~100,000 rows ✅ |
| Vercel | 100GB | ~10,000 visits/month ✅ |

All free tiers are **more than enough** for a hackathon or class project!

---

## 🔗 Useful Links

- **Groq Docs**: [console.groq.com/docs](https://console.groq.com/docs)
- **HuggingFace API**: [huggingface.co/docs/api-inference](https://huggingface.co/docs/api-inference)
- **Qdrant Cloud**: [qdrant.tech/documentation/cloud](https://qdrant.tech/documentation/cloud)
- **Neon**: [neon.tech/docs](https://neon.tech/docs)
- **Vercel**: [vercel.com/docs](https://vercel.com/docs)

---

## 💡 Tips

1. **Save all API keys** in a password manager
2. **Monitor usage** in each service's dashboard
3. **Use environment variables** - never commit keys to Git
4. **Test locally** before deploying
5. **Set up alerts** if available (HuggingFace has usage dashboard)

---

## 🎓 Educational Use

All these services offer **free tiers for education**:

- **GitHub Education Pack**: Includes credits for various services
- **HuggingFace for Education**: Extended limits for teachers
- **Qdrant for Research**: Extended free tier for academic projects

Contact each provider if you need extended limits for your school project!
