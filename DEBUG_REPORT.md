# Debug Report: RAG Chatbot Not Working on Vercel

**Date:** 2026-03-09  
**Deployment URL:** https://agenticaibook-seven.vercel.app  
**Status:** ⚠️ Partially Fixed - Awaiting Environment Variables

---

## 🔍 Issues Found

### Issue 1: API Endpoints Returning 500 Error ❌

**Symptom:** `/api/health` returns `FUNCTION_INVOCATION_FAILED` (500 Internal Server Error)

**Root Cause:** Missing environment variables in Vercel deployment

**Required Variables:**
- `GROQ_API_KEY` - LLM API key (free from groq.com)
- `QDRANT_URL` - Qdrant Cloud URL (free from cloud.qdrant.io)
- `QDRANT_API_KEY` - Qdrant API key
- `DATABASE_URL` - PostgreSQL connection string (free from neon.tech)
- `HUGGINGFACE_API_KEY` - Embeddings API key (free from huggingface.co)
- `ALLOWED_ORIGINS` - CORS configuration
- `LLM_PROVIDER` - Provider selection ("groq")

**Fix Applied:** ✅
- Improved `api/index.py` with multi-strategy Python path resolution
- Added debug logging to track import success
- Created `ENV_SETUP_GUIDE.md` with step-by-step setup instructions

**Action Required:** ⚠️ **YOU MUST SET ENVIRONMENT VARIABLES IN VERCEL**

---

### Issue 2: AI Tutor Widget Not Visible ❌

**Symptom:** No floating chat button in bottom-right corner

**Root Causes Found:**
1. Root.tsx theme override may not be properly typed for Docusaurus
2. No client-side fallback if React component fails to load
3. CSS might not be loaded correctly

**Fixes Applied:** ✅

1. **Updated `frontend/src/theme/Root.tsx`:**
   - Added TypeScript interface for props
   - Proper typing for React component

2. **Created `frontend/src/clientModules.js`:**
   - Client-side script that runs in browser
   - Checks if widget is loaded
   - Logs debug information to console
   - Verifies CSS styles are applied

3. **Updated `frontend/docusaurus.config.js`:**
   - Added clientModules.js as deferred script
   - Ensures script loads after DOM is ready

4. **CSS Import Verified:**
   - `@import '../components/AITutorWidget.css';` in `custom.css`
   - Build test passed successfully

**Status:** ⏳ Deployed, awaiting verification

---

## 📝 Changes Made

### Files Modified

| File | Changes |
|------|---------|
| `api/index.py` | Multi-strategy path resolution, debug logging, fallback imports |
| `frontend/src/theme/Root.tsx` | Added TypeScript interface, proper typing |
| `frontend/docusaurus.config.js` | Added clientModules.js script |

### Files Created

| File | Purpose |
|------|---------|
| `ENV_SETUP_GUIDE.md` | Complete guide for setting up FREE API keys |
| `frontend/src/clientModules.js` | Client-side debug and widget verification |
| `DEBUG_REPORT.md` | This file |

---

## 🚀 Deployment Status

### Git Status
```
Commit: 2ab8baf
Message: fix: AI Tutor widget visibility and API path handling
Pushed to: origin/master
```

### Vercel Deployment
- **Status:** Building/Deploying (check Vercel dashboard)
- **URL:** https://agenticaibook-seven.vercel.app
- **Build Command:** `bash vercel_build.sh`
- **Install Command:** `bash vercel_install.sh`

---

## ✅ Next Steps (CRITICAL)

### Step 1: Set Environment Variables in Vercel

**This is REQUIRED for the chatbot to work!**

1. Go to: https://vercel.com/dashboard
2. Click on your project: **agent-book-factory**
3. Go to **Settings** → **Environment Variables**
4. Add these variables (click "Add New" for each):

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

5. Select **Production** and **Preview** for each
6. Save all variables

**Get FREE API keys here:**
- Groq: https://console.groq.com (FREE LLM)
- HuggingFace: https://huggingface.co/settings/tokens (FREE embeddings)
- Qdrant Cloud: https://cloud.qdrant.io (FREE 1GB vector DB)
- Neon: https://neon.tech (FREE PostgreSQL)

See `ENV_SETUP_GUIDE.md` for detailed instructions.

---

### Step 2: Redeploy After Setting Variables

After setting environment variables:

```bash
# Trigger new deployment
vercel --prod

# Or wait for Git deployment to complete
```

---

### Step 3: Verify Deployment

#### Test API Health Check
```bash
curl https://agenticaibook-seven.vercel.app/api/health
```

**Expected Response:**
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

#### Test AI Tutor Widget

1. Open: https://agenticaibook-seven.vercel.app
2. Open browser DevTools (F12)
3. Check **Console** tab for `[AI Tutor]` messages
4. Look for **purple floating button** in bottom-right
5. Click button to open chat
6. Ask: "What is ROS 2?"

**Expected:** Chat response with citations from textbook

---

## 🔍 Debug Checklist

### If API Still Returns 500 Error

- [ ] Check Vercel Dashboard → Settings → Environment Variables
- [ ] Verify all 7 variables are set for **Production**
- [ ] Check Vercel Functions logs: `vercel logs <deployment-url>`
- [ ] Look for "Import error" or "Missing variable" messages
- [ ] Redeploy: `vercel --prod`

### If Widget Still Not Visible

- [ ] Open browser DevTools (F12)
- [ ] Check **Console** for `[AI Tutor]` logs
- [ ] Check **Console** for JavaScript errors
- [ ] Check **Elements** tab for `.ai-tutor-fab` element
- [ ] Inspect element styles (position, z-index, display)
- [ ] Verify `custom.css` is loaded (check Network tab)
- [ ] Clear browser cache and reload

### If Chat Returns Errors

- [ ] Verify Groq API key is valid: https://console.groq.com
- [ ] Check Qdrant cluster is active: https://cloud.qdrant.io
- [ ] Test Groq API separately with curl
- [ ] Check Vercel function logs for API errors

---

## 📊 Success Criteria

| Criterion | Status | How to Verify |
|-----------|--------|---------------|
| `/api/health` returns 200 | ⏳ Pending | `curl https://agenticaibook-seven.vercel.app/api/health` |
| AI Tutor button visible | ⏳ Pending | Look for purple button in bottom-right |
| Chat widget opens on click | ⏳ Pending | Click button, widget should appear |
| Chat responds to questions | ⏳ Pending | Ask "What is ROS 2?" |
| Responses include citations | ⏳ Pending | Check for chapter/section links |

---

## 🎯 Summary

### What Was Fixed ✅

1. **API Import Paths:** `api/index.py` now handles multiple path strategies for serverless
2. **Root.tsx Typing:** Proper TypeScript interface for Docusaurus theme override
3. **Client-Side Debug:** Added `clientModules.js` for browser-side troubleshooting
4. **Documentation:** Created comprehensive `ENV_SETUP_GUIDE.md`

### What's Still Needed ⚠️

1. **Environment Variables:** YOU MUST SET THESE IN VERCEL DASHBOARD
2. **Content Indexing:** Run `rebuild_embeddings.py` after deployment (optional, for chat to work)

### Expected Outcome 🎯

After setting environment variables and redeploying:
- ✅ API health check returns 200 OK
- ✅ AI Tutor floating button visible
- ✅ Chat responds with textbook-based answers
- ✅ All responses include citations

---

## 📞 Support Resources

- **Environment Setup:** See `ENV_SETUP_GUIDE.md`
- **FREE API Keys:** See `FREE_API_SETUP.md`
- **Vercel Logs:** `vercel logs <deployment-url>`
- **GitHub Repo:** https://github.com/M-DaniyalHS1/Agentic_AI_Book
- **Vercel Dashboard:** https://vercel.com/dashboard

---

**Report Generated:** 2026-03-09  
**Author:** AI Assistant  
**Next Action:** Set environment variables in Vercel Dashboard
