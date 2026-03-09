# ✅ Deployment Checklist - AI Tutor RAG Chatbot

**Deployment URL:** https://agenticaibook-seven.vercel.app  
**Date:** 2026-03-09

---

## 🎯 Critical Actions Required

### ⚠️ STEP 1: Set Environment Variables in Vercel (REQUIRED)

Your chatbot **WILL NOT WORK** without these. Go to Vercel Dashboard now:

**URL:** https://vercel.com/dashboard → Select your project → Settings → Environment Variables

**Add these 7 variables (click "Add New" for each):**

```env
# 1. Groq API Key (FREE - get at https://console.groq.com)
GROQ_API_KEY=gsk_your-actual-key-here

# 2. Groq Model
GROQ_MODEL=llama3-8b-8192

# 3. HuggingFace API Key (FREE - get at https://huggingface.co/settings/tokens)
HUGGINGFACE_API_KEY=hf_your-actual-token-here

# 4. HuggingFace Embedding Model
HUGGINGFACE_EMBEDDING_MODEL=sentence-transformers/all-MiniLM-L6-v2

# 5. Qdrant Cloud URL (FREE - get at https://cloud.qdrant.io)
QDRANT_URL=https://your-cluster.qdrant.tech

# 6. Qdrant API Key
QDRANT_API_KEY=your-actual-api-key

# 7. Database URL (FREE - get at https://neon.tech)
DATABASE_URL=postgresql://user:password@host:port/database

# 8. CORS Origins
ALLOWED_ORIGINS=http://localhost:3000,https://agenticaibook-seven.vercel.app

# 9. LLM Provider
LLM_PROVIDER=groq
```

**Important:**
- Select **Production** ✅ for each variable
- Click **Save** after adding each one
- **Do not commit these to Git** (they're secrets!)

**Get FREE API Keys:**
- Groq: https://console.groq.com (2 minutes signup)
- HuggingFace: https://huggingface.co/settings/tokens (2 minutes)
- Qdrant Cloud: https://cloud.qdrant.io (3 minutes)
- Neon: https://neon.tech (2 minutes)

**Detailed Guide:** See `ENV_SETUP_GUIDE.md` or `FREE_API_SETUP.md`

---

### ⏳ STEP 2: Wait for Vercel Deployment

After pushing code, Vercel automatically builds and deploys:

**Check Deployment Status:**
1. Go to: https://vercel.com/dashboard
2. Look for your project: **agent-book-factory**
3. Check deployment status (Building → Ready)

**Or use CLI:**
```bash
vercel ls
```

---

### ✅ STEP 3: Verify Deployment

After deployment shows "Ready":

#### Test 1: API Health Check
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

❌ **If you get 500 error:** Environment variables not set correctly. Go back to STEP 1.

---

#### Test 2: AI Tutor Widget Visibility

1. Open: https://agenticaibook-seven.vercel.app
2. Look at **bottom-right corner**
3. You should see a **purple circular button** with chat icon

**Expected:**
- ✅ Purple floating button visible
- ✅ Button has chat bubble icon
- ✅ Button is in bottom-right corner

❌ **If button not visible:**
- Open browser DevTools (F12)
- Check Console for `[AI Tutor]` messages
- Look for JavaScript errors
- See "Widget Debugging" section below

---

#### Test 3: Chat Functionality

1. **Click** the purple AI Tutor button
2. Chat widget should open
3. Type: "What is ROS 2?"
4. Press Enter or click Send

**Expected:**
- ✅ Loading animation appears
- ✅ Response appears after 2-5 seconds
- ✅ Response cites textbook chapters/sections
- ✅ Citations are clickable links

❌ **If chat returns error:**
- Check Vercel Function Logs (see below)
- Verify Groq API key is correct
- Verify Qdrant has indexed content

---

## 🔍 Debug Tools

### Check Vercel Function Logs

```bash
# Get recent logs
vercel logs agenticaibook-seven.vercel.app

# Or use Vercel Dashboard:
# Project → Deployments → Click latest → Function logs
```

**Look for:**
- `✓ Successfully imported main.app`
- `✓ Mangum handler created successfully`
- `=== Python Path Setup ===`

**Errors to watch for:**
- `ImportError: No module named...` → Path issue
- `ValueError: GROQ_API_KEY is required` → Missing env var
- `QDRANT_URL environment variable is required` → Missing env var

---

### Widget Debugging

**Open Browser DevTools (F12):**

1. **Console Tab:**
   - Look for `[AI Tutor]` messages
   - Check for JavaScript errors (red text)
   - Expected logs:
     ```
     [AI Tutor] Initializing AI Tutor Widget...
     [AI Tutor] DOM ready, checking for widget...
     [AI Tutor] ✓ Widget found in DOM
     [AI Tutor] ✓ Widget styles verified
     ```

2. **Elements Tab:**
   - Search for: `.ai-tutor-fab`
   - Should find: `<button class="ai-tutor-fab">`
   - Check computed styles:
     - `position: fixed`
     - `bottom: 24px`
     - `right: 24px`
     - `z-index: 1000`
     - `display: flex`

3. **Network Tab:**
   - Reload page
   - Check if `clientModules.js` loads (status 200)
   - Check if `custom.css` loads

---

### Test API Endpoints Manually

```bash
# Health check
curl https://agenticaibook-seven.vercel.app/api/health

# Content modules
curl https://agenticaibook-seven.vercel.app/api/content/modules

# Chat (requires POST with JSON)
curl -X POST https://agenticaibook-seven.vercel.app/api/tutor/chat \
  -H "Content-Type: application/json" \
  -d '{"query":"What is ROS 2?"}'
```

---

## 📋 Pre-Deployment Checklist

Before deploying, verify:

- [ ] `api/index.py` has multi-strategy path resolution
- [ ] `frontend/src/theme/Root.tsx` has TypeScript interface
- [ ] `frontend/docusaurus.config.js` includes clientModules.js
- [ ] All environment variables are set in Vercel
- [ ] Git push completed successfully
- [ ] Vercel deployment shows "Ready"

---

## 🎯 Success Criteria

| Test | Expected Result | Status |
|------|-----------------|--------|
| `/api/health` | Returns 200 OK with `{"status":"healthy"}` | ⏳ |
| AI Tutor button | Visible in bottom-right (purple) | ⏳ |
| Chat widget | Opens when button clicked | ⏳ |
| Chat response | Answers question about ROS 2 | ⏳ |
| Citations | Shows chapter/section references | ⏳ |

---

## 🆘 Common Issues & Fixes

### Issue: "FUNCTION_INVOCATION_FAILED" on /api/health

**Cause:** Missing environment variables

**Fix:**
1. Go to Vercel Dashboard → Settings → Environment Variables
2. Add all 7+ variables listed in STEP 1
3. Redeploy: `vercel --prod`

---

### Issue: Widget not visible, no console errors

**Cause:** Root.tsx not loaded or CSS issue

**Fix:**
1. Check if `clientModules.js` is in page source
2. Verify `custom.css` imports `AITutorWidget.css`
3. Clear browser cache (Ctrl+Shift+Delete)
4. Hard reload (Ctrl+F5)

---

### Issue: Widget visible but chat returns errors

**Cause:** API keys invalid or content not indexed

**Fix:**
1. Verify Groq API key: https://console.groq.com
2. Verify Qdrant cluster active: https://cloud.qdrant.io
3. Run content indexer locally:
   ```bash
   cd backend
   python scripts/rebuild_embeddings.py
   ```
4. Check Vercel function logs for details

---

### Issue: "This is not covered in the book yet"

**Cause:** Qdrant has no indexed content

**Fix:**
1. Run content indexer:
   ```bash
   cd backend
   python scripts/rebuild_embeddings.py
   ```
2. Verify Qdrant Cloud has collection created
3. Check Qdrant dashboard for document count

---

## 📞 Resources

- **Environment Setup Guide:** `ENV_SETUP_GUIDE.md`
- **FREE API Setup:** `FREE_API_SETUP.md`
- **Debug Report:** `DEBUG_REPORT.md`
- **Vercel Dashboard:** https://vercel.com/dashboard
- **GitHub Repo:** https://github.com/M-DaniyalHS1/Agentic_AI_Book

---

## ✅ Final Verification

After completing all steps:

- [ ] Environment variables set in Vercel
- [ ] Deployment shows "Ready"
- [ ] `/api/health` returns 200 OK
- [ ] AI Tutor button visible
- [ ] Chat widget opens
- [ ] Chat responds with citations
- [ ] No console errors

**If all checked:** 🎉 **SUCCESS!** Your RAG chatbot is working!

---

**Last Updated:** 2026-03-09  
**Deployment:** https://agenticaibook-seven.vercel.app  
**Status:** ⏳ Awaiting Environment Variables
