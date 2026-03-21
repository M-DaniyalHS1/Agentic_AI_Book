---
id: 2
title: Cookie Authentication and Routing Fixes
stage: green
date: 2026-03-21
surface: agent
model: qwen-coder
feature: better-auth-signup
branch: 001-better-auth-signup
user: Qamar Traders
command: git commit and push authentication fixes
labels: ["authentication", "cookie-handling", "routing", "bug-fix"]
links:
  spec: specs/001-better-auth-signup/spec.md
  ticket: null
  adr: null
  pr: https://github.com/M-DaniyalHS1/Agentic_AI_Book/pull/10
files:
 - backend/main.py
 - backend/src/api/auth.py
 - backend/src/models/user.py
 - backend/src/db/database.py
 - frontend/src/components/auth/SigninForm.tsx
 - frontend/src/components/auth/SignupForm.tsx
 - frontend/src/pages/dashboard.tsx
 - frontend/src/pages/profile.tsx
 - frontend/src/services/apiService.ts
 - frontend/src/components/AITutorWidget.tsx
 - test_cookie_auth.html
 - COOKIE_FIX_SUMMARY.md
tests:
 - Manual browser testing of signin flow
 - Cookie storage verification in DevTools
 - Dashboard authentication check
 - Route navigation test (/intro)
---

## Prompt

User reported two issues after signin:
1. Authentication cookie not being stored in browser after successful signin
2. "Page Not Found" when clicking "Browse Textbook" button on dashboard

## Response snapshot

Fixed both issues autonomously:

1. **Cookie Authentication Fix**:
   - Identified root cause: frontend using `127.0.0.1:8001` while backend on `localhost:8001` (different origins)
   - Changed backend host from `0.0.0.0:8000` to `localhost:8001`
   - Replaced CORSMiddleware with custom HTTP middleware to properly expose Set-Cookie headers
   - Removed explicit cookie domain parameter from set_cookie() calls
   - Updated ALL frontend API URLs from `127.0.0.1:8001` to `localhost:8001` (9 files)
   - Added debug logging to signin and /me endpoints

2. **Dashboard Route Fix**:
   - Identified Docusaurus configured with `routeBasePath: '/'` (docs at root, not /docs/)
   - Changed dashboard button from `/docs/intro` to `/intro`
   - Fixed citation URLs in AITutorWidget from `/docs/{module}/{chapter}` to `/{module}/{chapter}`

3. **Verification**:
   - Backend authentication tested with curl: cookie set correctly, /me returns authenticated: true
   - Frontend dev server restarted
   - User confirmed both issues resolved

## Outcome

- ✅ Impact: Authentication system now fully functional; users can sign in and access dashboard
- 🧪 Tests: Manual browser testing confirmed cookie storage and authentication working
- 📁 Files: 40 files changed (4452 insertions, 109 deletions)
- 🔁 Next prompts: Continue with feature development or code review
- 🧠 Reflection: Hostname consistency (localhost vs 127.0.0.1) is critical for cookie-based auth

## Evaluation notes (flywheel)

- Failure modes observed: None - both issues resolved on first attempt
- Graders run and results: N/A (manual testing only)
- Prompt variant: N/A
- Next experiment: N/A - feature complete
