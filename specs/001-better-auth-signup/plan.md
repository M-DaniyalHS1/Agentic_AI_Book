# Implementation Plan: Better Auth Signup and Signin with User Background Collection

**Branch**: `001-better-auth-signup` | **Date**: 2026-03-12 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `specs/001-better-auth-signup/spec.md`

## Summary

Implement user authentication system using Better Auth library with email/password signup, signin, and password reset flows. The system collects minimal user background data (4 questions: 2 software, 2 hardware) during signup for future personalization features. Personalization logic is explicitly deferred to a future phase.

## Technical Context

**Language/Version**: TypeScript 5.x, Node.js 20.x
**Primary Dependencies**: Better Auth (authentication), Next.js 14+ (framework), Prisma (ORM)
**Storage**: Neon Serverless PostgreSQL (user accounts, profiles, sessions)
**Testing**: Vitest (unit), Playwright (E2E)
**Target Platform**: Web application (responsive)
**Performance Goals**: 500 concurrent users, p95 latency <200ms for auth operations
**Constraints**: Signup flow <5 minutes, 85% questionnaire completion rate
**Scale/Scope**: MVP authentication system with background data collection
**Deployment**: Vercel (zero-config Next.js hosting)
**Email**: Gmail SMTP (free tier with app password)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| AI-Native First | ✅ Pass | Background data structured for ML queries; User profiles machine-readable |
| Open Knowledge & Accessibility | ✅ Pass | Authentication accessible via standard web protocols; JSON APIs |
| Embodied Intelligence Focus | ⚠️ N/A | Authentication is infrastructure; no direct robotics embodiment |
| Reproducibility & Simulation-First | ✅ Pass | All auth flows testable via Playwright; seed data for testing |
| Modular & Extensible Design | ✅ Pass | Better Auth modular; background schema extensible for future personalization |
| Quality Education Standards | ✅ Pass | Clear user flows; documented API contracts; version-controlled migrations |

**GATE RESULT**: PASS (N/A for embodiment principle as this is infrastructure)

## Project Structure

### Documentation (this feature)

```text
specs/001-better-auth-signup/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
└── tasks.md             # Phase 2 output
```

### Source Code (repository root)

```text
src/
├── lib/
│   └── auth.ts          # Better Auth configuration
├── models/
│   ├── user.ts          # User account schema
│   └── profile.ts       # User background profile
├── services/
│   ├── auth-service.ts  # Auth business logic
│   └── email-service.ts # Password reset emails
├── app/
│   ├── (auth)/
│   │   ├── signup/
│   │   ├── signin/
│   │   └── reset-password/
│   └── api/
│       └── auth/        # Auth API routes
└── components/
    ├── auth/
    │   ├── signup-form.tsx
    │   ├── signin-form.tsx
    │   └── background-questions.tsx
    └── ui/              # Reusable form components

tests/
├── contract/
│   └── auth-contract.test.ts
├── integration/
│   ├── auth-flow.test.ts
│   └── background-data.test.ts
└── unit/
    ├── auth-service.test.ts
    └── email-service.test.ts
```

**Structure Decision**: Single project structure (Option 1) with Next.js App Router convention. Auth feature isolated under `(auth)` route group for clear boundaries.

## Complexity Tracking

No constitution violations requiring justification.

---

## Phase Completion Status

### Phase 0: Research ✅ COMPLETE

- [x] `research.md` created with all technology decisions
- [x] All NEEDS CLARIFICATION items resolved
- [x] Best practices documented for Better Auth, Prisma, PostgreSQL

### Phase 1: Design & Contracts ✅ COMPLETE

- [x] `data-model.md` - Entity schemas, relationships, validation rules
- [x] `contracts/auth-api.md` - API endpoint specifications
- [x] `quickstart.md` - Step-by-step implementation guide
- [ ] Agent context update - Skipped (no existing Qwen agent context file found)

### Phase 2: Tasks

- [ ] `tasks.md` - To be created by `/sp.tasks` command

---

## Generated Artifacts

| File | Purpose | Status |
|------|---------|--------|
| `plan.md` | Technical architecture overview | ✅ Complete |
| `research.md` | Technology decisions & rationale | ✅ Complete |
| `data-model.md` | Database schema & entities | ✅ Complete |
| `contracts/auth-api.md` | API specifications | ✅ Complete |
| `quickstart.md` | Implementation guide | ✅ Complete |
| `tasks.md` | Task breakdown | ⏳ Pending `/sp.tasks` |

---

## Next Steps

**Ready for task breakdown**. Run `/sp.tasks` to create actionable implementation tasks.

**Production Stack** (All Resolved):
1. ✅ Email provider: Gmail SMTP (free tier with app password)
2. ✅ Database hosting: Neon Serverless Postgres (free tier)
3. ✅ Deployment platform: Vercel (zero-config Next.js)
4. ⏳ Monitoring: Defer to post-MVP (Sentry recommended)
