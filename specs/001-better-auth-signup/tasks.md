# Tasks: Better Auth Signup and Signin with User Background Collection

**Input**: Design documents from `specs/001-better-auth-signup/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/auth-api.md

**Tests**: Tests are OPTIONAL - included here for completeness. Remove test tasks if TDD is not required.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

**Existing Project Structure** (FastAPI Backend + Docusaurus Frontend):
- `backend/` - FastAPI Python backend
- `backend/src/` - Backend source code
- `backend/prisma/` - Database schema and migrations
- `frontend/` - Docusaurus frontend
- `frontend/src/` - Frontend React components
- `tests/` - Test files at repository root

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 [P] Verify existing project structure (FastAPI + Docusaurus)
- [X] T002 [P] Install Better Auth Python SDK in backend: `pip install better-auth-python`
- [X] T003 [P] Install Prisma and @prisma/client in backend: `pip install prisma`
- [X] T004 [P] Install bcrypt, python-jose, and aiosmtplib in backend: `pip install bcrypt python-jose[cryptography] aiosmtplib`
- [X] T005 [P] Install pytest and pytest-asyncio dev dependencies (already in requirements.txt)
- [X] T006 [P] Verify existing ESLint/Prettier config in frontend (Docusaurus default)
- [X] T007 [P] Update .env.example with auth environment variables (DATABASE_URL, BETTER_AUTH_SECRET, SMTP_*)
- [X] T008 [P] Verify .gitignore has node_modules, .env, .vercel, __pycache__ entries

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T009 Create Neon Serverless Postgres database and configure DATABASE_URL
- [X] T010 [P] Initialize Prisma schema in backend/prisma/schema.prisma
- [X] T011 [P] Create User model in backend/prisma/schema.prisma (id, email, passwordHash, emailVerified, timestamps)
- [X] T012 [P] Create UserProfile model in backend/prisma/schema.prisma (userId FK, softwareLevel, learningGoal, hardwareAccess, technicalComfort)
- [X] T013 [P] Create Session model in backend/prisma/schema.prisma (userId FK, token, expiresAt, timestamps)
- [X] T014 [P] Add indexes to backend/prisma/schema.prisma (email, userId, token, expiresAt)
- [ ] T015 Run initial Prisma migration: `prisma migrate dev --name init_auth`
- [X] T016 [P] Create backend/src/db/prisma.py - Prisma client singleton
- [X] T017 [P] Create backend/src/auth/auth.py - Better Auth configuration with Prisma adapter
- [X] T018 [P] Create backend/src/auth/validations.py - Pydantic schemas (signup, signin, profile)
- [X] T019 [P] Create backend/api/auth.py - FastAPI router for auth endpoints
- [X] T020 [P] Setup Gmail SMTP configuration in backend/src/services/email.py (aiosmtplib)
- [X] T021 [P] Create base error handling utility in backend/src/auth/errors.py
- [X] T022 [P] Configure Vercel deployment settings for backend (vercel.json already exists)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - New User Registration (Priority: P1) 🎯 MVP

**Goal**: Enable new users to create accounts with credentials and complete background questionnaire

**Independent Test**: A new user can complete the full registration flow (email/password + 4 background questions) and have their account + profile saved to database

### Tests for User Story 1 (OPTIONAL) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T023 [P] [US1] Create E2E test for signup flow in tests/integration/signup-flow.test.ts (Playwright)
- [ ] T024 [P] [US1] Create contract test for POST /api/auth/signup in tests/contract/auth-contract.test.ts (pytest)
- [ ] T025 [P] [US1] Create validation test for signup schema in tests/unit/validations.test.py

### Implementation for User Story 1

- [X] T026 [P] [US1] Create backend/api/auth.py - POST /signup handler with profile creation
- [X] T027 [P] [US1] Create frontend/src/components/auth/SignupForm.tsx - Two-step form (credentials → background questions)
- [X] T028 [P] [US1] Create frontend/src/components/auth/BackgroundQuestions.tsx - 4-question component
- [X] T029 [US1] Create frontend/src/pages/signup.tsx - Signup page wrapping SignupForm component
- [X] T030 [US1] Implement password hashing with bcrypt in signup API route
- [X] T031 [US1] Add email format validation and duplicate email check in signup API
- [X] T032 [US1] Add password minimum 6 characters validation
- [X] T033 [US1] Implement transactional user + profile creation in Prisma
- [X] T034 [US1] Add error responses (USER_EXISTS, INVALID_INPUT, SERVER_ERROR)
- [X] T035 [US1] Add logging for signup events (success/failure)
- [X] T036 [US1] Create frontend/src/pages/dashboard.tsx - Post-signup redirect page
- [X] T037 [US1] Add rate limiting to signup endpoint (5 requests per minute)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently
- Users can signup with email/password
- Background questionnaire collects 4 responses
- User + Profile saved to database
- Session created automatically

---

## Phase 4: User Story 2 - Existing User Signin (Priority: P2)

**Goal**: Enable registered users to authenticate and maintain sessions

**Independent Test**: An existing user can sign in with valid credentials and maintain authenticated session across page navigations

### Tests for User Story 2 (OPTIONAL) ⚠️

- [ ] T038 [P] [US2] Create E2E test for signin flow in tests/integration/signin-flow.test.ts (Playwright)
- [ ] T039 [P] [US2] Create contract test for POST /api/auth/signin in tests/contract/auth-contract.test.ts
- [ ] T040 [P] [US2] Create session management test in tests/unit/session.test.py

### Implementation for User Story 2

- [X] T041 [P] [US2] Create backend/api/auth.py - POST /signin handler
- [X] T042 [P] [US2] Create frontend/src/components/auth/SigninForm.tsx - Email/password form component
- [X] T043 [US2] Create frontend/src/pages/signin.tsx - Signin page wrapping SigninForm component
- [X] T044 [US2] Implement password verification with bcrypt in signin API
- [X] T045 [US2] Add generic error message for invalid credentials (prevent email enumeration)
- [X] T046 [US2] Configure session creation with 7-day expiration in Better Auth
- [X] T047 [US2] Create backend/middleware/auth.py - Session validation middleware
- [X] T048 [US2] Implement HTTP-only cookie handling for session token
- [X] T049 [US2] Add session refresh logic (updateAge: 24 hours)
- [X] T050 [US2] Add logging for signin events (success/failure)
- [X] T051 [US2] Add rate limiting to signin endpoint (5 requests per minute)
- [X] T052 [P] [US2] Create frontend/src/components/auth/SignOutButton.tsx - Signout functionality
- [X] T053 [US2] Create backend/api/auth.py - POST /signout handler

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently
- Users can signup (US1)
- Users can signin with existing credentials (US2)
- Sessions persist across page navigations
- Users can signout

---

## Phase 5: User Story 3 - Background Data Collection (Priority: P3)

**Goal**: Store and retrieve user background data for future personalization (personalization logic deferred)

**Independent Test**: User background responses are persisted in database and can be retrieved/viewed by authenticated users

### Tests for User Story 3 (OPTIONAL) ⚠️

- [ ] T054 [P] [US3] Create E2E test for profile view in tests/integration/profile-view.test.ts (Playwright)
- [ ] T055 [P] [US3] Create contract test for GET/PUT /api/auth/profile in tests/contract/profile-contract.test.ts
- [ ] T056 [P] [US3] Create integration test for background data persistence in tests/integration/background-data.test.ts

### Implementation for User Story 3

- [X] T057 [P] [US3] Create backend/api/auth.py - GET handler for user profile
- [X] T058 [P] [US3] Create backend/api/auth.py - PUT handler for profile update
- [X] T059 [P] [US3] Create frontend/src/components/auth/ProfileView.tsx - Display user background data
- [X] T060 [P] [US3] Create frontend/src/components/auth/ProfileForm.tsx - Editable profile form
- [X] T061 [US3] Create frontend/src/pages/profile.tsx - Profile page component
- [X] T062 [US3] Add authentication check to profile endpoints (require valid session)
- [X] T063 [US3] Implement profile update with validation (Pydantic profileSchema)
- [X] T064 [US3] Add enum validation for softwareLevel, hardwareAccess, technicalComfort
- [X] T065 [US3] Add learningGoal max 200 characters validation
- [ ] T066 [US3] Implement cascade delete (profile deleted when user deleted)
- [X] T067 [US3] Add logging for profile access/update events
- [ ] T068 [US3] Create seed script for test users in backend/prisma/seed.py (beginner/intermediate/advanced profiles)

**Checkpoint**: All user stories should now be independently functional
- Users can signup with background questions (US1)
- Users can signin/signout (US2)
- Users can view and update their background profile (US3)
- Personalization logic explicitly NOT implemented (deferred)

---

## Phase 6: User Story 4 - Password Recovery (Priority: P4)

**Goal**: Enable users to securely reset forgotten passwords via email

**Independent Test**: A user can request password reset, receive email with reset link, and set a new password successfully

### Tests for User Story 4 (OPTIONAL) ⚠️

- [ ] T069 [P] [US4] Create E2E test for password reset flow in tests/integration/password-reset.test.ts (Playwright)
- [ ] T070 [P] [US4] Create contract test for POST /api/auth/reset-password in tests/contract/auth-contract.test.ts
- [ ] T071 [P] [US4] Create email delivery test in tests/unit/email-service.test.py

### Implementation for User Story 4

- [X] T072 [P] [US4] Create backend/api/auth.py - POST handler for reset request
- [X] T073 [P] [US4] Create backend/api/auth.py - POST handler for reset confirmation
- [X] T074 [P] [US4] Create frontend/src/components/auth/ResetPasswordRequest.tsx - Email input form
- [X] T075 [P] [US4] Create frontend/src/components/auth/ResetPasswordConfirm.tsx - New password form with token
- [X] T076 [US4] Create frontend/src/pages/reset-password.tsx - Reset request page
- [X] T077 [US4] Create frontend/src/pages/reset-password/confirm.tsx - Reset confirmation page
- [X] T078 [US4] Implement password reset email with Gmail SMTP (aiosmtplib)
- [ ] T079 [US4] Create reset password email template in backend/src/templates/reset_password.html
- [X] T080 [US4] Implement single-use reset token generation (secrets.token_urlsafe)
- [X] T081 [US4] Add token expiration (1 hour from generation)
- [X] T082 [US4] Implement token validation in reset confirmation endpoint
- [X] T083 [US4] Add password hashing for new password (bcrypt)
- [X] T084 [US4] Invalidate all existing sessions on password reset (security)
- [X] T085 [US4] Add generic success message for reset request (prevent email enumeration)
- [X] T086 [US4] Add rate limiting to reset-password endpoint (3 requests per hour)
- [X] T087 [US4] Add logging for password reset events

**Checkpoint**: All 4 user stories are now complete and independently functional
- Full authentication system operational
- Password recovery working end-to-end
- All security measures in place

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T088 [P] Create comprehensive README.md with setup instructions from quickstart.md
- [ ] T089 [P] Create DEPLOYMENT.md with Vercel + Neon deployment guide
- [ ] T090 [P] Add TypeScript strict mode configuration in frontend/tsconfig.json
- [ ] T091 Code cleanup - remove unused imports, consolidate utilities
- [ ] T092 [P] Add comprehensive unit tests in tests/unit/ for all service functions
- [ ] T093 [P] Add accessibility testing (ARIA labels, keyboard navigation)
- [ ] T094 [P] Performance optimization - add database connection pooling
- [ ] T095 [P] Security hardening - add CSRF protection headers
- [ ] T096 [P] Add Sentry error tracking integration (optional monitoring)
- [ ] T097 [P] Run quickstart.md validation - verify all 10 steps work end-to-end
- [ ] T098 [P] Create API documentation from contracts/auth-api.md
- [ ] T099 [P] Add database backup strategy documentation
- [ ] T100 Final validation - run all E2E tests with Playwright

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - **BLOCKS all user stories**
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3 → P4)
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Independent of US1 but shares foundation
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Independent of US1/US2
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - Independent of other stories

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- API routes before UI components
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

**Phase 1 (Setup)**: All tasks T001-T008 marked [P] can run in parallel

**Phase 2 (Foundational)**:
- T010-T014 [P] - All Prisma model creation can run in parallel
- T016-T022 [P] - All library/utility creation can run in parallel

**Phase 3 (US1)**:
- T023-T025 [P] - All tests can run in parallel
- T026-T028 [P] - API route + components can run in parallel (different files)

**Phase 4 (US2)**:
- T038-T040 [P] - All tests can run in parallel
- T041-T043 [P] - API route + components can run in parallel

**Phase 5 (US3)**:
- T054-T056 [P] - All tests can run in parallel
- T057-T060 [P] - All API routes + components can run in parallel

**Phase 6 (US4)**:
- T069-T071 [P] - All tests can run in parallel
- T072-T075 [P] - All API routes + components can run in parallel

**Cross-Story Parallel**:
- Once Phase 2 completes, different developers can work on US1, US2, US3, US4 simultaneously

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together:
Task: "Create E2E test for signup flow in tests/integration/signup-flow.test.ts"
Task: "Create contract test for POST /api/auth/signup in tests/contract/auth-contract.test.ts"
Task: "Create validation test for signup schema in tests/unit/validations.test.py"

# Launch all parallelizable implementation for User Story 1:
Task: "Create backend/api/auth.py - POST /signup handler"
Task: "Create frontend/src/components/auth/SignupForm.tsx - Two-step form"
Task: "Create frontend/src/components/auth/BackgroundQuestions.tsx - 4-question component"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T008)
2. Complete Phase 2: Foundational (T009-T022) - **CRITICAL - blocks all stories**
3. Complete Phase 3: User Story 1 (T023-T037)
4. **STOP and VALIDATE**:
   - Test signup flow end-to-end
   - Verify background data saved to database
   - Test validation errors
5. Deploy MVP to Vercel if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (signup + background)
   - Developer B: User Story 2 (signin + signout)
   - Developer C: User Story 3 (profile view/edit)
   - Developer D: User Story 4 (password recovery)
3. Stories complete and integrate independently
4. Reconvene for Phase 7: Polish

---

## Task Summary

| Phase | Description | Task Count | Testable MVP |
|-------|-------------|------------|--------------|
| Phase 1 | Setup | 8 tasks | No |
| Phase 2 | Foundational | 14 tasks | No |
| Phase 3 | User Story 1 (Signup) | 15 tasks (3 tests + 12 impl) | **YES - MVP** |
| Phase 4 | User Story 2 (Signin) | 16 tasks (3 tests + 13 impl) | YES |
| Phase 5 | User Story 3 (Profile) | 15 tasks (3 tests + 12 impl) | YES |
| Phase 6 | User Story 4 (Reset) | 19 tasks (3 tests + 16 impl) | YES |
| Phase 7 | Polish | 13 tasks | N/A |
| **Total** | **All Phases** | **100 tasks** | |

**MVP Scope** (User Story 1 only): 37 tasks (Phases 1-3)
**Full Feature** (All stories): 100 tasks (Phases 1-7)

---

## Notes

- [P] tasks = different files, no dependencies within phase
- [Story] label maps task to specific user story for traceability
- Each user story is independently completable and testable
- Tests are OPTIONAL - remove test tasks if TDD not required
- Commit after each task or logical group
- Stop at checkpoints to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- **File paths are absolute from repository root**
- **Architecture adapted for FastAPI backend + Docusaurus frontend**
