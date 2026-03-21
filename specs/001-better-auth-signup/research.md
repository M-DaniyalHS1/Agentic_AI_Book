# Research: Better Auth Authentication System

**Date**: 2026-03-12
**Purpose**: Resolve technical unknowns and establish best practices for authentication implementation

---

## Decision 1: Authentication Library Selection

**Decision**: Use Better Auth (https://www.better-auth.com/)

**Rationale**: 
- User explicitly requested Better Auth in feature specification
- Modern TypeScript-first authentication framework
- Built-in support for email/password, sessions, password reset
- Integrates well with Next.js App Router
- Provides secure defaults (password hashing, session management)

**Alternatives Considered**:
- NextAuth.js: More mature but heavier; Better Auth is lighter weight
- Auth.js: Good but requires more configuration
- Custom implementation: Security risks; reinventing wheel

---

## Decision 2: Database Schema Design

**Decision**: Separate User and UserProfile tables with 1:1 relationship

**Rationale**:
- Clean separation of concerns (auth credentials vs. background data)
- Easier to extend profile schema without touching auth tables
- Supports future personalization features without migration complexity
- Follows Better Auth's recommended patterns

**Schema Structure**:
```prisma
model User {
  id            String    @id @default(cuid())
  email         String    @unique
  passwordHash  String
  emailVerified DateTime?
  createdAt     DateTime  @default(now())
  updatedAt     DateTime  @updatedAt
  profile       UserProfile?
  sessions      Session[]
}

model UserProfile {
  id                String  @id @default(cuid())
  userId            String  @unique
  user              User    @relation(fields: [userId], references: [id], onDelete: Cascade)
  softwareLevel     String  // beginner, intermediate, advanced
  learningGoal      String
  hardwareAccess    String  // none, basic, advanced
  technicalComfort  String  // low, medium, high
  createdAt         DateTime @default(now())
  updatedAt         DateTime @updatedAt
}

model Session {
  id        String   @id @default(cuid())
  userId    String
  user      User     @relation(fields: [userId], references: [id], onDelete: Cascade)
  expiresAt DateTime
  token     String   @unique
}
```

**Alternatives Considered**:
- Single table with all fields: Simpler but mixes auth + domain data
- JSON field for profile: Flexible but loses type safety and queryability

---

## Decision 3: Password Security Standards

**Decision**: Minimum 6 characters (per spec clarification), bcrypt hashing

**Rationale**:
- Aligns with user requirement (6+ characters)
- bcrypt is industry standard, automatically handles salting
- Better Auth uses bcrypt by default
- Cost factor 10 balances security vs. performance

**Implementation**:
```typescript
import { hash, verify } from 'bcryptjs';

const passwordHash = await hash(password, 10);
const isValid = await verify(password, passwordHash);
```

**Alternatives Considered**:
- Argon2: More secure but less battle-tested in Node.js
- PBKDF2: Slower; bcrypt preferred for web apps

---

## Decision 4: Session Management Strategy

**Decision**: Token-based sessions with 7-day inactivity timeout

**Rationale**:
- Meets spec requirement (SC-007: 7-day timeout)
- Better Auth handles session creation/validation automatically
- Database-backed sessions enable server-side invalidation
- Token stored in HTTP-only cookie for security

**Session Configuration**:
```typescript
export const auth = betterAuth({
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24,     // Refresh every 24h
    cookieCache: {
      enabled: true,
      maxAge: 60 * 5             // 5-minute cache
    }
  }
});
```

**Alternatives Considered**:
- JWT stateless sessions: Harder to invalidate; DB sessions preferred
- Redis sessions: Adds infrastructure complexity; PostgreSQL sufficient for 500 users

---

## Decision 5: Email Delivery for Password Reset

**Decision**: Use Nodemailer with SMTP (environment-configured provider)

**Rationale**:
- SC-005 requires delivery within 60 seconds
- Nodemailer is mature, well-maintained, supports all major providers
- Works with free tiers (Gmail, SendGrid, Resend, etc.)
- Better Auth integrates with custom email sending functions

**Implementation Pattern**:
```typescript
import nodemailer from 'nodemailer';

const transporter = nodemailer.createTransport({
  host: process.env.SMTP_HOST,
  port: process.env.SMTP_PORT,
  auth: {
    user: process.env.SMTP_USER,
    pass: process.env.SMTP_PASS
  }
});

await transporter.sendMail({
  from: process.env.FROM_EMAIL,
  to: user.email,
  subject: 'Reset Your Password',
  html: resetEmailTemplate(resetUrl)
});
```

**Alternatives Considered**:
- Dedicated email API (SendGrid/Resend): Better deliverability but adds dependency
- Console logging for dev: Use in development; SMTP required for production

---

## Decision 6: Form Validation Strategy

**Decision**: Zod schema validation on both client and server

**Rationale**:
- Type-safe validation with TypeScript
- Same schemas shared between client/server
- Better Auth works well with Zod
- Provides clear error messages for users

**Validation Schemas**:
```typescript
import { z } from 'zod';

export const signupSchema = z.object({
  email: z.string().email('Invalid email format'),
  password: z.string().min(6, 'Password must be at least 6 characters'),
  softwareLevel: z.enum(['beginner', 'intermediate', 'advanced']),
  learningGoal: z.string().min(1, 'Learning goal is required'),
  hardwareAccess: z.enum(['none', 'basic', 'advanced']),
  technicalComfort: z.enum(['low', 'medium', 'high'])
});
```

**Alternatives Considered**:
- Yup: Similar but less TypeScript-native
- Manual validation: Error-prone; Zod preferred

---

## Decision 7: Background Questionnaire UX Pattern

**Decision**: Single-page form with 4 questions after credential submission

**Rationale**:
- Minimal approach (per spec: 3-4 questions)
- Single page reduces complexity vs. multi-step wizard
- All questions required; simple radio button inputs
- Progress indicator shows "Step 2 of 2" (credentials → background)

**Question Design**:
1. **Software Experience Level**: Radio buttons (Beginner / Intermediate / Advanced)
2. **Primary Learning Goal**: Text input (short answer, max 200 chars)
3. **Hardware Access Level**: Radio buttons (None / Basic / Advanced)
4. **Technical Comfort**: Radio buttons (Low / Medium / High)

**Alternatives Considered**:
- Multi-step wizard: More complex; unnecessary for 4 questions
- Optional questions: All required per minimal design decision

---

## Decision 8: Testing Strategy

**Decision**: Vitest for unit tests, Playwright for E2E auth flow tests

**Rationale**:
- Vitest: Fast, TypeScript-native, integrates with Next.js
- Playwright: Reliable browser automation for full auth flows
- Contract tests ensure API stability
- 85% completion rate (SC-006) measured via E2E timing tests

**Test Coverage Requirements**:
- Unit: Auth service functions, validation schemas
- Integration: Database operations, session management
- E2E: Full signup flow, signin flow, password reset
- Contract: API request/response schemas

**Alternatives Considered**:
- Jest: More mature but slower; Vitest sufficient
- Cypress: Good but Playwright has better multi-browser support

---

## Decision 9: Security Best Practices

**Decision**: Implement standard web security measures

**Key Security Measures**:
1. **Password Hashing**: bcrypt with cost 10
2. **HTTP-only Cookies**: Prevent XSS token theft
3. **CSRF Protection**: Better Auth built-in
4. **Rate Limiting**: 5 attempts per minute on auth endpoints
5. **Generic Error Messages**: Don't reveal if email exists
6. **Secure Password Reset**: Single-use tokens, 1-hour expiry
7. **Input Sanitization**: Zod validation on all inputs

**Rate Limiting Implementation**:
```typescript
import { rateLimit } from 'express-rate-limit';

const authLimiter = rateLimit({
  windowMs: 60 * 1000,
  max: 5,
  message: 'Too many attempts. Please try again later.'
});
```

---

## Decision 10: Error Handling & User Feedback

**Decision**: User-friendly error messages with technical logging

**Error Categories**:
- **Validation Errors**: Clear field-specific messages
- **Auth Failures**: Generic "Invalid credentials" (no email enumeration)
- **System Errors**: Friendly "Something went wrong" + log details
- **Network Errors**: "Connection issue. Check your internet."

**Logging Strategy**:
- All auth events logged (signup, signin, signout, reset request)
- Failed attempts include IP (for rate limiting)
- Never log passwords or tokens
- Use structured logging (pino/winston)

---

## Summary of Technology Choices

| Component | Choice | Rationale |
|-----------|--------|-----------|
| Framework | Next.js 14+ | App Router, React Server Components |
| Auth Library | Better Auth | User-requested, modern, TypeScript-first |
| Database | PostgreSQL (Neon Serverless) | Serverless architecture, free tier, Vercel integration |
| ORM | Prisma | Type-safe, easy migrations |
| Validation | Zod | TypeScript-native, shared schemas |
| Testing (Unit) | Vitest | Fast, TypeScript integration |
| Testing (E2E) | Playwright | Reliable browser automation |
| Email | Nodemailer + Gmail SMTP | Free, reliable, easy setup |
| Password Hashing | bcryptjs | Industry standard, secure |
| Session Storage | PostgreSQL (Neon) | Simple, no extra infrastructure |
| Deployment | Vercel | Zero-config Next.js deployment, integrates with Neon |

---

## Open Questions for Planning Phase

**All questions resolved**:

1. **Email Provider**: ✅ Gmail SMTP (free tier, app password required)
2. **Deployment Target**: ✅ Vercel (zero-config Next.js deployment)
3. **Database Hosting**: ✅ Neon Serverless Postgres (free tier, Vercel integration)
4. **Monitoring**: Defer to post-MVP phase (Sentry recommended for future)

These will be resolved in Phase 1 design or during task breakdown.
