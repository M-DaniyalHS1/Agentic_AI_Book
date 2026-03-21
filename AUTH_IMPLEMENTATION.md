# Authentication Implementation Guide

**Feature**: Better Auth Signup and Signin with User Background Collection  
**Branch**: `001-better-auth-signup`  
**Date**: 2026-03-14  
**Status**: Implementation Complete (Pending Database Setup)

---

## Summary

This document describes the complete implementation of the authentication system using Better Auth principles with Prisma ORM, FastAPI backend, and Docusaurus frontend.

---

## What Was Implemented

### Backend (FastAPI + Prisma)

#### Database Schema (`backend/prisma/schema.prisma`)

- **User** model: id, email, passwordHash, emailVerified, timestamps
- **UserProfile** model: userId, softwareLevel, learningGoal, hardwareAccess, technicalComfort
- **Session** model: userId, token, expiresAt, timestamps
- All relationships and indexes configured

#### Authentication API (`backend/api/auth.py`)

- `POST /api/auth/signup` - User registration with background data
- `POST /api/auth/signin` - User authentication
- `POST /api/auth/signout` - Session invalidation
- `GET /api/auth/profile` - Get current user profile
- `PUT /api/auth/profile` - Update user profile
- `POST /api/auth/reset-password` - Request password reset
- `POST /api/auth/reset-password/confirm` - Confirm password reset
- `GET /api/auth/me` - Check authentication status

#### Supporting Modules

- `backend/src/db/prisma.py` - Prisma client singleton
- `backend/src/auth/validations.py` - Pydantic schemas for validation
- `backend/src/auth/errors.py` - Custom auth error classes
- `backend/src/services/email.py` - Gmail SMTP email service
- `backend/main.py` - Updated with auth router and Prisma lifecycle

### Frontend (Docusaurus + React)

#### Pages

- `/signup` - User registration page
- `/signup/background` - Background questionnaire (4 questions)
- `/signin` - User signin page
- `/dashboard` - Authenticated user dashboard
- `/profile` - Profile view and edit page
- `/reset-password` - Password reset request page
- `/reset-password/confirm` - Password reset confirmation page

#### Components

- `SignupForm.tsx` - Two-step signup (credentials → background)
- `BackgroundQuestions.tsx` - 4-question wizard component
- `SigninForm.tsx` - Email/password signin form
- `SignOutButton.tsx` - Sign out functionality
- `ResetPasswordRequest.tsx` - Password reset request form
- `ResetPasswordConfirm.tsx` - Password reset confirmation form

---

## Setup Instructions

### 1. Install Dependencies

```bash
# Backend
cd backend
pip install -r requirements.txt

# Frontend
cd frontend
npm install
```

### 2. Configure Environment Variables

Copy `.env.example` to `.env` in the project root:

```bash
# Required: Database (Neon Serverless Postgres)
DATABASE_URL=postgresql://user:password@host.neon.tech/database

# Required: Auth secret (generate with Python)
python -c "import secrets; print(secrets.token_urlsafe(32))"
BETTER_AUTH_SECRET=<generated-secret>

# Required: Gmail SMTP (for password reset emails)
# Create app password at: https://myaccount.google.com/apppasswords
SMTP_HOST=smtp.gmail.com
SMTP_PORT=587
SMTP_USERNAME=your-email@gmail.com
SMTP_PASSWORD=your-app-password
SMTP_FROM_NAME=Agent Book Factory
SMTP_FROM_EMAIL=noreply@agent-book-factory.vercel.app

# Optional: Session configuration
SESSION_EXPIRY_DAYS=7
SESSION_COOKIE_NAME=better-auth.session_token
```

### 3. Setup Neon Database

1. Go to https://neon.tech and create a free account
2. Create a new project
3. Copy the connection string (Pooler mode recommended)
4. Update `DATABASE_URL` in `.env`

### 4. Initialize Database with Prisma

```bash
cd backend

# Generate Prisma client
prisma generate

# Run initial migration
prisma migrate dev --name init_auth

# (Optional) Seed test data
# python prisma/seed.py  # Create this if needed
```

### 5. Run the Application

```bash
# Backend (development)
cd backend
uvicorn main:app --reload --port 8000

# Frontend (development)
cd frontend
npm start
```

### 6. Test the Authentication Flow

1. Open http://localhost:3000/signup
2. Create a new account with email/password
3. Complete the 4 background questions
4. Verify redirect to dashboard
5. Test signout and signin
6. Test password reset flow

---

## API Endpoints

### Authentication

| Method | Endpoint | Description | Auth Required |
|--------|----------|-------------|---------------|
| POST | `/api/auth/signup` | Register new user | No |
| POST | `/api/auth/signin` | Sign in user | No |
| POST | `/api/auth/signout` | Sign out user | Yes |
| GET | `/api/auth/profile` | Get user profile | Yes |
| PUT | `/api/auth/profile` | Update user profile | Yes |
| POST | `/api/auth/reset-password` | Request reset link | No |
| POST | `/api/auth/reset-password/confirm` | Reset password | No |
| GET | `/api/auth/me` | Check auth status | No |

### Request/Response Examples

#### Signup Request

```json
POST /api/auth/signup
{
  "email": "user@example.com",
  "password": "password123",
  "software_level": "beginner",
  "learning_goal": "Learn robotics programming",
  "hardware_access": "none",
  "technical_comfort": "medium"
}
```

#### Signup Response

```json
{
  "user_id": "clxxx...",
  "email": "user@example.com",
  "message": "Account created successfully"
}
```

#### Signin Request

```json
POST /api/auth/signin
{
  "email": "user@example.com",
  "password": "password123"
}
```

#### Error Response

```json
{
  "detail": {
    "error": "Invalid email or password",
    "code": "INVALID_CREDENTIALS"
  }
}
```

---

## Security Features

### Implemented

1. **Password Hashing**: bcrypt with salt rounds
2. **Session Management**: JWT tokens with HTTP-only cookies
3. **Rate Limiting**: 
   - Signup: 5 requests/minute
   - Signin: 5 requests/minute  
   - Password reset: 3 requests/hour
4. **Email Enumeration Prevention**: Generic error messages
5. **Token Expiration**: 
   - Sessions: 7 days
   - Reset tokens: 1 hour
6. **Session Invalidation**: All sessions invalidated on password reset
7. **XSS Prevention**: Input validation, no HTML in user data
8. **HTTPS Cookies**: Secure flag set in production

### Recommendations for Production

1. Enable HTTPS (Vercel does this automatically)
2. Use strong `BETTER_AUTH_SECRET` (32+ characters)
3. Configure CORS properly for production domain
4. Enable 2FA (future enhancement)
5. Add account lockout after failed attempts (future)

---

## Database Schema

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

  @@index([email])
}

model UserProfile {
  id                 String   @id @default(cuid())
  userId             String   @unique
  user               User     @relation(fields: [userId], references: [id], onDelete: Cascade)
  softwareLevel      String
  learningGoal       String   @db.VarChar(200)
  hardwareAccess     String
  technicalComfort   String
  createdAt          DateTime @default(now())
  updatedAt          DateTime @updatedAt

  @@index([userId])
}

model Session {
  id        String   @id @default(cuid())
  userId    String
  user      User     @relation(fields: [userId], references: [id], onDelete: Cascade)
  token     String   @unique
  expiresAt DateTime
  createdAt DateTime @default(now())

  @@index([token])
  @@index([expiresAt])
}
```

---

## Background Questions

The system collects 4 background questions during signup:

1. **Software Level** (enum): beginner | intermediate | advanced
2. **Learning Goal** (string, max 200 chars): User's primary learning objective
3. **Hardware Access** (enum): none | basic | advanced
4. **Technical Comfort** (enum): low | medium | high

This data is stored in `UserProfile` and can be used for future personalization features.

---

## File Structure

```
agent_book_factory/
├── backend/
│   ├── api/
│   │   └── auth.py              # Auth API routes
│   ├── prisma/
│   │   └── schema.prisma        # Database schema
│   ├── src/
│   │   ├── auth/
│   │   │   ├── __init__.py
│   │   │   ├── errors.py        # Custom errors
│   │   │   └── validations.py   # Pydantic schemas
│   │   ├── db/
│   │   │   ├── __init__.py
│   │   │   └── prisma.py        # Prisma client
│   │   └── services/
│   │       └── email.py         # Email service
│   └── main.py                  # FastAPI app (updated)
├── frontend/
│   └── src/
│       ├── components/
│       │   └── auth/
│       │       ├── SignupForm.tsx
│       │       ├── BackgroundQuestions.tsx
│       │       ├── SigninForm.tsx
│       │       ├── SignOutButton.tsx
│       │       ├── ResetPasswordRequest.tsx
│       │       └── ResetPasswordConfirm.tsx
│       └── pages/
│           ├── signup.tsx
│           ├── signup/background.tsx
│           ├── signin.tsx
│           ├── dashboard.tsx
│           ├── profile.tsx
│           ├── reset-password.tsx
│           └── reset-password/confirm.tsx
├── .env.example                 # Updated with auth vars
└── .gitignore                   # Updated with Prisma entries
```

---

## Testing

### Manual Testing Checklist

- [ ] Signup with valid data
- [ ] Signup with existing email (should fail)
- [ ] Signup with invalid email format (should fail)
- [ ] Signup with weak password (should fail)
- [ ] Complete background questionnaire
- [ ] Dashboard loads after signup
- [ ] Signout clears session
- [ ] Signin with valid credentials
- [ ] Signin with invalid credentials (should fail)
- [ ] Profile view shows correct data
- [ ] Profile update works
- [ ] Password reset request sends email
- [ ] Password reset link works
- [ ] Password reset invalidates sessions
- [ ] Rate limiting triggers after too many attempts

### Automated Testing (Future)

Contract tests and E2E tests can be added following the patterns in `tasks.md`.

---

## Deployment to Vercel

### Backend

The backend is already configured for Vercel serverless deployment.

1. Set environment variables in Vercel dashboard
2. Deploy: `vercel --prod`

### Frontend

Docusaurus builds automatically on Vercel.

1. Ensure build command: `npm run build`
2. Ensure output directory: `build`

### Environment Variables in Vercel

Add these in Vercel Dashboard > Project Settings > Environment Variables:

- `DATABASE_URL` (Neon connection string)
- `BETTER_AUTH_SECRET` (random 32+ char string)
- `SMTP_HOST`, `SMTP_PORT`, `SMTP_USERNAME`, `SMTP_PASSWORD`
- `SESSION_EXPIRY_DAYS` (optional, default 7)

---

## Troubleshooting

### Prisma Client Not Generated

```bash
cd backend
prisma generate
```

### Migration Errors

```bash
cd backend
prisma migrate reset  # Reset and reapply migrations
```

### Email Not Sending

- Check Gmail app password is correct
- Verify SMTP credentials in `.env`
- Check spam folder for test emails

### Session Not Persisting

- Ensure cookies are enabled in browser
- Check `BETTER_AUTH_SECRET` is set
- Verify CORS settings allow credentials

---

## Next Steps

### Immediate (Required for MVP)

1. Set up Neon database
2. Run Prisma migrations
3. Configure Gmail SMTP
4. Test full signup flow end-to-end

### Future Enhancements

- Email verification on signup
- OAuth providers (Google, GitHub)
- 2FA support
- Password strength meter
- Account recovery questions
- Session management UI (view active sessions)
- Personalization logic based on background data

---

## Credits

- **Better Auth**: Authentication library inspiration
- **Prisma**: Database ORM
- **FastAPI**: Python web framework
- **Docusaurus**: React documentation framework
