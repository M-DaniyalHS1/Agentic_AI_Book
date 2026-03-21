# Quickstart: Better Auth Authentication Setup

**Version**: 1.0.0
**Date**: 2026-03-12
**Prerequisites**: Node.js 20+, Neon account, Gmail account, Vercel account

---

## Overview

This quickstart guide walks you through setting up the Better Auth authentication system with user background collection.

**Production Stack**:
- **Hosting**: Vercel (zero-config deployment)
- **Database**: Neon Serverless Postgres (free tier)
- **Email**: Gmail SMTP (free tier with app password)

**Time to Complete**: 15-20 minutes

---

## Step 1: Install Dependencies

```bash
# Core dependencies
npm install better-auth @better-auth/react
npm install prisma @prisma/client
npm install bcryptjs
npm install zod

# Development dependencies
npm install -D prisma
npm install -D @types/bcryptjs
npm install -D vitest @playwright/test
```

---

## Step 2: Configure Environment Variables

Create `.env` file in project root:

```bash
# Database (Neon Serverless Postgres)
# Get connection string from Neon dashboard: https://console.neon.tech
DATABASE_URL="postgresql://user:password@ep-xxx.us-east-2.aws.neon.tech/agent_book_factory?sslmode=require"

# Better Auth Secret (generate with: openssl rand -base64 32)
BETTER_AUTH_SECRET="your-secret-key-here-min-32-chars"

# Email (Gmail SMTP)
# Get app password from Google Account settings: myaccount.google.com/apppasswords
SMTP_HOST="smtp.gmail.com"
SMTP_PORT="587"
SMTP_USER="your-email@gmail.com"
SMTP_PASS="your-gmail-app-password"  # NOT your regular password
FROM_EMAIL="noreply@yourdomain.com"

# Application
NEXT_PUBLIC_APP_URL="http://localhost:3000"

# Vercel (for deployment)
VERCEL_PROJECT_ID="your-project-id"
```

**Gmail App Password Setup**:
1. Go to Google Account → Security
2. Enable 2-Step Verification (if not enabled)
3. Go to App Passwords: myaccount.google.com/apppasswords
4. Select "Mail" and your device
5. Copy the 16-character password to `SMTP_PASS`

**Neon Database Setup**:
1. Go to https://console.neon.tech
2. Create new project
3. Copy connection string (includes `?sslmode=require`)
4. Create database named `agent_book_factory`

**Security Note**: Never commit `.env` to version control. Add to `.gitignore`.

---

## Step 3: Initialize Prisma Schema

Create `prisma/schema.prisma`:

```prisma
generator client {
  provider = "prisma-client-js"
}

datasource db {
  provider = "postgresql"
  url      = env("DATABASE_URL")
}

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
  id               String   @id @default(cuid())
  userId           String   @unique
  user             User     @relation(fields: [userId], references: [id], onDelete: Cascade)
  softwareLevel    String
  learningGoal     String   @db.VarChar(200)
  hardwareAccess   String
  technicalComfort String
  createdAt        DateTime @default(now())
  updatedAt        DateTime @updatedAt

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

Run Prisma migrations:

```bash
npx prisma migrate dev --name init_auth
npx prisma generate
```

---

## Step 4: Configure Better Auth

Create `src/lib/auth.ts`:

```typescript
import { betterAuth } from 'better-auth';
import { prismaAdapter } from 'better-auth/adapters/prisma';
import { prisma } from './prisma';

export const auth = betterAuth({
  database: prismaAdapter(prisma, {
    provider: 'postgresql'
  }),
  
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 6,
    requireEmailVerification: false // Set to true if implementing email verification
  },
  
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24,     // Refresh every 24h
    cookieCache: {
      enabled: true,
      maxAge: 60 * 5             // 5-minute cache
    }
  },
  
  rateLimit: {
    enabled: true,
    window: 60,
    max: 5
  }
});
```

---

## Step 5: Create Auth API Routes

Create `src/app/api/auth/[...all]/route.ts`:

```typescript
import { auth } from '@/lib/auth';
import { toNextJsHandler } from 'better-auth/next-js';

export const { GET, POST } = toNextJsHandler(auth);
```

---

## Step 6: Implement Signup with Background Questions

Create `src/app/api/auth/signup/route.ts`:

```typescript
import { NextRequest, NextResponse } from 'next/server';
import { auth } from '@/lib/auth';
import { prisma } from '@/lib/prisma';
import { hash } from 'bcryptjs';
import { signupSchema } from '@/lib/validations';

export async function POST(request: NextRequest) {
  try {
    const body = await request.json();
    
    // Validate input
    const validated = signupSchema.safeParse(body);
    if (!validated.success) {
      return NextResponse.json(
        {
          success: false,
          error: {
            code: 'INVALID_INPUT',
            message: 'Validation failed',
            details: validated.error.errors
          }
        },
        { status: 400 }
      );
    }

    const { email, password, ...profileData } = validated.data;

    // Check if user exists
    const existingUser = await prisma.user.findUnique({
      where: { email }
    });

    if (existingUser) {
      return NextResponse.json(
        {
          success: false,
          error: {
            code: 'USER_EXISTS',
            message: 'An account with this email already exists'
          }
        },
        { status: 409 }
      );
    }

    // Hash password
    const passwordHash = await hash(password, 10);

    // Create user with profile
    const user = await prisma.user.create({
      data: {
        email,
        passwordHash,
        profile: {
          create: profileData
        }
      },
      include: {
        profile: true
      }
    });

    // Sign in user (create session)
    const signInResponse = await auth.api.signInEmail({
      body: {
        email,
        password
      }
    });

    return NextResponse.json(
      {
        success: true,
        data: {
          user: {
            id: user.id,
            email: user.email,
            emailVerified: user.emailVerified
          },
          profile: user.profile
        }
      },
      { status: 201 }
    );
  } catch (error) {
    console.error('Signup error:', error);
    return NextResponse.json(
      {
        success: false,
        error: {
          code: 'SERVER_ERROR',
          message: 'An unexpected error occurred'
        }
      },
      { status: 500 }
    );
  }
}
```

---

## Step 7: Create Validation Schemas

Create `src/lib/validations.ts`:

```typescript
import { z } from 'zod';

export const signupSchema = z.object({
  email: z.string().email('Invalid email format').max(255),
  password: z.string().min(6, 'Password must be at least 6 characters'),
  softwareLevel: z.enum(['beginner', 'intermediate', 'advanced']),
  learningGoal: z.string().min(1, 'Learning goal is required').max(200),
  hardwareAccess: z.enum(['none', 'basic', 'advanced']),
  technicalComfort: z.enum(['low', 'medium', 'high'])
});

export const signinSchema = z.object({
  email: z.string().email('Invalid email format'),
  password: z.string().min(1, 'Password is required')
});

export const profileSchema = z.object({
  softwareLevel: z.enum(['beginner', 'intermediate', 'advanced']).optional(),
  learningGoal: z.string().min(1, 'Learning goal is required').max(200).optional(),
  hardwareAccess: z.enum(['none', 'basic', 'advanced']).optional(),
  technicalComfort: z.enum(['low', 'medium', 'high']).optional()
});
```

---

## Step 8: Create Auth Components

Create `src/components/auth/signup-form.tsx`:

```typescript
'use client';

import { useState } from 'react';
import { useRouter } from 'next/navigation';
import { BackgroundQuestions } from './background-questions';

export function SignupForm() {
  const router = useRouter();
  const [step, setStep] = useState<'credentials' | 'background'>('credentials');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  
  const [credentials, setCredentials] = useState({
    email: '',
    password: ''
  });
  
  const [background, setBackground] = useState({
    softwareLevel: '',
    learningGoal: '',
    hardwareAccess: '',
    technicalComfort: ''
  });

  const handleCredentialsSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    // Validate credentials
    if (credentials.password.length < 6) {
      setError('Password must be at least 6 characters');
      return;
    }
    setStep('background');
  };

  const handleBackgroundSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    try {
      const response = await fetch('/api/auth/signup', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          ...credentials,
          ...background
        })
      });

      const data = await response.json();

      if (!data.success) {
        throw new Error(data.error.message);
      }

      // Redirect to dashboard
      router.push('/dashboard');
    } catch (err: any) {
      setError(err.message);
      setLoading(false);
    }
  };

  return (
    <div className="max-w-md mx-auto p-6">
      <h1 className="text-2xl font-bold mb-6">Create Your Account</h1>
      
      {error && (
        <div className="bg-red-50 text-red-600 p-3 rounded mb-4">
          {error}
        </div>
      )}

      {step === 'credentials' ? (
        <form onSubmit={handleCredentialsSubmit}>
          <div className="mb-4">
            <label className="block text-sm font-medium mb-2">Email</label>
            <input
              type="email"
              value={credentials.email}
              onChange={(e) => setCredentials({ ...credentials, email: e.target.value })}
              className="w-full border rounded px-3 py-2"
              required
            />
          </div>
          <div className="mb-4">
            <label className="block text-sm font-medium mb-2">Password</label>
            <input
              type="password"
              value={credentials.password}
              onChange={(e) => setCredentials({ ...credentials, password: e.target.value })}
              className="w-full border rounded px-3 py-2"
              minLength={6}
              required
            />
            <p className="text-xs text-gray-500 mt-1">Minimum 6 characters</p>
          </div>
          <button
            type="submit"
            disabled={loading}
            className="w-full bg-blue-600 text-white py-2 rounded hover:bg-blue-700"
          >
            Next
          </button>
        </form>
      ) : (
        <BackgroundQuestions
          background={background}
          setBackground={setBackground}
          onSubmit={handleBackgroundSubmit}
          loading={loading}
        />
      )}
    </div>
  );
}
```

Create `src/components/auth/background-questions.tsx`:

```typescript
'use client';

interface BackgroundQuestionsProps {
  background: {
    softwareLevel: string;
    learningGoal: string;
    hardwareAccess: string;
    technicalComfort: string;
  };
  setBackground: (data: any) => void;
  onSubmit: (e: React.FormEvent) => void;
  loading: boolean;
}

export function BackgroundQuestions({ background, setBackground, onSubmit, loading }: BackgroundQuestionsProps) {
  return (
    <form onSubmit={onSubmit}>
      <div className="mb-4">
        <label className="block text-sm font-medium mb-2">
          1. Software Experience Level
        </label>
        <select
          value={background.softwareLevel}
          onChange={(e) => setBackground({ ...background, softwareLevel: e.target.value })}
          className="w-full border rounded px-3 py-2"
          required
        >
          <option value="">Select...</option>
          <option value="beginner">Beginner - Little to no programming experience</option>
          <option value="intermediate">Intermediate - Comfortable with basic programming</option>
          <option value="advanced">Advanced - Experienced developer</option>
        </select>
      </div>

      <div className="mb-4">
        <label className="block text-sm font-medium mb-2">
          2. Primary Learning Goal
        </label>
        <input
          type="text"
          value={background.learningGoal}
          onChange={(e) => setBackground({ ...background, learningGoal: e.target.value })}
          className="w-full border rounded px-3 py-2"
          placeholder="e.g., Build autonomous robots"
          maxLength={200}
          required
        />
      </div>

      <div className="mb-4">
        <label className="block text-sm font-medium mb-2">
          3. Hardware Access
        </label>
        <select
          value={background.hardwareAccess}
          onChange={(e) => setBackground({ ...background, hardwareAccess: e.target.value })}
          className="w-full border rounded px-3 py-2"
          required
        >
          <option value="">Select...</option>
          <option value="none">None - No robotics hardware</option>
          <option value="basic">Basic - Arduino, simple sensors, or free simulators</option>
          <option value="advanced">Advanced - ROS, humanoid robots, Gazebo</option>
        </select>
      </div>

      <div className="mb-4">
        <label className="block text-sm font-medium mb-2">
          4. Technical Comfort Level
        </label>
        <select
          value={background.technicalComfort}
          onChange={(e) => setBackground({ ...background, technicalComfort: e.target.value })}
          className="w-full border rounded px-3 py-2"
          required
        >
          <option value="">Select...</option>
          <option value="low">Low - Prefer step-by-step guidance</option>
          <option value="medium">Medium - Comfortable with some independent problem-solving</option>
          <option value="high">High - Confident debugging and advanced topics</option>
        </select>
      </div>

      <div className="flex gap-3">
        <button
          type="button"
          onClick={() => setStep('credentials')}
          className="flex-1 border py-2 rounded hover:bg-gray-50"
        >
          Back
        </button>
        <button
          type="submit"
          disabled={loading}
          className="flex-1 bg-blue-600 text-white py-2 rounded hover:bg-blue-700"
        >
          {loading ? 'Creating Account...' : 'Complete Signup'}
        </button>
      </div>
    </form>
  );
}
```

---

## Step 9: Run Development Server

```bash
# Start Next.js development server
npm run dev
```

Visit `http://localhost:3000/signup` to test the authentication flow.

---

## Step 10: Test the Implementation

### Manual Testing Checklist

- [ ] Signup with valid credentials creates account
- [ ] Background questions are saved correctly
- [ ] Duplicate email shows error message
- [ ] Password < 6 chars shows validation error
- [ ] Signin with valid credentials works
- [ ] Signin with invalid credentials shows generic error
- [ ] Session persists across page refreshes
- [ ] Signout clears session
- [ ] Profile page shows background data
- [ ] Profile update works

### Automated Testing

```bash
# Run unit tests
npm run test:unit

# Run E2E tests with Playwright
npm run test:e2e
```

---

## Troubleshooting

### Database Connection Error

```bash
# Verify PostgreSQL is running
psql -U your_user -d agent_book_factory

# Check DATABASE_URL in .env
```

### Better Auth Configuration Error

```bash
# Ensure BETTER_AUTH_SECRET is set (min 32 characters)
echo $BETTER_AUTH_SECRET

# Regenerate if needed
openssl rand -base64 32
```

### Email Not Sending

```bash
# Verify SMTP credentials
# For Gmail: enable "App Passwords" in Google Account settings
# Use App Password, not regular password
```

---

## Production Deployment on Vercel

### Step 1: Push to Git

```bash
git add .
git commit -m "Add Better Auth authentication system"
git push origin 001-better-auth-signup
```

### Step 2: Connect to Vercel

1. Go to https://vercel.com/new
2. Import your GitHub repository
3. Select branch: `001-better-auth-signup` (or `main` for production)
4. Configure project:
   - **Framework Preset**: Next.js
   - **Root Directory**: `./`
   - **Build Command**: `npm run build`
   - **Output Directory**: (leave default)

### Step 3: Add Environment Variables

In Vercel dashboard → Project Settings → Environment Variables:

```
DATABASE_URL=postgresql://user:password@ep-xxx.us-east-2.aws.neon.tech/agent_book_factory?sslmode=require
BETTER_AUTH_SECRET=<generate-32-char-secret>
SMTP_HOST=smtp.gmail.com
SMTP_PORT=587
SMTP_USER=your-email@gmail.com
SMTP_PASS=<gmail-app-password>
FROM_EMAIL=noreply@yourdomain.com
NEXT_PUBLIC_APP_URL=https://your-project.vercel.app
```

### Step 4: Deploy

```bash
# Install Vercel CLI (optional)
npm i -g vercel

# Deploy to preview
vercel

# Deploy to production
vercel --prod
```

### Step 5: Run Database Migrations

```bash
# Run migrations on production database
npx prisma migrate deploy
```

### Step 6: Verify Deployment

1. Visit your Vercel URL
2. Test signup flow
3. Verify email sending works
4. Check Neon dashboard for database connections

---

## Next Steps

1. **Email Verification**: Enable `requireEmailVerification: true` in auth config
2. **Password Reset**: Implement reset password flow using Better Auth's `forgetPassword` API
3. **Session Management**: Add middleware for protected routes
4. **Monitoring**: Set up Sentry for error tracking (recommended)
5. **Custom Domain**: Configure in Vercel settings

---

## Resources

- Better Auth Documentation: https://www.better-auth.com/
- Prisma Documentation: https://www.prisma.io/docs
- Next.js Authentication Guide: https://nextjs.org/docs/authentication
