# Authentication API Contracts

**Version**: 1.0.0
**Date**: 2026-03-12
**Based on**: Feature spec `specs/001-better-auth-signup/spec.md`

---

## Overview

This document defines the API contracts for the authentication system. All endpoints follow RESTful conventions and return JSON responses.

### Base URL

- **Development**: `http://localhost:3000/api/auth`
- **Production**: `https://<domain>/api/auth`

### Authentication Method

- **Session-based**: HTTP-only cookie (`better-auth.session_token`)
- **CSRF Protection**: Built into Better Auth

### Response Format

**Success Response**:
```json
{
  "success": true,
  "data": { ... }
}
```

**Error Response**:
```json
{
  "success": false,
  "error": {
    "code": "ERROR_CODE",
    "message": "User-friendly error message"
  }
}
```

### Common Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_CREDENTIALS` | 401 | Email or password incorrect |
| `USER_EXISTS` | 409 | Email already registered |
| `INVALID_INPUT` | 400 | Validation failed |
| `UNAUTHORIZED` | 401 | Not authenticated |
| `SESSION_EXPIRED` | 401 | Session timeout |
| `RATE_LIMITED` | 429 | Too many attempts |
| `SERVER_ERROR` | 500 | Internal server error |

---

## Endpoints

### POST `/signup`

Create a new user account with credentials and background information.

**Request**:
```typescript
{
  email: string;      // Valid email format, max 255 chars
  password: string;   // Min 6 characters
  softwareLevel: 'beginner' | 'intermediate' | 'advanced';
  learningGoal: string;  // 1-200 characters
  hardwareAccess: 'none' | 'basic' | 'advanced';
  technicalComfort: 'low' | 'medium' | 'high';
}
```

**Success Response** (201 Created):
```json
{
  "success": true,
  "data": {
    "user": {
      "id": "user_abc123",
      "email": "user@example.com",
      "emailVerified": null,
      "createdAt": "2026-03-12T10:00:00Z"
    },
    "profile": {
      "id": "profile_xyz789",
      "softwareLevel": "beginner",
      "learningGoal": "Learn robotics programming",
      "hardwareAccess": "none",
      "technicalComfort": "low"
    }
  }
}
```

**Error Responses**:

```json
// 400 Bad Request - Validation error
{
  "success": false,
  "error": {
    "code": "INVALID_INPUT",
    "message": "Validation failed",
    "details": [
      { "field": "email", "message": "Invalid email format" },
      { "field": "password", "message": "Password must be at least 6 characters" }
    ]
  }
}
```

```json
// 409 Conflict - Email exists
{
  "success": false,
  "error": {
    "code": "USER_EXISTS",
    "message": "An account with this email already exists"
  }
}
```

**Better Auth Integration**:
```typescript
import { auth } from '@/lib/auth';

// Better Auth signup with custom profile
const user = await auth.api.signUpEmail({
  body: {
    email: input.email,
    password: input.password,
    name: input.email.split('@')[0] // Derive name from email
  }
});

// Create profile separately
const profile = await prisma.userProfile.create({
  data: {
    userId: user.id,
    softwareLevel: input.softwareLevel,
    learningGoal: input.learningGoal,
    hardwareAccess: input.hardwareAccess,
    technicalComfort: input.technicalComfort
  }
});
```

---

### POST `/signin`

Authenticate existing user and create session.

**Request**:
```typescript
{
  email: string;
  password: string;
}
```

**Success Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "user": {
      "id": "user_abc123",
      "email": "user@example.com",
      "emailVerified": "2026-03-12T10:00:00Z"
    },
    "session": {
      "expiresAt": "2026-03-19T10:00:00Z"
    }
  }
}
```

**Note**: Session token set as HTTP-only cookie automatically by Better Auth.

**Error Responses**:

```json
// 401 Unauthorized - Invalid credentials
{
  "success": false,
  "error": {
    "code": "INVALID_CREDENTIALS",
    "message": "Invalid email or password"
  }
}
```

```json
// 400 Bad Request - Missing fields
{
  "success": false,
  "error": {
    "code": "INVALID_INPUT",
    "message": "Email and password are required"
  }
}
```

**Better Auth Integration**:
```typescript
import { auth } from '@/lib/auth';

const user = await auth.api.signInEmail({
  body: {
    email: input.email,
    password: input.password
  }
});
```

---

### POST `/signout`

Invalidate current session and clear authentication.

**Request**: No body (session from cookie)

**Success Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "message": "Successfully signed out"
  }
}
```

**Error Responses**:

```json
// 401 Unauthorized - No active session
{
  "success": false,
  "error": {
    "code": "UNAUTHORIZED",
    "message": "No active session"
  }
}
```

**Better Auth Integration**:
```typescript
import { auth } from '@/lib/auth';

await auth.api.signOut({
  headers: request.headers
});
```

---

### POST `/reset-password`

Request password reset email.

**Request**:
```typescript
{
  email: string;
}
```

**Success Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "message": "If an account exists with this email, a reset link has been sent"
  }
}
```

**Note**: Generic message prevents email enumeration attacks. Email sent regardless of whether account exists.

**Error Responses**:

```json
// 400 Bad Request - Invalid email
{
  "success": false,
  "error": {
    "code": "INVALID_INPUT",
    "message": "Valid email is required"
  }
}
```

**Better Auth Integration**:
```typescript
import { auth } from '@/lib/auth';

await auth.api.forgetPassword({
  body: {
    email: input.email
  }
});
```

---

### POST `/reset-password/confirm`

Complete password reset with token.

**Request**:
```typescript
{
  token: string;      // Reset token from email link
  newPassword: string; // New password (min 6 chars)
}
```

**Success Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "message": "Password reset successfully"
  }
}
```

**Error Responses**:

```json
// 400 Bad Request - Invalid/expired token
{
  "success": false,
  "error": {
    "code": "INVALID_TOKEN",
    "message": "Invalid or expired reset token"
  }
}
```

```json
// 400 Bad Request - Weak password
{
  "success": false,
  "error": {
    "code": "INVALID_INPUT",
    "message": "Password must be at least 6 characters"
  }
}
```

**Better Auth Integration**:
```typescript
import { auth } from '@/lib/auth';

await auth.api.resetPassword({
  query: {
    token: input.token
  },
  body: {
    newPassword: input.newPassword
  }
});
```

---

### GET `/session`

Get current authenticated user session.

**Request**: No body (session from cookie)

**Success Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "user": {
      "id": "user_abc123",
      "email": "user@example.com",
      "emailVerified": "2026-03-12T10:00:00Z"
    },
    "session": {
      "expiresAt": "2026-03-19T10:00:00Z"
    }
  }
}
```

**Error Responses**:

```json
// 401 Unauthorized - No active session
{
  "success": false,
  "error": {
    "code": "UNAUTHORIZED",
    "message": "Not authenticated"
  }
}
```

**Better Auth Integration**:
```typescript
import { auth } from '@/lib/auth';

const session = await auth.api.getSession({
  headers: request.headers
});
```

---

### GET `/profile`

Get current user's background profile.

**Request**: No body (session from cookie)

**Success Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "id": "profile_xyz789",
    "softwareLevel": "beginner",
    "learningGoal": "Learn robotics programming",
    "hardwareAccess": "none",
    "technicalComfort": "low",
    "createdAt": "2026-03-12T10:00:00Z",
    "updatedAt": "2026-03-12T10:00:00Z"
  }
}
```

**Error Responses**:

```json
// 401 Unauthorized - No active session
{
  "success": false,
  "error": {
    "code": "UNAUTHORIZED",
    "message": "Not authenticated"
  }
}
```

```json
// 404 Not Found - Profile doesn't exist
{
  "success": false,
  "error": {
    "code": "NOT_FOUND",
    "message": "Profile not found"
  }
}
```

**Implementation**:
```typescript
const profile = await prisma.userProfile.findUnique({
  where: { userId: session.user.id }
});
```

---

### PUT `/profile`

Update current user's background profile.

**Request**:
```typescript
{
  softwareLevel?: 'beginner' | 'intermediate' | 'advanced';
  learningGoal?: string;  // 1-200 characters
  hardwareAccess?: 'none' | 'basic' | 'advanced';
  technicalComfort?: 'low' | 'medium' | 'high';
}
```

**Success Response** (200 OK):
```json
{
  "success": true,
  "data": {
    "id": "profile_xyz789",
    "softwareLevel": "intermediate",
    "learningGoal": "Build autonomous robots",
    "hardwareAccess": "basic",
    "technicalComfort": "medium",
    "updatedAt": "2026-03-12T11:00:00Z"
  }
}
```

**Error Responses**:

```json
// 400 Bad Request - Validation error
{
  "success": false,
  "error": {
    "code": "INVALID_INPUT",
    "message": "Invalid profile data",
    "details": [
      { "field": "learningGoal", "message": "Learning goal is required" }
    ]
  }
}
```

**Implementation**:
```typescript
const profile = await prisma.userProfile.update({
  where: { userId: session.user.id },
  data: {
    softwareLevel: input.softwareLevel,
    learningGoal: input.learningGoal,
    hardwareAccess: input.hardwareAccess,
    technicalComfort: input.technicalComfort
  }
});
```

---

## Rate Limiting

All authentication endpoints are rate-limited to prevent abuse:

| Endpoint | Limit | Window |
|----------|-------|--------|
| `/signup` | 5 requests | 1 minute |
| `/signin` | 5 requests | 1 minute |
| `/reset-password` | 3 requests | 1 hour |
| `/reset-password/confirm` | 5 requests | 1 minute |
| Other endpoints | 100 requests | 1 minute |

**Rate Limit Response** (429 Too Many Requests):
```json
{
  "success": false,
  "error": {
    "code": "RATE_LIMITED",
    "message": "Too many attempts. Please try again in 60 seconds.",
    "retryAfter": 60
  }
}
```

---

## OpenAPI Specification

Full OpenAPI 3.0 spec available in `contracts/auth-api.yaml`.

```yaml
openapi: 3.0.3
info:
  title: Authentication API
  version: 1.0.0
  description: User authentication and background profile management

servers:
  - url: http://localhost:3000/api/auth
    description: Development
  - url: https://yourdomain.com/api/auth
    description: Production

paths:
  /signup:
    post:
      summary: Create new user account
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/SignupRequest'
      responses:
        '201':
          description: Account created successfully
        '400':
          description: Validation error
        '409':
          description: Email already exists
  # ... additional paths
```

---

## Testing Guidelines

### Contract Tests

Verify API behavior with these test scenarios:

1. **Signup Flow**:
   - Valid signup creates user and profile
   - Duplicate email returns 409
   - Invalid email format returns 400
   - Password < 6 chars returns 400

2. **Signin Flow**:
   - Valid credentials create session
   - Invalid credentials return 401 (generic message)
   - Session cookie set on response

3. **Password Reset**:
   - Reset request succeeds even for non-existent email
   - Valid token allows password change
   - Expired token returns 400

4. **Profile Operations**:
   - Authenticated user can view/update profile
   - Unauthenticated request returns 401
   - Invalid enum values return 400

### Example Test (Vitest)

```typescript
import { describe, it, expect } from 'vitest';

describe('POST /api/auth/signup', () => {
  it('creates user and profile with valid data', async () => {
    const response = await fetch('/api/auth/signup', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        email: 'test@example.com',
        password: 'password123',
        softwareLevel: 'beginner',
        learningGoal: 'Learn robotics',
        hardwareAccess: 'none',
        technicalComfort: 'low'
      })
    });

    expect(response.status).toBe(201);
    const data = await response.json();
    expect(data.success).toBe(true);
    expect(data.data.user.email).toBe('test@example.com');
    expect(data.data.profile.softwareLevel).toBe('beginner');
  });

  it('rejects duplicate email', async () => {
    // ... test duplicate email scenario
  });
});
```
