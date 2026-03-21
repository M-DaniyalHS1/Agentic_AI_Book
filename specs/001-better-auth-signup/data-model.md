# Data Model: User Authentication & Background Collection

**Version**: 1.0.0
**Date**: 2026-03-12
**Source**: Feature spec `specs/001-better-auth-signup/spec.md`

---

## Entity Relationship Diagram

```
┌─────────────────┐       ┌──────────────────┐       ┌─────────────────┐
│     User        │       │   UserProfile    │       │     Session     │
├─────────────────┤       ├──────────────────┤       ├─────────────────┤
│ id (PK)         │◄──────│ userId (FK, UQ)  │       │ id (PK)         │
│ email (UQ)      │ 1   1 │ id (PK)          │       │ userId (FK)     │
│ passwordHash    │──────►│ softwareLevel    │       │ token (UQ)      │
│ emailVerified   │       │ learningGoal     │       │ expiresAt       │
│ createdAt       │       │ hardwareAccess   │       │ createdAt       │
│ updatedAt       │       │ technicalComfort │       │                 │
└─────────────────┘       │ createdAt        │       └─────────────────┘
                          │ updatedAt        │
                          └──────────────────┘
```

---

## Entity: User

**Purpose**: Core authentication account representing a registered user

### Attributes

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | String (CUID) | Primary Key, Auto-generated | Unique user identifier |
| email | String | Unique, Not Null, Email format | User's email address for authentication |
| passwordHash | String | Not Null, Min 60 chars | Bcrypt hashed password |
| emailVerified | DateTime | Nullable | Timestamp when email was verified (null if not verified) |
| createdAt | DateTime | Auto-generated | Account creation timestamp |
| updatedAt | DateTime | Auto-updated | Last profile update timestamp |

### Validation Rules

- **Email**: Must match RFC 5322 email format, max 255 characters, unique across all users
- **Password**: Minimum 6 characters (enforced before hashing)
- **Email Uniqueness**: Case-insensitive comparison to prevent duplicates

### State Transitions

```
[Unregistered] ──(signup)──► [Unverified] ──(email verify)──► [Verified]
                                     │
                              (password reset)
                                     │
                                     ▼
                              [Password Reset Pending] ──(reset complete)──► [Verified]
```

### Lifecycle Rules

1. User created with `emailVerified = null`
2. Email verification sets `emailVerified = now()`
3. Password reset request does not change verification status
4. Account deletion cascades to Profile and Sessions

---

## Entity: UserProfile

**Purpose**: Stores user's software/hardware background for future personalization

### Attributes

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | String (CUID) | Primary Key, Auto-generated | Unique profile identifier |
| userId | String (CUID) | Foreign Key → User.id, Unique, Not Null | Reference to user account |
| softwareLevel | Enum | Not Null | User's software development experience |
| learningGoal | String | Not Null, Max 200 chars | Primary learning objective |
| hardwareAccess | Enum | Not Null | User's access to robotics hardware |
| technicalComfort | Enum | Not Null | User's self-reported technical confidence |
| createdAt | DateTime | Auto-generated | Profile creation timestamp |
| updatedAt | DateTime | Auto-updated | Last profile update timestamp |

### Enumerations

**softwareLevel**:
- `beginner`: Little to no programming experience
- `intermediate`: Comfortable with basic programming concepts
- `advanced`: Experienced developer, comfortable with complex systems

**hardwareAccess**:
- `none`: No access to robotics hardware or simulators
- `basic`: Access to basic components (Arduino, simple sensors) or free simulators
- `advanced`: Access to advanced robotics platforms (ROS, humanoid robots, Gazebo)

**technicalComfort**:
- `low`: Prefers step-by-step guidance, minimal troubleshooting
- `medium`: Comfortable with some independent problem-solving
- `high`: Confident debugging and exploring advanced topics

### Validation Rules

- **User Reference**: Must have exactly one associated User (1:1 relationship)
- **Learning Goal**: Required, 1-200 characters, no HTML/script tags (XSS prevention)
- **Enum Values**: Must be one of the defined enum values (case-sensitive)

### Cascade Behavior

- **On User Delete**: Profile deleted automatically (CASCADE)
- **On User Update**: Profile updatedAt does not change unless profile itself modified

---

## Entity: Session

**Purpose**: Represents an active user authentication session

### Attributes

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | String (CUID) | Primary Key, Auto-generated | Unique session identifier |
| userId | String (CUID) | Foreign Key → User.id, Not Null, Indexed | Reference to authenticated user |
| token | String | Unique, Not Null, 64 chars | Session token stored in HTTP-only cookie |
| expiresAt | DateTime | Not Null | Session expiration timestamp |
| createdAt | DateTime | Auto-generated | Session creation timestamp |

### Validation Rules

- **Token**: Must be cryptographically secure random string (64 characters)
- **Expiration**: `expiresAt` must be in the future, max 7 days from creation
- **User Reference**: Must reference a valid, non-deleted User

### Lifecycle Rules

1. Session created on successful signin with `expiresAt = now() + 7 days`
2. Session validated on each authenticated request
3. Session expired if `expiresAt < now()` (automatic cleanup via cron or lazy deletion)
4. Session deleted on user signout or password reset (security invalidation)
5. Inactivity timeout: Session expired after 7 days without activity

---

## Indexes

```prisma
// Performance indexes
@@index([email])              // User lookup by email
@@index([userId])             // Profile lookup by user
@@index([token])              // Session validation
@@index([expiresAt])          // Session cleanup queries
```

---

## Migration Strategy

### Initial Schema (v1.0.0)

```sql
-- Users table
CREATE TABLE "User" (
  "id" TEXT PRIMARY KEY DEFAULT cuid(),
  "email" TEXT NOT NULL UNIQUE,
  "passwordHash" TEXT NOT NULL,
  "emailVerified" TIMESTAMP(3),
  "createdAt" TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
  "updatedAt" TIMESTAMP(3) NOT NULL
);

-- User profiles table
CREATE TABLE "UserProfile" (
  "id" TEXT PRIMARY KEY DEFAULT cuid(),
  "userId" TEXT NOT NULL UNIQUE,
  "softwareLevel" TEXT NOT NULL,
  "learningGoal" VARCHAR(200) NOT NULL,
  "hardwareAccess" TEXT NOT NULL,
  "technicalComfort" TEXT NOT NULL,
  "createdAt" TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
  "updatedAt" TIMESTAMP(3) NOT NULL,
  CONSTRAINT "UserProfile_userId_fkey" 
    FOREIGN KEY ("userId") REFERENCES "User"("id") ON DELETE CASCADE
);

-- Sessions table
CREATE TABLE "Session" (
  "id" TEXT PRIMARY KEY DEFAULT cuid(),
  "userId" TEXT NOT NULL,
  "token" TEXT NOT NULL UNIQUE,
  "expiresAt" TIMESTAMP(3) NOT NULL,
  "createdAt" TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
  CONSTRAINT "Session_userId_fkey" 
    FOREIGN KEY ("userId") REFERENCES "User"("id") ON DELETE CASCADE
);

-- Indexes
CREATE INDEX "User_email_idx" ON "User"("email");
CREATE INDEX "UserProfile_userId_idx" ON "UserProfile"("userId");
CREATE INDEX "Session_token_idx" ON "Session"("token");
CREATE INDEX "Session_expiresAt_idx" ON "Session"("expiresAt");
```

### Future Migrations (Personalization Phase)

```sql
-- Example: Add personalization preferences (future)
ALTER TABLE "UserProfile" 
  ADD COLUMN "contentPreferences" JSONB DEFAULT '[]',
  ADD COLUMN "lastPersonalizationUpdate" TIMESTAMP(3);
```

---

## Data Access Patterns

### Common Queries

**Get user by email** (signin):
```typescript
const user = await prisma.user.findUnique({
  where: { email: input.email },
  include: { profile: true }
});
```

**Create user with profile** (signup):
```typescript
const user = await prisma.user.create({
  data: {
    email: input.email,
    passwordHash: hashedPassword,
    profile: {
      create: {
        softwareLevel: input.softwareLevel,
        learningGoal: input.learningGoal,
        hardwareAccess: input.hardwareAccess,
        technicalComfort: input.technicalComfort
      }
    }
  },
  include: { profile: true }
});
```

**Get user profile** (profile view):
```typescript
const profile = await prisma.userProfile.findUnique({
  where: { userId: authenticatedUserId }
});
```

**Update profile** (profile edit):
```typescript
const profile = await prisma.userProfile.update({
  where: { userId: authenticatedUserId },
  data: {
    softwareLevel: input.softwareLevel,
    learningGoal: input.learningGoal,
    // ...
  }
});
```

**Delete expired sessions** (cleanup):
```typescript
await prisma.session.deleteMany({
  where: { expiresAt: { lt: new Date() } }
});
```

---

## Security Considerations

1. **Password Hash**: Never expose in queries; always exclude from serialization
2. **Session Token**: Transmitted only via HTTP-only cookie; never in URL or response body
3. **Email Enumeration**: Use generic error messages to prevent user enumeration attacks
4. **Cascade Deletes**: Ensure foreign key constraints enforce CASCADE to prevent orphaned records
5. **Input Validation**: All enum values validated server-side; never trust client input

---

## Testing Data

### Seed Data Example

```typescript
// Test user: beginner with no hardware
{
  email: 'test.beginner@example.com',
  passwordHash: '$2a$10$...', // bcrypt hash of 'password123'
  profile: {
    softwareLevel: 'beginner',
    learningGoal: 'Learn robotics programming from scratch',
    hardwareAccess: 'none',
    technicalComfort: 'low'
  }
}

// Test user: advanced with hardware access
{
  email: 'test.advanced@example.com',
  passwordHash: '$2a$10$...',
  profile: {
    softwareLevel: 'advanced',
    learningGoal: 'Build autonomous humanoid robot behaviors',
    hardwareAccess: 'advanced',
    technicalComfort: 'high'
  }
}
```
