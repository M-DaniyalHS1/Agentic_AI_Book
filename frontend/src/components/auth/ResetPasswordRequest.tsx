import React, { useState } from 'react';

interface ResetPasswordRequestErrors {
  email?: string;
  general?: string;
}

export default function ResetPasswordRequest() {
  const [email, setEmail] = useState('');
  const [errors, setErrors] = useState<ResetPasswordRequestErrors>({});
  const [isLoading, setIsLoading] = useState(false);
  const [isSubmitted, setIsSubmitted] = useState(false);

  const validateEmail = (email: string): boolean => {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    const newErrors: ResetPasswordRequestErrors = {};
    
    if (!email) {
      newErrors.email = 'Email is required';
    } else if (!validateEmail(email)) {
      newErrors.email = 'Please enter a valid email address';
    }
    
    if (Object.keys(newErrors).length > 0) {
      setErrors(newErrors);
      return;
    }

    setIsLoading(true);
    setErrors({});

    try {
      // Use hardcoded API URL for local development - using 'localhost' not '127.0.0.1' for cookie consistency
      const apiUrl = 'http://localhost:8001';
      const response = await fetch(`${apiUrl}/api/auth/reset-password`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email }),
      });

      const data = await response.json();

      if (!response.ok) {
        setErrors({ general: data.detail?.message || 'Request failed. Please try again.' });
        return;
      }

      // Always show success message (even if email doesn't exist - security)
      setIsSubmitted(true);
    } catch (error) {
      console.error('Reset request error:', error);
      setErrors({ general: 'Network error. Please check your connection and try again.' });
    } finally {
      setIsLoading(false);
    }
  };

  if (isSubmitted) {
    return (
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__header text--center">
                <h2>Check Your Email</h2>
              </div>
              <div className="card__body text--center">
                <p>
                  If an account exists with <strong>{email}</strong>, you will receive 
                  a password reset link shortly.
                </p>
                <p className="margin-top--md">
                  Didn't receive an email? Check your spam folder or{' '}
                  <a href="/reset-password" onClick={() => setIsSubmitted(false)}>
                    try again
                  </a>.
                </p>
              </div>
              <div className="card__footer">
                <a href="/signin" className="button button--primary button--block">
                  Back to Sign In
                </a>
              </div>
            </div>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--6 col--offset-3">
          <div className="card">
            <div className="card__header">
              <h1>Reset Password</h1>
              <p>Enter your email address and we'll send you a link to reset your password.</p>
            </div>
            <div className="card__body">
              {errors.general && (
                <div className="alert alert--danger margin-bottom--md" role="alert">
                  {errors.general}
                </div>
              )}

              <form onSubmit={handleSubmit}>
                <div className="margin-bottom--md">
                  <label htmlFor="email" className="margin-bottom--sm">
                    Email Address
                  </label>
                  <input
                    type="email"
                    id="email"
                    name="email"
                    className="input"
                    style={{
                      width: '100%',
                      padding: '0.5rem',
                      border: errors.email ? '2px solid var(--ifm-color-danger)' : '1px solid var(--ifm-color-emphasis-300)',
                      borderRadius: '4px',
                    }}
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
                    disabled={isLoading}
                    placeholder="you@example.com"
                    autoComplete="email"
                  />
                  {errors.email && (
                    <small className="color--danger">{errors.email}</small>
                  )}
                </div>

                <button
                  type="submit"
                  className="button button--primary button--block margin-top--md"
                  disabled={isLoading}
                >
                  {isLoading ? 'Sending...' : 'Send Reset Link'}
                </button>
              </form>
            </div>
            <div className="card__footer text--center">
              <p>
                Remember your password?{' '}
                <a href="/signin">Sign in</a>
              </p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
