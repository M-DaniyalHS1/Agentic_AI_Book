import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';

interface ResetPasswordConfirmErrors {
  new_password?: string;
  confirm_password?: string;
  general?: string;
}

export default function ResetPasswordConfirm() {
  const location = useLocation();
  const [token, setToken] = useState('');
  const [newPassword, setNewPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [errors, setErrors] = useState<ResetPasswordConfirmErrors>({});
  const [isLoading, setIsLoading] = useState(false);
  const [isSuccess, setIsSuccess] = useState(false);
  const [isValidToken, setIsValidToken] = useState(true);

  useEffect(() => {
    // Extract token from URL query params
    const params = new URLSearchParams(location.search);
    const tokenParam = params.get('token');
    
    if (tokenParam) {
      setToken(tokenParam);
    } else {
      setIsValidToken(false);
    }
  }, [location]);

  const validateForm = (): boolean => {
    const newErrors: ResetPasswordConfirmErrors = {};

    if (!newPassword) {
      newErrors.new_password = 'Password is required';
    } else if (newPassword.length < 6) {
      newErrors.new_password = 'Password must be at least 6 characters';
    }

    if (!confirmPassword) {
      newErrors.confirm_password = 'Please confirm your password';
    } else if (newPassword !== confirmPassword) {
      newErrors.confirm_password = 'Passwords do not match';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    if (!token) {
      setErrors({ general: 'Reset token is missing' });
      return;
    }

    if (!validateForm()) {
      return;
    }

    setIsLoading(true);
    setErrors({});

    try {
      // Use hardcoded API URL for local development - using 'localhost' not '127.0.0.1' for cookie consistency
      const apiUrl = 'http://localhost:8001';
      const response = await fetch(`${apiUrl}/api/auth/reset-password/confirm`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          token,
          new_password: newPassword,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        if (data.detail?.code === 'INVALID_TOKEN') {
          setErrors({ general: 'This reset link is invalid or has expired. Please request a new one.' });
          setIsValidToken(false);
        } else {
          setErrors({ general: data.detail?.message || 'Reset failed. Please try again.' });
        }
        return;
      }

      setIsSuccess(true);
    } catch (error) {
      console.error('Reset confirm error:', error);
      setErrors({ general: 'Network error. Please check your connection and try again.' });
    } finally {
      setIsLoading(false);
    }
  };

  if (!isValidToken && !isSuccess) {
    return (
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__header text--center">
                <h2>Invalid Reset Link</h2>
              </div>
              <div className="card__body text--center">
                <p>
                  This password reset link is invalid or has expired. 
                  Please request a new one.
                </p>
              </div>
              <div className="card__footer">
                <a href="/reset-password" className="button button--primary button--block">
                  Request New Reset Link
                </a>
              </div>
            </div>
          </div>
        </div>
      </div>
    );
  }

  if (isSuccess) {
    return (
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__header text--center">
                <h2>Password Reset Successful! 🎉</h2>
              </div>
              <div className="card__body text--center">
                <p>Your password has been reset successfully.</p>
                <p className="margin-top--md">
                  You can now sign in with your new password.
                </p>
              </div>
              <div className="card__footer">
                <a href="/signin" className="button button--primary button--block">
                  Go to Sign In
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
              <h1>Create New Password</h1>
              <p>Enter your new password below.</p>
            </div>
            <div className="card__body">
              {errors.general && (
                <div className="alert alert--danger margin-bottom--md" role="alert">
                  {errors.general}
                </div>
              )}

              <form onSubmit={handleSubmit}>
                <div className="margin-bottom--md">
                  <label htmlFor="newPassword" className="margin-bottom--sm">
                    New Password
                  </label>
                  <input
                    type="password"
                    id="newPassword"
                    name="newPassword"
                    className="input"
                    style={{
                      width: '100%',
                      padding: '0.5rem',
                      border: errors.new_password ? '2px solid var(--ifm-color-danger)' : '1px solid var(--ifm-color-emphasis-300)',
                      borderRadius: '4px',
                    }}
                    value={newPassword}
                    onChange={(e) => setNewPassword(e.target.value)}
                    disabled={isLoading}
                    placeholder="Minimum 6 characters"
                    autoComplete="new-password"
                  />
                  {errors.new_password && (
                    <small className="color--danger">{errors.new_password}</small>
                  )}
                </div>

                <div className="margin-bottom--md">
                  <label htmlFor="confirmPassword" className="margin-bottom--sm">
                    Confirm New Password
                  </label>
                  <input
                    type="password"
                    id="confirmPassword"
                    name="confirmPassword"
                    className="input"
                    style={{
                      width: '100%',
                      padding: '0.5rem',
                      border: errors.confirm_password ? '2px solid var(--ifm-color-danger)' : '1px solid var(--ifm-color-emphasis-300)',
                      borderRadius: '4px',
                    }}
                    value={confirmPassword}
                    onChange={(e) => setConfirmPassword(e.target.value)}
                    disabled={isLoading}
                    placeholder="Re-enter your new password"
                    autoComplete="new-password"
                  />
                  {errors.confirm_password && (
                    <small className="color--danger">{errors.confirm_password}</small>
                  )}
                </div>

                <button
                  type="submit"
                  className="button button--primary button--block margin-top--md"
                  disabled={isLoading}
                >
                  {isLoading ? 'Resetting...' : 'Reset Password'}
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
