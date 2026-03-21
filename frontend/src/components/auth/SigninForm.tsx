import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';

interface SigninFormData {
  email: string;
  password: string;
}

interface SigninErrors {
  email?: string;
  password?: string;
  general?: string;
}

export default function SigninForm() {
  const [formData, setFormData] = useState<SigninFormData>({
    email: '',
    password: '',
  });
  const [errors, setErrors] = useState<SigninErrors>({});
  const [isLoading, setIsLoading] = useState(false);
  const history = useHistory();

  const validateEmail = (email: string): boolean => {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
  };

  const validateForm = (): boolean => {
    const newErrors: SigninErrors = {};

    if (!formData.email) {
      newErrors.email = 'Email is required';
    } else if (!validateEmail(formData.email)) {
      newErrors.email = 'Please enter a valid email address';
    }

    if (!formData.password) {
      newErrors.password = 'Password is required';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    if (!validateForm()) {
      return;
    }

    setIsLoading(true);
    setErrors({});

    try {
      // Use hardcoded API URL for local development - using 'localhost' not '127.0.0.1' for cookie consistency
      const apiUrl = 'http://localhost:8001';
      const response = await fetch(`${apiUrl}/api/auth/signin`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include',
        body: JSON.stringify({
          email: formData.email,
          password: formData.password,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        if (data.detail?.code === 'INVALID_CREDENTIALS') {
          setErrors({ general: 'Invalid email or password. Please try again.' });
        } else {
          setErrors({ general: data.detail?.message || 'Signin failed. Please try again.' });
        }
        return;
      }

      // Success - redirect to dashboard
      history.push('/dashboard');
    } catch (error) {
      console.error('Signin error:', error);
      setErrors({ general: 'Network error. Please check your connection and try again.' });
    } finally {
      setIsLoading(false);
    }
  };

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));
    if (errors[name as keyof SigninErrors]) {
      setErrors((prev) => ({ ...prev, [name]: undefined }));
    }
  };

  return (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--6 col--offset-3">
          <div className="card">
            <div className="card__header">
              <h1>Sign In</h1>
              <p>Welcome back! Sign in to continue your learning journey.</p>
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
                    value={formData.email}
                    onChange={handleInputChange}
                    disabled={isLoading}
                    placeholder="you@example.com"
                    autoComplete="email"
                  />
                  {errors.email && (
                    <small className="color--danger">{errors.email}</small>
                  )}
                </div>

                <div className="margin-bottom--md">
                  <label htmlFor="password" className="margin-bottom--sm">
                    Password
                  </label>
                  <input
                    type="password"
                    id="password"
                    name="password"
                    className="input"
                    style={{
                      width: '100%',
                      padding: '0.5rem',
                      border: errors.password ? '2px solid var(--ifm-color-danger)' : '1px solid var(--ifm-color-emphasis-300)',
                      borderRadius: '4px',
                    }}
                    value={formData.password}
                    onChange={handleInputChange}
                    disabled={isLoading}
                    placeholder="Your password"
                    autoComplete="current-password"
                  />
                  {errors.password && (
                    <small className="color--danger">{errors.password}</small>
                  )}
                </div>

                <div className="text--right margin-bottom--md">
                  <a href="/reset-password" style={{ fontSize: '0.875rem' }}>
                    Forgot your password?
                  </a>
                </div>

                <button
                  type="submit"
                  className="button button--primary button--block margin-top--md"
                  disabled={isLoading}
                >
                  {isLoading ? 'Signing In...' : 'Sign In'}
                </button>
              </form>
            </div>
            <div className="card__footer text--center">
              <p>
                Don't have an account?{' '}
                <a href="/signup">Sign up</a>
              </p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
