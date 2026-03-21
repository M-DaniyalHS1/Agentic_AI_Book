import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';

interface SignupFormData {
  email: string;
  password: string;
  confirmPassword: string;
}

interface SignupErrors {
  email?: string;
  password?: string;
  confirmPassword?: string;
  general?: string;
}

export default function SignupForm() {
  const [formData, setFormData] = useState<SignupFormData>({
    email: '',
    password: '',
    confirmPassword: '',
  });
  const [errors, setErrors] = useState<SignupErrors>({});
  const [isLoading, setIsLoading] = useState(false);
  const [step, setStep] = useState<'credentials' | 'background'>('credentials');
  const history = useHistory();

  const validateEmail = (email: string): boolean => {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
  };

  const validateCredentials = (): boolean => {
    const newErrors: SignupErrors = {};

    if (!formData.email) {
      newErrors.email = 'Email is required';
    } else if (!validateEmail(formData.email)) {
      newErrors.email = 'Please enter a valid email address';
    }

    if (!formData.password) {
      newErrors.password = 'Password is required';
    } else if (formData.password.length < 6) {
      newErrors.password = 'Password must be at least 6 characters';
    }

    if (!formData.confirmPassword) {
      newErrors.confirmPassword = 'Please confirm your password';
    } else if (formData.password !== formData.confirmPassword) {
      newErrors.confirmPassword = 'Passwords do not match';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleCredentialsSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    if (!validateCredentials()) {
      return;
    }

    setIsLoading(true);
    setErrors({});

    try {
      // Use hardcoded API URL for local development
      const apiUrl = 'http://localhost:8001';
      const response = await fetch(`${apiUrl}/api/auth/signup`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include',
        body: JSON.stringify({
          email: formData.email,
          password: formData.password,
          // These will be sent in the next step
          software_level: 'beginner',
          learning_goal: 'To be updated',
          hardware_access: 'none',
          technical_comfort: 'medium',
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        if (data.detail?.code === 'USER_EXISTS') {
          setErrors({ email: 'This email is already registered. Please sign in instead.' });
        } else {
          setErrors({ general: data.detail?.message || 'Signup failed. Please try again.' });
        }
        return;
      }

      // Credentials accepted, move to background questions
      setStep('background');
    } catch (error) {
      console.error('Signup error:', error);
      setErrors({ general: 'Network error. Please check your connection and try again.' });
    } finally {
      setIsLoading(false);
    }
  };

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));
    // Clear error when user starts typing
    if (errors[name as keyof SignupErrors]) {
      setErrors((prev) => ({ ...prev, [name]: undefined }));
    }
  };

  if (step === 'background') {
    // Redirect to background questions component
    window.location.href = '/signup/background';
    return null;
  }

  return (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--6 col--offset-3">
          <div className="card">
            <div className="card__header">
              <h1>Create Your Account</h1>
              <p>Join Agent Book Factory to access personalized AI-powered robotics learning</p>
            </div>
            <div className="card__body">
              {errors.general && (
                <div className="alert alert--danger margin-bottom--md" role="alert">
                  {errors.general}
                </div>
              )}

              <form onSubmit={handleCredentialsSubmit}>
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
                    placeholder="Minimum 6 characters"
                    autoComplete="new-password"
                  />
                  {errors.password && (
                    <small className="color--danger">{errors.password}</small>
                  )}
                </div>

                <div className="margin-bottom--md">
                  <label htmlFor="confirmPassword" className="margin-bottom--sm">
                    Confirm Password
                  </label>
                  <input
                    type="password"
                    id="confirmPassword"
                    name="confirmPassword"
                    className="input"
                    style={{
                      width: '100%',
                      padding: '0.5rem',
                      border: errors.confirmPassword ? '2px solid var(--ifm-color-danger)' : '1px solid var(--ifm-color-emphasis-300)',
                      borderRadius: '4px',
                    }}
                    value={formData.confirmPassword}
                    onChange={handleInputChange}
                    disabled={isLoading}
                    placeholder="Re-enter your password"
                    autoComplete="new-password"
                  />
                  {errors.confirmPassword && (
                    <small className="color--danger">{errors.confirmPassword}</small>
                  )}
                </div>

                <button
                  type="submit"
                  className="button button--primary button--block margin-top--md"
                  disabled={isLoading}
                >
                  {isLoading ? 'Creating Account...' : 'Continue to Background Questions'}
                </button>
              </form>
            </div>
            <div className="card__footer text--center">
              <p>
                Already have an account?{' '}
                <a href="/signin">Sign in</a>
              </p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
