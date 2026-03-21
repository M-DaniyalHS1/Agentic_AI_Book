import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';

interface UserProfile {
  user_id: string;
  email: string;
  software_level: string;
  learning_goal: string;
  hardware_access: string;
  technical_comfort: string;
}

export default function DashboardPage() {
  const history = useHistory();
  const [profile, setProfile] = useState<UserProfile | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [isAuthenticated, setIsAuthenticated] = useState(false);

  useEffect(() => {
    checkAuth();
  }, []);

  const checkAuth = async () => {
    try {
      const apiUrl = 'http://localhost:8001';
      const response = await fetch(`${apiUrl}/api/auth/me`, {
        credentials: 'include',
      });
      const data = await response.json();

      if (data.authenticated) {
        setIsAuthenticated(true);
        // Fetch full profile
        fetchProfile();
      } else {
        setIsAuthenticated(false);
        setIsLoading(false);
      }
    } catch (error) {
      console.error('Auth check error:', error);
      setIsAuthenticated(false);
      setIsLoading(false);
    }
  };

  const fetchProfile = async () => {
    try {
      const apiUrl = 'http://localhost:8001';
      const response = await fetch(`${apiUrl}/api/auth/profile`, {
        credentials: 'include',
      });
      
      if (response.ok) {
        const data = await response.json();
        setProfile(data);
      }
    } catch (error) {
      console.error('Profile fetch error:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSignOut = async () => {
    try {
      const apiUrl = 'http://localhost:8001';
      await fetch(`${apiUrl}/api/auth/signout`, {
        method: 'POST',
        credentials: 'include',
      });
      history.push('/');
    } catch (error) {
      console.error('Sign out error:', error);
    }
  };

  if (isLoading) {
    return (
      <Layout title="Dashboard" description="Your learning dashboard">
        <main className="container margin-vert--xl text--center">
          <div className="row">
            <div className="col col--12">
              <h2>Loading...</h2>
            </div>
          </div>
        </main>
      </Layout>
    );
  }

  if (!isAuthenticated) {
    return (
      <Layout title="Dashboard" description="Your learning dashboard">
        <main className="container margin-vert--xl">
          <div className="row">
            <div className="col col--6 col--offset-3">
              <div className="card">
                <div className="card__header text--center">
                  <h2>Sign In Required</h2>
                </div>
                <div className="card__body text--center">
                  <p>Please sign in to access your dashboard.</p>
                </div>
                <div className="card__footer">
                  <button
                    className="button button--primary button--block"
                    onClick={() => history.push('/signin')}
                  >
                    Sign In
                  </button>
                </div>
              </div>
            </div>
          </div>
        </main>
      </Layout>
    );
  }

  return (
    <Layout title="Dashboard" description="Your learning dashboard">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <div className="hero hero--primary margin-bottom--lg">
              <div className="container">
                <h1 className="hero__title">Welcome to Agent Book Factory!</h1>
                <p className="hero__subtitle">
                  Your personalized AI-powered robotics learning platform
                </p>
                <button
                  className="button button--secondary margin-top--md"
                  onClick={handleSignOut}
                >
                  Sign Out
                </button>
              </div>
            </div>
          </div>
        </div>

        {profile && (
          <div className="row">
            <div className="col col--6">
              <div className="card margin-bottom--md">
                <div className="card__header">
                  <h3>Your Profile</h3>
                </div>
                <div className="card__body">
                  <p><strong>Email:</strong> {profile.email}</p>
                  <p><strong>Experience Level:</strong> {profile.software_level}</p>
                  <p><strong>Hardware Access:</strong> {profile.hardware_access}</p>
                  <p><strong>Technical Comfort:</strong> {profile.technical_comfort}</p>
                </div>
                <div className="card__footer">
                  <button
                    className="button button--outline button--block"
                    onClick={() => history.push('/profile')}
                  >
                    Edit Profile
                  </button>
                </div>
              </div>
            </div>

            <div className="col col--6">
              <div className="card margin-bottom--md">
                <div className="card__header">
                  <h3>Learning Goal</h3>
                </div>
                <div className="card__body">
                  <p>{profile.learning_goal}</p>
                </div>
              </div>
            </div>
          </div>
        )}

        <div className="row">
          <div className="col col--12">
            <div className="card">
              <div className="card__header">
                <h3>Get Started</h3>
              </div>
              <div className="card__body">
                <p>Explore the textbook and start learning with AI-powered assistance:</p>
                <button
                  className="button button--primary"
                  onClick={() => history.push('/intro')}
                >
                  Browse Textbook
                </button>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
