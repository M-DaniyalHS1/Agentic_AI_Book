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

export default function ProfilePage() {
  const history = useHistory();
  const [profile, setProfile] = useState<UserProfile | null>(null);
  const [isEditing, setIsEditing] = useState(false);
  const [isLoading, setIsLoading] = useState(true);
  const [isSaving, setIsSaving] = useState(false);
  const [errors, setErrors] = useState<Record<string, string>>({});

  const [formData, setFormData] = useState({
    software_level: 'beginner',
    learning_goal: '',
    hardware_access: 'none',
    technical_comfort: 'medium',
  });

  useEffect(() => {
    fetchProfile();
  }, []);

  const fetchProfile = async () => {
    try {
      const apiUrl = 'http://localhost:8001';
      const response = await fetch(`${apiUrl}/api/auth/profile`, {
        credentials: 'include',
      });

      if (!response.ok) {
        if (response.status === 401) {
          history.push('/signin');
          return;
        }
        throw new Error('Failed to fetch profile');
      }

      const data = await response.json();
      setProfile(data);
      setFormData({
        software_level: data.software_level,
        learning_goal: data.learning_goal,
        hardware_access: data.hardware_access,
        technical_comfort: data.technical_comfort,
      });
    } catch (error) {
      console.error('Profile fetch error:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));
    if (errors[name]) {
      setErrors((prev) => ({ ...prev, [name]: '' }));
    }
  };

  const validateForm = (): boolean => {
    const newErrors: Record<string, string> = {};

    if (!formData.learning_goal.trim()) {
      newErrors.learning_goal = 'Learning goal is required';
    } else if (formData.learning_goal.length > 200) {
      newErrors.learning_goal = 'Learning goal must be 200 characters or less';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validateForm()) {
      return;
    }

    setIsSaving(true);
    setErrors({});

    try {
      const apiUrl = 'http://localhost:8001';
      const response = await fetch(`${apiUrl}/api/auth/profile`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include',
        body: JSON.stringify(formData),
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail?.message || 'Failed to update profile');
      }

      const updatedProfile = await response.json();
      setProfile(updatedProfile);
      setIsEditing(false);
    } catch (error) {
      console.error('Profile update error:', error);
      setErrors({ general: 'Failed to update profile. Please try again.' });
    } finally {
      setIsSaving(false);
    }
  };

  if (isLoading) {
    return (
      <Layout title="Profile" description="Your profile">
        <main className="container margin-vert--xl text--center">
          <h2>Loading profile...</h2>
        </main>
      </Layout>
    );
  }

  if (!profile) {
    return null; // Will redirect to signin
  }

  return (
    <Layout title="Profile" description="Your profile">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <div className="card">
              <div className="card__header">
                <h1>Your Profile</h1>
                <p>Manage your background information and learning preferences</p>
              </div>

              <div className="card__body">
                {errors.general && (
                  <div className="alert alert--danger margin-bottom--md" role="alert">
                    {errors.general}
                  </div>
                )}

                {isEditing ? (
                  <form onSubmit={handleSubmit}>
                    <div className="margin-bottom--md">
                      <label htmlFor="email" className="margin-bottom--sm">
                        Email
                      </label>
                      <input
                        type="email"
                        id="email"
                        className="input"
                        style={{
                          width: '100%',
                          padding: '0.5rem',
                          border: '1px solid var(--ifm-color-emphasis-300)',
                          borderRadius: '4px',
                          backgroundColor: 'var(--ifm-color-emphasis-100)',
                        }}
                        value={profile.email}
                        disabled
                      />
                      <small className="color--emphasized">Email cannot be changed</small>
                    </div>

                    <div className="margin-bottom--md">
                      <label htmlFor="software_level" className="margin-bottom--sm">
                        Software Experience Level
                      </label>
                      <select
                        id="software_level"
                        name="software_level"
                        className="input"
                        style={{
                          width: '100%',
                          padding: '0.5rem',
                          border: '1px solid var(--ifm-color-emphasis-300)',
                          borderRadius: '4px',
                        }}
                        value={formData.software_level}
                        onChange={handleInputChange}
                      >
                        <option value="beginner">Beginner - Little to no programming experience</option>
                        <option value="intermediate">Intermediate - Comfortable with basic programming concepts</option>
                        <option value="advanced">Advanced - Experienced developer</option>
                      </select>
                    </div>

                    <div className="margin-bottom--md">
                      <label htmlFor="learning_goal" className="margin-bottom--sm">
                        Learning Goal
                      </label>
                      <textarea
                        id="learning_goal"
                        name="learning_goal"
                        className="input"
                        style={{
                          width: '100%',
                          padding: '0.5rem',
                          border: errors.learning_goal 
                            ? '2px solid var(--ifm-color-danger)' 
                            : '1px solid var(--ifm-color-emphasis-300)',
                          borderRadius: '4px',
                          resize: 'vertical',
                        }}
                        value={formData.learning_goal}
                        onChange={handleInputChange}
                        rows={4}
                        placeholder="What do you want to learn?"
                      />
                      <div style={{ textAlign: 'right', fontSize: '0.875rem', color: 'var(--ifm-color-emphasis-600)' }}>
                        {formData.learning_goal.length} / 200 characters
                      </div>
                      {errors.learning_goal && (
                        <small className="color--danger">{errors.learning_goal}</small>
                      )}
                    </div>

                    <div className="margin-bottom--md">
                      <label htmlFor="hardware_access" className="margin-bottom--sm">
                        Hardware Access
                      </label>
                      <select
                        id="hardware_access"
                        name="hardware_access"
                        className="input"
                        style={{
                          width: '100%',
                          padding: '0.5rem',
                          border: '1px solid var(--ifm-color-emphasis-300)',
                          borderRadius: '4px',
                        }}
                        value={formData.hardware_access}
                        onChange={handleInputChange}
                      >
                        <option value="none">None - No access to robotics hardware</option>
                        <option value="basic">Basic - Arduino, simple sensors, or free simulators</option>
                        <option value="advanced">Advanced - ROS, humanoid robots, Gazebo</option>
                      </select>
                    </div>

                    <div className="margin-bottom--md">
                      <label htmlFor="technical_comfort" className="margin-bottom--sm">
                        Technical Comfort Level
                      </label>
                      <select
                        id="technical_comfort"
                        name="technical_comfort"
                        className="input"
                        style={{
                          width: '100%',
                          padding: '0.5rem',
                          border: '1px solid var(--ifm-color-emphasis-300)',
                          borderRadius: '4px',
                        }}
                        value={formData.technical_comfort}
                        onChange={handleInputChange}
                      >
                        <option value="low">Low - Prefer step-by-step guidance</option>
                        <option value="medium">Medium - Comfortable with some troubleshooting</option>
                        <option value="high">High - Confident debugging and advanced topics</option>
                      </select>
                    </div>

                    <div className="row margin-top--lg">
                      <div className="col col--6">
                        <button
                          type="button"
                          className="button button--secondary button--block"
                          onClick={() => {
                            setIsEditing(false);
                            setFormData({
                              software_level: profile.software_level,
                              learning_goal: profile.learning_goal,
                              hardware_access: profile.hardware_access,
                              technical_comfort: profile.technical_comfort,
                            });
                            setErrors({});
                          }}
                          disabled={isSaving}
                        >
                          Cancel
                        </button>
                      </div>
                      <div className="col col--6">
                        <button
                          type="submit"
                          className="button button--primary button--block"
                          disabled={isSaving}
                        >
                          {isSaving ? 'Saving...' : 'Save Changes'}
                        </button>
                      </div>
                    </div>
                  </form>
                ) : (
                  <>
                    <div className="margin-bottom--md">
                      <strong>Email:</strong>
                      <p>{profile.email}</p>
                    </div>

                    <div className="margin-bottom--md">
                      <strong>Software Experience Level:</strong>
                      <p className="text--capitalize">{profile.software_level}</p>
                    </div>

                    <div className="margin-bottom--md">
                      <strong>Learning Goal:</strong>
                      <p>{profile.learning_goal}</p>
                    </div>

                    <div className="margin-bottom--md">
                      <strong>Hardware Access:</strong>
                      <p className="text--capitalize">{profile.hardware_access}</p>
                    </div>

                    <div className="margin-bottom--md">
                      <strong>Technical Comfort Level:</strong>
                      <p className="text--capitalize">{profile.technical_comfort}</p>
                    </div>

                    <button
                      className="button button--primary button--block margin-top--md"
                      onClick={() => setIsEditing(true)}
                    >
                      Edit Profile
                    </button>
                  </>
                )}
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
