import React, { useState, useEffect } from 'react';
import { useHistory } from '@docusaurus/router';

interface BackgroundData {
  software_level: 'beginner' | 'intermediate' | 'advanced';
  learning_goal: string;
  hardware_access: 'none' | 'basic' | 'advanced';
  technical_comfort: 'low' | 'medium' | 'high';
}

interface BackgroundErrors {
  learning_goal?: string;
  general?: string;
}

export default function BackgroundQuestions() {
  const history = useHistory();
  const [formData, setFormData] = useState<BackgroundData>({
    software_level: 'beginner',
    learning_goal: '',
    hardware_access: 'none',
    technical_comfort: 'medium',
  });
  const [errors, setErrors] = useState<BackgroundErrors>({});
  const [isLoading, setIsLoading] = useState(false);
  const [currentQuestion, setCurrentQuestion] = useState(0);

  const questions = [
    {
      id: 'software_level',
      question: 'What is your software development experience level?',
      type: 'select',
      options: [
        { value: 'beginner', label: 'Beginner - Little to no programming experience' },
        { value: 'intermediate', label: 'Intermediate - Comfortable with basic programming concepts' },
        { value: 'advanced', label: 'Advanced - Experienced developer, comfortable with complex systems' },
      ],
    },
    {
      id: 'learning_goal',
      question: 'What is your primary learning goal?',
      type: 'text',
      placeholder: 'e.g., Learn robotics programming from scratch, Build autonomous robots...',
      maxLength: 200,
    },
    {
      id: 'hardware_access',
      question: 'What robotics hardware do you have access to?',
      type: 'select',
      options: [
        { value: 'none', label: 'None - No access to robotics hardware or simulators' },
        { value: 'basic', label: 'Basic - Arduino, simple sensors, or free simulators' },
        { value: 'advanced', label: 'Advanced - ROS, humanoid robots, Gazebo, or similar' },
      ],
    },
    {
      id: 'technical_comfort',
      question: 'How comfortable are you with technical troubleshooting?',
      type: 'select',
      options: [
        { value: 'low', label: 'Low - Prefer step-by-step guidance, minimal troubleshooting' },
        { value: 'medium', label: 'Medium - Comfortable with some independent problem-solving' },
        { value: 'high', label: 'High - Confident debugging and exploring advanced topics' },
      ],
    },
  ];

  const validateCurrentQuestion = (): boolean => {
    const currentQ = questions[currentQuestion];
    
    if (currentQ.id === 'learning_goal') {
      if (!formData.learning_goal.trim()) {
        setErrors({ learning_goal: 'Please tell us your learning goal' });
        return false;
      }
      if (formData.learning_goal.trim().length > 200) {
        setErrors({ learning_goal: 'Learning goal must be 200 characters or less' });
        return false;
      }
    }
    
    return true;
  };

  const handleNext = () => {
    if (!validateCurrentQuestion()) {
      return;
    }
    
    setErrors({});
    
    if (currentQuestion < questions.length - 1) {
      setCurrentQuestion(currentQuestion + 1);
    } else {
      handleSubmit();
    }
  };

  const handleBack = () => {
    if (currentQuestion > 0) {
      setCurrentQuestion(currentQuestion - 1);
      setErrors({});
    }
  };

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));
    if (errors[name as keyof BackgroundErrors]) {
      setErrors((prev) => ({ ...prev, [name as keyof BackgroundErrors]: undefined }));
    }
  };

  const handleSubmit = async () => {
    setIsLoading(true);
    setErrors({});

    try {
      // Update the profile with background data
      // Use hardcoded API URL for local development - using 'localhost' not '127.0.0.1' for cookie consistency
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
        throw new Error(data.detail?.message || 'Failed to save profile');
      }

      // Success - redirect to dashboard
      history.push('/dashboard');
    } catch (error) {
      console.error('Profile update error:', error);
      setErrors({ general: 'Failed to save your responses. Please try again.' });
    } finally {
      setIsLoading(false);
    }
  };

  const renderQuestion = () => {
    const currentQ = questions[currentQuestion];

    if (currentQ.type === 'select') {
      return (
        <select
          name={currentQ.id}
          value={formData[currentQ.id as keyof BackgroundData]}
          onChange={handleInputChange}
          className="input"
          style={{
            width: '100%',
            padding: '0.75rem',
            border: '1px solid var(--ifm-color-emphasis-300)',
            borderRadius: '4px',
            fontSize: '1rem',
          }}
        >
          {currentQ.options.map((opt) => (
            <option key={opt.value} value={opt.value}>
              {opt.label}
            </option>
          ))}
        </select>
      );
    }

    if (currentQ.type === 'text') {
      return (
        <>
          <textarea
            name={currentQ.id}
            value={formData.learning_goal}
            onChange={handleInputChange}
            placeholder={currentQ.placeholder}
            maxLength={currentQ.maxLength}
            rows={4}
            className="input"
            style={{
              width: '100%',
              padding: '0.75rem',
              border: errors.learning_goal
                ? '2px solid var(--ifm-color-danger)'
                : '1px solid var(--ifm-color-emphasis-300)',
              borderRadius: '4px',
              fontSize: '1rem',
              resize: 'vertical',
            }}
          />
          <div style={{ textAlign: 'right', fontSize: '0.875rem', color: 'var(--ifm-color-emphasis-600)' }}>
            {formData.learning_goal.length} / {currentQ.maxLength} characters
          </div>
          {errors.learning_goal && (
            <small className="color--danger">{errors.learning_goal}</small>
          )}
        </>
      );
    }

    return null;
  };

  const currentQuestionData = questions[currentQuestion];

  return (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--8 col--offset-2">
          <div className="card">
            <div className="card__header">
              <h2>Tell Us About Yourself</h2>
              <p className="margin-bottom--md">
                Answer these 4 questions to help us personalize your learning experience
              </p>
              <div className="progress-bar" style={{ height: '4px' }}>
                <div
                  className="progress-bar__progress"
                  style={{
                    width: `${((currentQuestion + 1) / questions.length) * 100}%`,
                    backgroundColor: 'var(--ifm-color-primary)',
                    height: '100%',
                    transition: 'width 0.3s ease',
                  }}
                />
              </div>
              <small className="margin-top--sm">
                Question {currentQuestion + 1} of {questions.length}
              </small>
            </div>

            <div className="card__body">
              {errors.general && (
                <div className="alert alert--danger margin-bottom--md" role="alert">
                  {errors.general}
                </div>
              )}

              <div className="margin-vert--lg">
                <h3 className="margin-bottom--md">{currentQuestionData.question}</h3>
                {renderQuestion()}
              </div>
            </div>

            <div className="card__footer">
              <div className="row">
                <div className="col col--6">
                  {currentQuestion > 0 && (
                    <button
                      type="button"
                      onClick={handleBack}
                      className="button button--secondary button--block"
                      disabled={isLoading}
                    >
                      ← Back
                    </button>
                  )}
                </div>
                <div className="col col--6">
                  <button
                    type="button"
                    onClick={handleNext}
                    className="button button--primary button--block"
                    disabled={isLoading}
                  >
                    {isLoading
                      ? 'Saving...'
                      : currentQuestion === questions.length - 1
                      ? 'Complete Setup'
                      : 'Next →'}
                  </button>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>

      <style>{`
        .progress-bar {
          background-color: var(--ifm-color-emphasis-200);
          border-radius: 2px;
          overflow: hidden;
        }
      `}</style>
    </div>
  );
}
