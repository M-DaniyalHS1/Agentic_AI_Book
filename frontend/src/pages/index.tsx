import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import ThemedImage from '@theme/ThemedImage';

import styles from './index.module.css';

function HeroSection() {
  return (
    <header className={styles.hero}>
      <div className={styles.heroContainer}>
        <div className={styles.heroBadge}>
          🤖 Panaversity Presents
        </div>
        
        <h1 className={styles.heroTitle}>
          <span className={styles.gradientText}>
            AI-Native Digital Textbook
          </span>
          <br />
          on Physical AI & Humanoid Robotics
        </h1>
        
        <p className={styles.heroSubtitle}>
          The world's first AI-native textbook where the book is machine-readable 
          and an AI tutor understands the content better than any human TA.
        </p>
        
        <div className={styles.heroDescription}>
          Learn Digital AI → Physical World → Human Interaction with 
          runnable simulations, AI-powered tutoring, and reproducible robotics code.
        </div>
        
        <div className={styles.ctaButtons}>
          <Link
            to="/docs/intro"
            className={clsx(styles.button, styles.buttonPrimary)}
          >
            📖 Get Started
          </Link>
          <Link
            to="/signup"
            className={clsx(styles.button, styles.buttonSecondary)}
          >
            ✨ Sign Up
          </Link>
          <Link
            to="/signin"
            className={clsx(styles.button, styles.buttonOutline)}
          >
            🔐 Sign In
          </Link>
        </div>
        
        <div className={styles.socialProof}>
          <div className={styles.socialProofText}>
            🎓 Free for students | 100% Open Source
          </div>
        </div>
      </div>
    </header>
  );
}

function FeatureCard({ icon, title, description }) {
  return (
    <div className={styles.featureCard}>
      <div className={styles.featureIcon}>{icon}</div>
      <h3 className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
    </div>
  );
}

function FeaturesSection() {
  const features = [
    {
      icon: '🤖',
      title: 'AI-Powered Tutor',
      description: 'Context-aware tutoring that answers questions based solely on textbook content with proper citations.',
    },
    {
      icon: '📚',
      title: 'Machine-Readable Content',
      description: 'Every concept is structured for AI understanding, enabling intelligent search and explanations.',
    },
    {
      icon: '🧪',
      title: 'Runnable Simulations',
      description: 'Hands-on labs with ROS 2, Gazebo, and NVIDIA Isaac Sim - all code is documented and tested.',
    },
    {
      icon: '🌐',
      title: 'Open & Accessible',
      description: 'Free for everyone, mobile-friendly, and designed for low-bandwidth connections.',
    },
    {
      icon: '🎯',
      title: 'Embodied Intelligence',
      description: 'Connect digital AI concepts to physical robotics - perception, reasoning, and action unified.',
    },
    {
      icon: '📖',
      title: 'Step-by-Step Learning',
      description: 'Complex concepts broken down with learning objectives, visuals, and assessments.',
    },
  ];

  return (
    <section className={styles.featuresSection}>
      <div className={styles.container}>
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>Why This Textbook?</h2>
          <p className={styles.sectionSubtitle}>
            Built for the future of Physical AI education
          </p>
        </div>
        
        <div className={styles.featuresGrid}>
          {features.map((feature, index) => (
            <FeatureCard key={index} {...feature} />
          ))}
        </div>
      </div>
    </section>
  );
}

function CurriculumSection() {
  const modules = [
    {
      number: '01',
      title: 'The Robotic Nervous System',
      topics: ['ROS 2 Architecture', 'Nodes & Topics', 'Python with rclpy', 'URDF Modeling'],
      icon: '🔌',
    },
    {
      number: '02',
      title: 'The Digital Twin',
      topics: ['Gazebo Simulations', 'Physics & Collision', 'Sensor Simulation', 'HRI Environments'],
      icon: '🖥️',
    },
    {
      number: '03',
      title: 'The AI-Robot Brain',
      topics: ['NVIDIA Isaac Sim', 'Synthetic Data', 'VSLAM', 'Nav2 Planning'],
      icon: '🧠',
    },
    {
      number: '04',
      title: 'Vision-Language-Action',
      topics: ['Speech Input', 'LLM Planning', 'Natural Language → ROS', 'Multimodal Control'],
      icon: '👁️',
    },
  ];

  return (
    <section className={styles.curriculumSection}>
      <div className={styles.container}>
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>Curriculum Modules</h2>
          <p className={styles.sectionSubtitle}>
            Four comprehensive modules from ROS 2 to Vision-Language-Action
          </p>
        </div>
        
        <div className={styles.modulesGrid}>
          {modules.map((module, index) => (
            <div key={index} className={styles.moduleCard}>
              <div className={styles.moduleNumber}>{module.number}</div>
              <div className={styles.moduleIcon}>{module.icon}</div>
              <h3 className={styles.moduleTitle}>{module.title}</h3>
              <ul className={styles.moduleTopics}>
                {module.topics.map((topic, i) => (
                  <li key={i}>{topic}</li>
                ))}
              </ul>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.ctaSection}>
      <div className={styles.ctaContainer}>
        <h2 className={styles.ctaTitle}>
          Ready to Learn Physical AI?
        </h2>
        <p className={styles.ctaSubtitle}>
          Join thousands of students learning the future of AI and robotics
        </p>
        <div className={styles.ctaButtons}>
          <Link
            to="/docs/intro"
            className={clsx(styles.button, styles.buttonPrimary, styles.buttonLarge)}
          >
            Start Learning Now
          </Link>
          <Link
            to="https://github.com/M-DaniyalHS1/Agentic_AI_Book"
            className={clsx(styles.button, styles.buttonOutline, styles.buttonLarge)}
          >
            View on GitHub
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="AI-Native Digital Textbook on Physical AI & Humanoid Robotics - Learn with AI-powered tutoring"
    >
      <HeroSection />
      <main>
        <FeaturesSection />
        <CurriculumSection />
        <CTASection />
      </main>
    </Layout>
  );
}
