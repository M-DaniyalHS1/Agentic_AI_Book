import React from 'react';
import Layout from '@theme/Layout';
import BackgroundQuestions from '@site/src/components/auth/BackgroundQuestions';

export default function SignupBackgroundPage() {
  return (
    <Layout
      title="Background Questions"
      description="Tell us about yourself"
    >
      <main>
        <BackgroundQuestions />
      </main>
    </Layout>
  );
}
