import React from 'react';
import Layout from '@theme/Layout';
import ResetPasswordRequest from '@site/src/components/auth/ResetPasswordRequest';

export default function ResetPasswordPage() {
  return (
    <Layout
      title="Reset Password"
      description="Reset your password"
    >
      <main>
        <ResetPasswordRequest />
      </main>
    </Layout>
  );
}
