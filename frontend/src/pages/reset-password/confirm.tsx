import React from 'react';
import Layout from '@theme/Layout';
import ResetPasswordConfirm from '@site/src/components/auth/ResetPasswordConfirm';

export default function ResetPasswordConfirmPage() {
  return (
    <Layout
      title="Confirm Password Reset"
      description="Set your new password"
    >
      <main>
        <ResetPasswordConfirm />
      </main>
    </Layout>
  );
}
