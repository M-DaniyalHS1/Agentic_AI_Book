import React from 'react';
import { useHistory } from '@docusaurus/router';

interface SignOutButtonProps {
  className?: string;
  children?: React.ReactNode;
}

export default function SignOutButton({ className = '', children }: SignOutButtonProps) {
  const history = useHistory();

  const handleSignOut = async () => {
    try {
      // Use hardcoded API URL for local development - using 'localhost' not '127.0.0.1' for cookie consistency
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

  return (
    <button onClick={handleSignOut} className={className}>
      {children || 'Sign Out'}
    </button>
  );
}
