import React from 'react';
import AITutorWidget from '../components/AITutorWidget';

interface RootProps {
  children: React.ReactNode;
}

export default function Root({ children }: RootProps) {
  return (
    <>
      {children}
      <AITutorWidget />
    </>
  );
}
