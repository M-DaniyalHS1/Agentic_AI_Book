import React from 'react';
import AITutorWidget from '../components/AITutorWidget';

export default function Root({ children }) {
  return (
    <>
      {children}
      <AITutorWidget />
    </>
  );
}
