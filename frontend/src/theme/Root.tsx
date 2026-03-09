import React, { useEffect } from 'react';
import AITutorWidget from '../components/AITutorWidget';

interface RootProps {
  children: React.ReactNode;
}

export default function Root({ children }: RootProps) {
  // Listen for mount event from plugin
  useEffect(() => {
    const handleMount = (event: CustomEvent) => {
      console.log('[AI Tutor] Mount event received', event.detail);
    };

    window.addEventListener('ai-tutor-mount', handleMount as EventListener);
    return () => {
      window.removeEventListener('ai-tutor-mount', handleMount as EventListener);
    };
  }, []);

  return (
    <>
      {children}
      <div id="ai-tutor-react-root">
        <AITutorWidget />
      </div>
    </>
  );
}
