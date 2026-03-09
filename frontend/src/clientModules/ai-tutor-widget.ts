// Docusaurus Client Module
// This injects the AI Tutor widget on the client side after hydration

import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

if (ExecutionEnvironment.canUseDOM) {
  // Wait for DOM to be ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => {
      injectAITutorWidget();
    });
  } else {
    injectAITutorWidget();
  }
}

function injectAITutorWidget() {
  // Check if widget container already exists
  if (document.getElementById('ai-tutor-widget-container')) {
    return;
  }

  // Create container
  const container = document.createElement('div');
  container.id = 'ai-tutor-widget-container';
  document.body.appendChild(container);

  // Log for debugging
  console.log('[AI Tutor] Widget container injected');

  // Dynamically import and render the React component
  import('../components/AITutorWidget').then((module) => {
    const React = require('react');
    const ReactDOM = require('react-dom/client');
    
    const AITutorWidget = module.default;
    const root = ReactDOM.createRoot(container);
    root.render(React.createElement(AITutorWidget));
    
    console.log('[AI Tutor] Widget rendered');
  }).catch((error) => {
    console.error('[AI Tutor] Failed to load widget:', error);
  });
}

export default {};
