/**
 * Client-side entry point for AI Tutor Widget
 * This script ensures the widget is loaded even if Docusaurus theme override fails
 * 
 * Usage: Add to docusaurus.config.js scripts section or include in custom.css
 */

// Self-executing function to load AI Tutor widget
(function() {
  'use strict';

  // Debug logging
  const DEBUG = true;
  function log(...args) {
    if (DEBUG) {
      console.log('[AI Tutor]', ...args);
    }
  }

  log('Initializing AI Tutor Widget...');

  // Check if widget is already loaded
  const existingWidget = document.querySelector('.ai-tutor-fab');
  if (existingWidget) {
    log('Widget already loaded, skipping initialization');
    return;
  }

  // Wait for DOM to be ready
  function initWidget() {
    log('DOM ready, checking for widget...');
    
    // Check if React has loaded the widget
    const checkInterval = setInterval(() => {
      const widget = document.querySelector('.ai-tutor-fab');
      if (widget) {
        log('✓ Widget found in DOM');
        clearInterval(checkInterval);
        
        // Add debug styles to ensure visibility
        widget.style.position = 'fixed';
        widget.style.bottom = '24px';
        widget.style.right = '24px';
        widget.style.zIndex = '1000';
        widget.style.display = 'flex';
        
        log('✓ Widget styles verified');
      }
    }, 500);

    // Stop checking after 5 seconds
    setTimeout(() => {
      clearInterval(checkInterval);
      const widget = document.querySelector('.ai-tutor-fab');
      if (!widget) {
        log('✗ Widget NOT found after 5 seconds');
        log('Possible causes:');
        log('  1. Root.tsx not loaded by Docusaurus');
        log('  2. React component failed to render');
        log('  3. JavaScript error in AITutorWidget.tsx');
        log('');
        log('Check browser console for errors');
      }
    }, 5000);
  }

  // Initialize when DOM is ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initWidget);
  } else {
    initWidget();
  }

  // Log environment info
  log('Page URL:', window.location.href);
  log('User Agent:', navigator.userAgent);

})();
