/**
 * @type {import('@docusaurus/types').Plugin}
 * 
 * This plugin injects the AI Tutor Widget into every page
 * More reliable than Root.tsx theme override
 */
module.exports = function (context, options) {
  return {
    name: 'ai-tutor-widget-injector',
    
    injectHtmlTags() {
      return {
        postBodyTags: [
          `
<div id="ai-tutor-widget-container"></div>
<script>
  // AI Tutor Widget Auto-Loader
  (function() {
    'use strict';
    
    console.log('[AI Tutor] Widget loader initialized');
    
    // Wait for React to load
    function initWidget() {
      // Check if we're in a browser (not SSR)
      if (typeof window === 'undefined') return;
      
      // Wait for React hydration
      setTimeout(function() {
        // Look for the widget in the DOM
        var widget = document.querySelector('.ai-tutor-fab');
        
        if (widget) {
          console.log('[AI Tutor] Widget already present');
          return;
        }
        
        // Try to find React root
        var reactRoot = document.getElementById('ai-tutor-widget-container');
        if (!reactRoot) {
          console.error('[AI Tutor] Container not found');
          return;
        }
        
        console.log('[AI Tutor] Attempting to mount widget...');
        
        // Dispatch custom event for React to listen to
        window.dispatchEvent(new CustomEvent('ai-tutor-mount', {
          detail: { containerId: 'ai-tutor-widget-container' }
        }));
        
      }, 500);
    }
    
    // Initialize when DOM is ready
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', initWidget);
    } else {
      initWidget();
    }
    
    // Re-check on page navigation (SPA)
    document.addEventListener('click', function() {
      setTimeout(initWidget, 100);
    });
    
  })();
</script>
          `,
        ],
      };
    },
  };
};
