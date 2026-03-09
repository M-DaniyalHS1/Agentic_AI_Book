/**
 * Docusaurus Plugin: AI Tutor Widget Injector
 * Injects widget container into every page HTML
 */
module.exports = function (context, options) {
  return {
    name: 'ai-tutor-widget-injector',
    
    injectHtmlTags() {
      return {
        postBodyTags: [
          '<div id="ai-tutor-widget-container"></div>',
          `<script>
(function() {
  console.log('[AI Tutor] Widget loader initialized');
  
  function initWidget() {
    if (typeof window === 'undefined') return;
    
    setTimeout(function() {
      var widget = document.querySelector('.ai-tutor-fab');
      if (widget) {
        console.log('[AI Tutor] Widget already present');
        return;
      }
      console.log('[AI Tutor] Checking widget mount...');
    }, 500);
  }
  
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initWidget);
  } else {
    initWidget();
  }
})();
</script>`
        ],
      };
    },
  };
};
