// Custom dev server configuration for proxying API requests
module.exports = {
  devServer: {
    port: 3000,
    proxy: {
      '/api': {
        target: 'http://127.0.0.1:8001',
        changeOrigin: true,
        secure: false,
      },
    },
  },
};
