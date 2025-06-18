import { defineConfig } from 'vite';

// This configuration makes the project's root directory the server's root.
// This allows for predictable absolute path resolution for assets.
export default defineConfig({
  server: {
    // This is still useful to allow the browser to fetch modules
    // from anywhere in the project if needed.
    fs: {
      allow: ['..']
    }
  }
});
