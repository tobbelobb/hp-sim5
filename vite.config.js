import { defineConfig } from 'vite';

export default defineConfig({
  // Set the root to the directory containing your index.html
  root: 'examples/js_flipper',

  server: {
    // Allow serving files from one level up to access the `src` directory
    fs: {
      allow: ['../..']
    }
  }
});
