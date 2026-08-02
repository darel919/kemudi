import { defineConfig } from 'vitest/config'
import { fileURLToPath } from 'url'

const __dirname = fileURLToPath(new URL('.', import.meta.url))

export default defineConfig({
  test: {
    globals: true,
    environment: 'happy-dom',
    include: ['tests/**/*.test.ts']
  },
  resolve: {
    alias: {
      '#app': fileURLToPath(new URL('./app', import.meta.url)),
      'app/composables': fileURLToPath(new URL('./app/composables', import.meta.url)),
      'app/components': fileURLToPath(new URL('./app/components', import.meta.url)),
      '@': __dirname,
      '~': fileURLToPath(new URL('./app', import.meta.url)),
    }
  }
})
