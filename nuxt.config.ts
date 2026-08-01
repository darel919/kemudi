// https://nuxt.com/docs/api/configuration/nuxt-config
export default defineNuxtConfig({
  compatibilityDate: '2025-07-15',
  // DevTools' reactive inspection is opt-in because the driving scene has a
  // high-frequency worker boundary. Enable it with KEMUDI_DEVTOOLS=true.
  devtools: { enabled: process.env.KEMUDI_DEVTOOLS === 'true' },
  ssr: false,
  future: {
    compatibilityVersion: 4
  },
  modules: ['@pinia/nuxt']
})
