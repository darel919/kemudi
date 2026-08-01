import { defineStore } from 'pinia'

export type MultiplayerState = 'disconnected' | 'connecting' | 'connected' | 'error'

export const useMultiplayerStore = defineStore('multiplayer', {
  state: () => ({
    connectionState: 'disconnected' as MultiplayerState,
    serverUrl: null as string | null,
    roomId: null as string | null,
    localPlayerId: null as string | null,
    remotePlayers: [] as { id: string; joinedAt: number; vehicleId?: string }[],
    spectatorCount: 0,
  }),

  getters: {
    isConnected: (state) => state.connectionState === 'connected',
    isJoining: (state) => state.connectionState === 'connecting',
    isSpectating: (state) => state.localPlayerId === null && state.connectionState === 'connected',
  },

  actions: {
    connect(url: string) {
      this.connectionState = 'connecting'
      this.serverUrl = url
      this.remotePlayers = []
    },

    setConnected(playerId: string) {
      this.connectionState = 'connected'
      this.localPlayerId = playerId
    },

    addRemotePlayer(playerId: string) {
      this.remotePlayers.push({ id: playerId, joinedAt: Date.now() })
    },

    removeRemotePlayer(playerId: string) {
      this.remotePlayers = this.remotePlayers.filter((p) => p.id !== playerId)
    },

    disconnect() {
      this.$reset()
    },

    setError() {
      this.connectionState = 'error'
    },

    $reset() {
      this.connectionState = 'disconnected'
      this.serverUrl = null
      this.localPlayerId = null
      this.remotePlayers = []
      this.spectatorCount = 0
    },
  },
})
