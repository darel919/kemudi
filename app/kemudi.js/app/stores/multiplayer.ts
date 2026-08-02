import { defineStore } from 'pinia'
import type { RemoteVehicleSnapshot } from '~/types/multiplayer'

export type MultiplayerState = 'disconnected' | 'connecting' | 'connected' | 'error'

export const useMultiplayerStore = defineStore('multiplayer', {
  state: () => ({
    connectionState: 'disconnected' as MultiplayerState,
    multiplayerEnabled: false,
    configuredServerUrl: '',
    configuredRoomId: 'freeroam',
    serverUrl: null as string | null,
    roomId: null as string | null,
    localPlayerId: null as string | null,
    remotePlayers: [] as { id: string; joinedAt: number; vehicleId?: string }[],
    spectatorCount: 0,
    remoteSnapshots: {} as Record<string, RemoteVehicleSnapshot>,
  }),

  getters: {
    isConnected: (state) => state.connectionState === 'connected',
    isJoining: (state) => state.connectionState === 'connecting',
    isSpectating: (state) => state.localPlayerId === null && state.connectionState === 'connected',
  },

  actions: {
    configure(enabled: boolean, serverUrl: string, roomId: string) {
      this.multiplayerEnabled = enabled
      this.configuredServerUrl = serverUrl.trim()
      this.configuredRoomId = roomId.trim() || 'freeroam'
    },

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
      if (this.remotePlayers.some(player => player.id === playerId)) return
      if (this.remotePlayers.length >= 16) return
      this.remotePlayers.push({ id: playerId, joinedAt: Date.now() })
    },

    removeRemotePlayer(playerId: string) {
      this.remotePlayers = this.remotePlayers.filter((p) => p.id !== playerId)
      delete this.remoteSnapshots[playerId]
    },

    setRemoteSnapshot(snapshot: RemoteVehicleSnapshot) {
      const previous = this.remoteSnapshots[snapshot.id]
      if (previous && snapshot.timestamp <= previous.timestamp) return
      this.remoteSnapshots[snapshot.id] = snapshot
      const remote = this.remotePlayers.find(player => player.id === snapshot.id)
      if (remote) remote.vehicleId = snapshot.vehicleId
    },

    pruneStaleSnapshots(now = Date.now(), maxAgeMs = 3000) {
      for (const [playerId, snapshot] of Object.entries(this.remoteSnapshots)) {
        if (now - snapshot.timestamp > maxAgeMs) this.removeRemotePlayer(playerId)
      }
    },

    disconnect() {
      this.connectionState = 'disconnected'
      this.serverUrl = null
      this.roomId = null
      this.localPlayerId = null
      this.remotePlayers = []
      this.remoteSnapshots = {}
      this.spectatorCount = 0
    },

    setError() {
      this.connectionState = 'error'
    },

    $reset() {
      this.connectionState = 'disconnected'
      this.serverUrl = null
      this.localPlayerId = null
      this.remotePlayers = []
      this.remoteSnapshots = {}
      this.spectatorCount = 0
    },
  },
})
