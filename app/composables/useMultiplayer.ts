import { ref, readonly } from 'vue'
import { storeToRefs } from 'pinia'
import { useMultiplayerStore } from '~/stores/multiplayer'
import {
  isFiniteNumberArray,
  MULTIPLAYER_LIMITS,
  type MultiplayerServerMessage,
  type RemoteVehicleSnapshot,
} from '~/types/multiplayer'
import { logDebug } from '~/utils/debug'

/** Bounded client transport for the optional multiplayer server. */
export function useMultiplayer() {
  const store = useMultiplayerStore()
  const { connectionState } = storeToRefs(store)
  const lastError = ref<string | null>(null)
  let socket: WebSocket | null = null
  let reconnectTimer: ReturnType<typeof setTimeout> | null = null
  let reconnectAttempt = 0
  let intentionalClose = false
  let pendingJoin: { roomId: string; vehicleId: string } | null = null
  let lastSnapshotAt = 0

  function clearReconnectTimer() {
    if (reconnectTimer !== null) clearTimeout(reconnectTimer)
    reconnectTimer = null
  }

  function parseMessage(value: unknown): MultiplayerServerMessage | null {
    if (!value || typeof value !== 'object') return null
    const message = value as Record<string, unknown>
    if (typeof message.type !== 'string') return null
    if (message.type === 'welcome' && typeof message.playerId === 'string' && typeof message.roomId === 'string') return message as MultiplayerServerMessage
    if (message.type === 'peer_join' && typeof message.id === 'string' && typeof message.vehicleId === 'string') return message as MultiplayerServerMessage
    if (message.type === 'peer_leave' && typeof message.id === 'string') return message as MultiplayerServerMessage
    if (message.type === 'pong' && typeof message.timestamp === 'number' && Number.isFinite(message.timestamp)) return message as MultiplayerServerMessage
    if (message.type === 'error' && typeof message.message === 'string') return message as MultiplayerServerMessage
    if (message.type === 'peer_snapshot') {
      const snapshot = message.snapshot as Record<string, unknown> | undefined
      if (snapshot && typeof snapshot.id === 'string' && typeof snapshot.vehicleId === 'string' && typeof snapshot.timestamp === 'number' && Number.isFinite(snapshot.timestamp) && isFiniteNumberArray(snapshot.positions, MULTIPLAYER_LIMITS.maxNodes * 3) && isFiniteNumberArray(snapshot.telemetry, MULTIPLAYER_LIMITS.maxTelemetry)) {
        return { type: 'peer_snapshot', snapshot: snapshot as unknown as RemoteVehicleSnapshot }
      }
    }
    return null
  }

  function onMessage(event: MessageEvent) {
    try {
      const parsed = parseMessage(JSON.parse(typeof event.data === 'string' ? event.data : ''))
      if (!parsed) return
      if (parsed.type === 'welcome') {
        store.setConnected(parsed.playerId)
      } else if (parsed.type === 'peer_join') {
        store.addRemotePlayer(parsed.id)
      } else if (parsed.type === 'peer_leave') {
        store.removeRemotePlayer(parsed.id)
      } else if (parsed.type === 'peer_snapshot') {
        store.setRemoteSnapshot(parsed.snapshot)
      } else if (parsed.type === 'error') {
        lastError.value = parsed.message
        store.setError()
      }
    } catch {
      // Malformed network data is ignored; it must not crash the simulation.
    }
  }

  function scheduleReconnect() {
    if (intentionalClose || !pendingJoin || reconnectTimer !== null) return
    const delay = Math.min(10_000, 500 * 2 ** reconnectAttempt++)
    reconnectTimer = setTimeout(() => {
      reconnectTimer = null
      if (pendingJoin && store.serverUrl) open(store.serverUrl, pendingJoin.roomId, pendingJoin.vehicleId)
    }, delay)
  }

  function open(url: string, roomId: string, vehicleId: string) {
    if (typeof WebSocket === 'undefined') {
      lastError.value = 'WebSocket is unavailable in this browser'
      store.setError()
      return
    }
    socket?.close()
    store.connect(url)
    try {
      const parsed = new URL(url)
      if (parsed.protocol !== 'ws:' && parsed.protocol !== 'wss:') throw new Error('WebSocket URL must use ws:// or wss://')
      socket = new WebSocket(parsed)
    } catch (error) {
      lastError.value = error instanceof Error ? error.message : String(error)
      store.setError()
      scheduleReconnect()
      return
    }
    socket.onopen = () => {
      reconnectAttempt = 0
      socket?.send(JSON.stringify({ type: 'join', roomId, vehicleId }))
      logDebug('network:state-changed', { state: 'connected' })
    }
    socket.onmessage = onMessage
    socket.onerror = () => {
      lastError.value = 'Multiplayer connection failed'
      store.setError()
      logDebug('network:state-changed', { state: 'error' })
    }
    socket.onclose = () => {
      socket = null
      if (!intentionalClose) {
        store.connectionState = 'connecting'
        scheduleReconnect()
      }
    }
  }

  function connect(url: string, roomId = 'default', vehicleId = 'vehicle') {
    if (!/^[A-Za-z0-9_-]{1,64}$/.test(roomId) || !/^[A-Za-z0-9_-]{1,64}$/.test(vehicleId)) {
      lastError.value = 'Room and vehicle identifiers contain unsupported characters'
      store.setError()
      return
    }
    intentionalClose = false
    pendingJoin = { roomId, vehicleId }
    open(url, roomId, vehicleId)
  }

  function sendSnapshot(vehicleId: string, positions: ArrayLike<number>, telemetry: ArrayLike<number>) {
    const now = performance.now()
    if (!socket || socket.readyState !== WebSocket.OPEN || now - lastSnapshotAt < 1000 / MULTIPLAYER_LIMITS.maxSnapshotRate) return false
    if (!/^[A-Za-z0-9_-]{1,64}$/.test(vehicleId)) return false
    if (positions.length > MULTIPLAYER_LIMITS.maxNodes * 3 || telemetry.length > MULTIPLAYER_LIMITS.maxTelemetry) return false
    for (let i = 0; i < positions.length; i++) if (!Number.isFinite(positions[i])) return false
    for (let i = 0; i < telemetry.length; i++) if (!Number.isFinite(telemetry[i])) return false
    lastSnapshotAt = now
    socket.send(JSON.stringify({ type: 'snapshot', vehicleId, positions: Array.from(positions), telemetry: Array.from(telemetry), timestamp: Date.now() }))
    return true
  }

  function disconnect() {
    intentionalClose = true
    pendingJoin = null
    clearReconnectTimer()
    socket?.close()
    socket = null
    store.disconnect()
    logDebug('network:state-changed', { state: 'disconnected' })
  }

  return { connectionState: readonly(connectionState), lastError: readonly(lastError), connect, sendSnapshot, disconnect }
}
