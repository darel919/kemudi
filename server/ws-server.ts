import { randomUUID } from 'node:crypto'
import { WebSocketServer, WebSocket } from 'ws'

const port = Number(process.env.KEMUDI_WS_PORT ?? 8787)
const maxMessageBytes = 256 * 1024
const rooms = new Map<string, Set<Client>>()

interface Client {
  socket: WebSocket
  id: string
  roomId: string
  vehicleId: string
  lastSnapshotAt: number
}

const server = new WebSocketServer({ port, maxPayload: maxMessageBytes })

server.on('connection', socket => {
  let client: Client | null = null
  socket.on('message', raw => {
    const rawText = raw.toString()
    if (Buffer.byteLength(rawText) > maxMessageBytes) return closeWithError(socket, 'message too large')
    let message: unknown
    try { message = JSON.parse(rawText) } catch { return closeWithError(socket, 'invalid JSON') }
    if (!message || typeof message !== 'object') return closeWithError(socket, 'invalid message')
    const data = message as Record<string, unknown>
    if (data.type === 'join') {
      if (client || typeof data.roomId !== 'string' || typeof data.vehicleId !== 'string' || !/^[A-Za-z0-9_-]{1,64}$/.test(data.roomId) || !/^[A-Za-z0-9_-]{1,64}$/.test(data.vehicleId)) return closeWithError(socket, 'invalid join')
      client = { socket, id: randomUUID(), roomId: data.roomId, vehicleId: data.vehicleId, lastSnapshotAt: 0 }
      const room = rooms.get(client.roomId) ?? new Set<Client>()
      rooms.set(client.roomId, room)
      send(socket, { type: 'welcome', playerId: client.id, roomId: client.roomId })
      for (const peer of room) send(socket, { type: 'peer_join', id: peer.id, vehicleId: peer.vehicleId })
      for (const peer of room) send(peer.socket, { type: 'peer_join', id: client.id, vehicleId: client.vehicleId })
      room.add(client)
      return
    }
    if (!client) return closeWithError(socket, 'join required')
    if (data.type === 'ping' && typeof data.timestamp === 'number' && Number.isFinite(data.timestamp)) return send(socket, { type: 'pong', timestamp: data.timestamp })
    if (data.type !== 'snapshot' || typeof data.vehicleId !== 'string' || !/^[A-Za-z0-9_-]{1,64}$/.test(data.vehicleId) || !Array.isArray(data.positions) || !Array.isArray(data.telemetry) || typeof data.timestamp !== 'number') return closeWithError(socket, 'invalid snapshot')
    if (data.positions.length > 384 || data.telemetry.length > 80 || !data.positions.every(isFiniteNumber) || !data.telemetry.every(isFiniteNumber) || !Number.isFinite(data.timestamp)) return closeWithError(socket, 'snapshot exceeds limits')
    const now = Date.now()
    if (now - client.lastSnapshotAt < 25) return
    client.lastSnapshotAt = now
    const snapshot = { id: client.id, vehicleId: data.vehicleId, timestamp: data.timestamp, positions: data.positions, telemetry: data.telemetry }
    for (const peer of rooms.get(client.roomId) ?? []) {
      if (peer !== client) send(peer.socket, { type: 'peer_snapshot', snapshot })
    }
  })
  socket.on('close', () => {
    if (!client) return
    const room = rooms.get(client.roomId)
    room?.delete(client)
    for (const peer of room ?? []) send(peer.socket, { type: 'peer_leave', id: client.id })
    if (room?.size === 0) rooms.delete(client.roomId)
  })
})

server.on('listening', () => console.debug('[network:server] listening', { port }))

function isFiniteNumber(value: unknown): value is number { return typeof value === 'number' && Number.isFinite(value) }
function send(socket: WebSocket, message: unknown) { if (socket.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message)) }
function closeWithError(socket: WebSocket, message: string) { send(socket, { type: 'error', message }); socket.close(1008, message) }
