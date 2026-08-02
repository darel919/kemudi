export interface RemoteVehicleSnapshot {
  id: string
  vehicleId: string
  timestamp: number
  positions: number[]
  telemetry: number[]
}

export type MultiplayerClientMessage =
  | { type: 'join'; roomId: string; vehicleId: string }
  | { type: 'snapshot'; vehicleId: string; positions: number[]; telemetry: number[]; timestamp: number }
  | { type: 'ping'; timestamp: number }

export type MultiplayerServerMessage =
  | { type: 'welcome'; playerId: string; roomId: string }
  | { type: 'peer_join'; id: string; vehicleId: string }
  | { type: 'peer_leave'; id: string }
  | ({ type: 'peer_snapshot'; snapshot: RemoteVehicleSnapshot })
  | { type: 'pong'; timestamp: number }
  | { type: 'error'; message: string }

export const MULTIPLAYER_LIMITS = {
  maxRoomLength: 64,
  maxVehicleIdLength: 64,
  maxNodes: 128,
  maxTelemetry: 80,
  maxSnapshotRate: 30,
} as const

export function isFiniteNumberArray(value: unknown, maxLength: number): value is number[] {
  return Array.isArray(value) && value.length <= maxLength && value.every(item => typeof item === 'number' && Number.isFinite(item))
}
