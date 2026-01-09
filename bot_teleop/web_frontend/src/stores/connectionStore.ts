/**
 * 连接状态存储
 * 管理 WebSocket 连接状态
 */

import { create } from 'zustand'

interface ConnectionState {
  isConnected: boolean
  lastMessage: any | null
  connectWebSocket: () => void
  disconnectWebSocket: () => void
  setConnected: (connected: boolean) => void
  setLastMessage: (message: any) => void
}

export const useConnectionStore = create<ConnectionState>((set) => ({
  isConnected: false,
  lastMessage: null,

  connectWebSocket: () => {
    set({ isConnected: true })
  },

  disconnectWebSocket: () => {
    set({ isConnected: false })
  },

  setConnected: (connected: boolean) => {
    set({ isConnected: connected })
  },

  setLastMessage: (message: any) => {
    set({ lastMessage: message })
  },
}))
