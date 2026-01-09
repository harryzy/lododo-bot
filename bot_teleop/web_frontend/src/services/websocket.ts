/**
 * WebSocket 服务
 * 管理与后端的 WebSocket 连接，接收实时数据
 */

type MessageCallback = (data: any) => void

class WebSocketService {
  private ws: WebSocket | null = null
  private url: string = ''
  private reconnectAttempts: number = 0
  private maxReconnectAttempts: number = 5
  private reconnectDelay: number = 3000
  private messageCallbacks: Set<MessageCallback> = new Set()

  /**
   * 连接 WebSocket
   */
  connect(url: string) {
    this.url = url
    this.ws = new WebSocket(url)

    this.ws.onopen = () => {
      console.log('[WebSocket] Connected to', url)
      this.reconnectAttempts = 0
    }

    this.ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data)
        console.log('[WebSocket] Received:', data)
        
        // 通知所有订阅者
        this.messageCallbacks.forEach((callback) => callback(data))
      } catch (error) {
        console.error('[WebSocket] Failed to parse message:', error)
      }
    }

    this.ws.onerror = (error) => {
      console.error('[WebSocket] Error:', error)
    }

    this.ws.onclose = () => {
      console.log('[WebSocket] Disconnected')
      this.attemptReconnect()
    }
  }

  /**
   * 尝试重新连接
   */
  private attemptReconnect() {
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.error('[WebSocket] Max reconnect attempts reached')
      return
    }

    this.reconnectAttempts++
    console.log(`[WebSocket] Reconnecting in ${this.reconnectDelay}ms (attempt ${this.reconnectAttempts})`)

    setTimeout(() => {
      if (this.url) {
        this.connect(this.url)
      }
    }, this.reconnectDelay)
  }

  /**
   * 断开连接
   */
  disconnect() {
    if (this.ws) {
      this.ws.close()
      this.ws = null
    }
  }

  /**
   * 发送消息
   */
  send(data: any) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      const message = typeof data === 'string' ? data : JSON.stringify(data)
      this.ws.send(message)
    } else {
      console.warn('[WebSocket] Cannot send message: not connected')
    }
  }

  /**
   * 订阅消息
   */
  subscribe(callback: MessageCallback) {
    this.messageCallbacks.add(callback)
    
    // 返回取消订阅函数
    return () => {
      this.messageCallbacks.delete(callback)
    }
  }

  /**
   * 获取连接状态
   */
  isConnected(): boolean {
    return this.ws !== null && this.ws.readyState === WebSocket.OPEN
  }
}

export const websocketService = new WebSocketService()
