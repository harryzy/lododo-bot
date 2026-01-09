/**
 * ROS 连接服务
 * 封装 rosbridge WebSocket 连接管理
 */

import * as ROSLIB from 'roslib'

class ROSConnection {
  private ros: ROSLIB.Ros | null = null
  private reconnectAttempts = 0
  private maxReconnectAttempts = 5
  private reconnectInterval = 5000 // 5秒
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null
  private connectionCallbacks: Array<(connected: boolean) => void> = []

  /**
   * 初始化 ROS 连接
   * @param url rosbridge WebSocket URL
   */
  connect(url: string = 'ws://localhost:9090'): ROSLIB.Ros {
    if (this.ros) {
      console.warn('[ROSConnection] 已存在连接，先断开')
      this.disconnect()
    }

    console.log(`[ROSConnection] 连接到 ${url}`)
    
    this.ros = new ROSLIB.Ros({
      url: url
    })

    // 连接成功
    this.ros.on('connection', () => {
      console.log('[ROSConnection] ✓ 连接成功')
      this.reconnectAttempts = 0
      if (this.reconnectTimer) {
        clearTimeout(this.reconnectTimer)
        this.reconnectTimer = null
      }
      this.notifyConnectionChange(true)
    })

    // 连接错误
    this.ros.on('error', (error: unknown) => {
      console.error('[ROSConnection] ✗ 连接错误:', error)
      this.notifyConnectionChange(false)
    })

    // 连接关闭
    this.ros.on('close', () => {
      console.warn('[ROSConnection] 连接已关闭')
      this.notifyConnectionChange(false)
      this.attemptReconnect(url)
    })

    return this.ros
  }

  /**
   * 尝试重新连接
   */
  private attemptReconnect(url: string) {
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.error(`[ROSConnection] 达到最大重连次数 (${this.maxReconnectAttempts})，停止重连`)
      return
    }

    this.reconnectAttempts++
    console.log(`[ROSConnection] 尝试重连 (${this.reconnectAttempts}/${this.maxReconnectAttempts})...`)

    this.reconnectTimer = setTimeout(() => {
      this.connect(url)
    }, this.reconnectInterval)
  }

  /**
   * 断开连接
   */
  disconnect() {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer)
      this.reconnectTimer = null
    }

    if (this.ros) {
      this.ros.close()
      this.ros = null
    }

    this.reconnectAttempts = 0
    console.log('[ROSConnection] 已断开连接')
  }

  /**
   * 获取 ROS 实例
   */
  getRos(): ROSLIB.Ros | null {
    return this.ros
  }

  /**
   * 检查是否已连接
   */
  isConnected(): boolean {
    return this.ros !== null && this.ros.isConnected
  }

  /**
   * 订阅连接状态变化
   */
  onConnectionChange(callback: (connected: boolean) => void) {
    this.connectionCallbacks.push(callback)
    
    // 立即通知当前状态
    callback(this.isConnected())
  }

  /**
   * 取消订阅连接状态
   */
  offConnectionChange(callback: (connected: boolean) => void) {
    const index = this.connectionCallbacks.indexOf(callback)
    if (index > -1) {
      this.connectionCallbacks.splice(index, 1)
    }
  }

  /**
   * 通知所有监听者连接状态变化
   */
  private notifyConnectionChange(connected: boolean) {
    this.connectionCallbacks.forEach(callback => {
      try {
        callback(connected)
      } catch (error) {
        console.error('[ROSConnection] 回调执行错误:', error)
      }
    })
  }
}

// 创建单例实例
const rosConnection = new ROSConnection()

export default rosConnection
export { ROSConnection }
