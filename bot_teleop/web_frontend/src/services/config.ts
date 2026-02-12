/**
 * 前端配置管理
 * 从后端API获取配置，避免硬编码
 */

interface WebConfig {
  server: {
    host: string
    port: number
  }
  websocket: {
    ping_interval: number
    ping_timeout: number
  }
  rosbridge: {
    url: string
  }
  ui: {
    language: string
    theme: string
  }
}

class ConfigService {
  private config: WebConfig | null = null
  private configPromise: Promise<WebConfig> | null = null

  /**
   * 获取配置（单例模式，只加载一次）
   */
  async getConfig(): Promise<WebConfig> {
    if (this.config) {
      return this.config
    }

    if (this.configPromise) {
      return this.configPromise
    }

    this.configPromise = this.loadConfig()
    this.config = await this.configPromise
    return this.config
  }

  /**
   * 从后端加载配置
   */
  private async loadConfig(): Promise<WebConfig> {
    try {
      const response = await fetch('/api/config')
      if (!response.ok) {
        throw new Error('Failed to load config')
      }
      const config = await response.json()
      console.log('[Config] Loaded configuration:', config)
      return config
    } catch (error) {
      console.error('[Config] Failed to load config, using defaults:', error)
      // 如果加载失败，使用默认配置
      return this.getDefaultConfig()
    }
  }

  /**
   * 获取默认配置（作为fallback）
   */
  private getDefaultConfig(): WebConfig {
    return {
      server: {
        host: window.location.hostname,
        port: 8000,
      },
      websocket: {
        ping_interval: 30,
        ping_timeout: 10,
      },
      rosbridge: {
        url: 'ws://localhost:9091',
      },
      ui: {
        language: 'zh-CN',
        theme: 'light',
      },
    }
  }

  /**
   * 获取WebSocket URL
   */
  async getWebSocketUrl(): Promise<string> {
    const config = await this.getConfig()
    const host = window.location.hostname
    const port = config.server.port
    return `ws://${host}:${port}/ws`
  }

  /**
   * 获取API基础URL
   */
  async getApiBaseUrl(): Promise<string> {
    const config = await this.getConfig()
    const host = window.location.hostname
    const port = config.server.port
    return `http://${host}:${port}/api`
  }
}

// 导出单例
export const configService = new ConfigService()
export default configService
