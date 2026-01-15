import { useEffect } from 'react'
import { ConfigProvider, theme } from 'antd'
import zhCN from 'antd/locale/zh_CN'
import enUS from 'antd/locale/en_US'
import { useTranslation } from 'react-i18next'
import MainLayout from './components/Layout/MainLayout'
import { useConnectionStore } from './stores/connectionStore'
import { websocketService } from './services/websocket'
import { configService } from './services/config'

function App() {
  const { i18n } = useTranslation()
  const { connectWebSocket } = useConnectionStore()

  // 根据i18n语言选择Ant Design locale
  const getAntdLocale = () => {
    return i18n.language === 'en-US' ? enUS : zhCN
  }

  useEffect(() => {
    // 从配置服务获取WebSocket URL并连接
    const initWebSocket = async () => {
      try {
        const wsUrl = await configService.getWebSocketUrl()
        console.log('[App] Connecting to WebSocket:', wsUrl)
        websocketService.connect(wsUrl)
        connectWebSocket()
      } catch (error) {
        console.error('[App] Failed to get WebSocket URL:', error)
      }
    }

    initWebSocket()

    return () => {
      websocketService.disconnect()
    }
  }, [connectWebSocket])

  return (
    <ConfigProvider
      locale={getAntdLocale()}
      theme={{
        algorithm: theme.defaultAlgorithm,
        token: {
          colorPrimary: '#1890ff',
        },
      }}
    >
      <MainLayout />
    </ConfigProvider>
  )
}

export default App
