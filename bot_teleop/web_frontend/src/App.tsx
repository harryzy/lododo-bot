import { useEffect } from 'react'
import { ConfigProvider, theme } from 'antd'
import MainLayout from './components/Layout/MainLayout'
import { useConnectionStore } from './stores/connectionStore'
import { websocketService } from './services/websocket'

function App() {
  const { connectWebSocket } = useConnectionStore()

  useEffect(() => {
    // 启动时连接 WebSocket
    const wsUrl = `ws://${window.location.hostname}:8000/ws`
    websocketService.connect(wsUrl)
    connectWebSocket()

    return () => {
      websocketService.disconnect()
    }
  }, [connectWebSocket])

  return (
    <ConfigProvider
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
