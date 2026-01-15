import { useState } from 'react'
import { Layout, Menu, Button, Badge, Space } from 'antd'
import {
  RobotOutlined,
  DashboardOutlined,
  EnvironmentOutlined,
  SettingOutlined,
  WifiOutlined,
  DisconnectOutlined,
  GlobalOutlined,
} from '@ant-design/icons'
import { useTranslation } from 'react-i18next'
import { useConnectionStore } from '../../stores/connectionStore'
import TaskControl from '../TaskControl/TaskControl'
import MapView from '../MapView/MapView'
import MapManager from '../MapManager'
import WaypointManager from '../WaypointManager/WaypointManager'
import StatusMonitor from '../StatusMonitor'

const { Header, Sider, Content } = Layout

type MenuKey = 'dashboard' | 'tasks' | 'maps' | 'waypoints' | 'settings'

function MainLayout() {
  const { t } = useTranslation()
  const [selectedMenu, setSelectedMenu] = useState<MenuKey>('dashboard')
  const { isConnected } = useConnectionStore()

  const menuItems = [
    {
      key: 'dashboard',
      icon: <DashboardOutlined />,
      label: t('menu.dashboard'),
    },
    {
      key: 'tasks',
      icon: <RobotOutlined />,
      label: t('menu.tasks'),
    },
    {
      key: 'maps',
      icon: <GlobalOutlined />,
      label: t('menu.maps'),
    },
    {
      key: 'waypoints',
      icon: <EnvironmentOutlined />,
      label: t('menu.waypoints'),
    },
    {
      key: 'settings',
      icon: <SettingOutlined />,
      label: t('menu.settings'),
    },
  ]

  const renderContent = () => {
    switch (selectedMenu) {
      case 'dashboard':
        return (
          <div style={{ padding: '24px' }}>
            <h2>{t('menu.dashboard')}</h2>
            <MapView />
          </div>
        )
      case 'tasks':
        return (
          <div style={{ padding: '24px' }}>
            <h2>{t('menu.tasks')}</h2>
            <TaskControl />
          </div>
        )
      case 'maps':
        return <MapManager />
      case 'waypoints':
        return <WaypointManager />
      case 'settings':
        return (
          <div style={{ padding: '24px' }}>
            <h2>{t('menu.settings')}</h2>
            <p>系统设置</p>
          </div>
        )
      default:
        return null
    }
  }

  return (
    <Layout style={{ height: '100vh' }}>
      <Header
        style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          background: '#001529',
          padding: '0 24px',
        }}
      >
        <Space>
          <RobotOutlined style={{ fontSize: '24px', color: '#fff' }} />
          <h1 style={{ color: '#fff', margin: 0 }}>LeKiwi Robot</h1>
        </Space>
        
        <Space>
          <Badge status={isConnected ? 'success' : 'error'} />
          <Button
            type="text"
            icon={isConnected ? <WifiOutlined /> : <DisconnectOutlined />}
            style={{ color: '#fff' }}
          >
            {isConnected ? t('connection.connected') : t('connection.disconnected')}
          </Button>
        </Space>
      </Header>

      <Layout>
        <Sider width={200} style={{ background: '#fff' }}>
          <Menu
            mode="inline"
            selectedKeys={[selectedMenu]}
            items={menuItems}
            onClick={({ key }) => setSelectedMenu(key as MenuKey)}
            style={{ height: '100%', borderRight: 0 }}
          />
        </Sider>

        <Layout style={{ padding: '0' }}>
          <Content
            style={{
              padding: 0,
              margin: 0,
              minHeight: 280,
              background: '#fff',
              overflow: 'auto',
            }}
          >
            {renderContent()}
          </Content>
          
          {/* 右侧状态监控面板 */}
          <Sider width={300} style={{ background: '#f0f2f5' }}>
            <StatusMonitor />
          </Sider>
        </Layout>
      </Layout>
    </Layout>
  )
}

export default MainLayout
