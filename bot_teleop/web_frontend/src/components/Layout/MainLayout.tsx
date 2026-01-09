import { useState } from 'react'
import { Layout, Menu, Button, Badge, Space } from 'antd'
import {
  RobotOutlined,
  DashboardOutlined,
  EnvironmentOutlined,
  SettingOutlined,
  WifiOutlined,
  DisconnectOutlined,
} from '@ant-design/icons'
import { useTranslation } from 'react-i18next'
import { useConnectionStore } from '../../stores/connectionStore'
import TaskControl from '../TaskControl/TaskControl'
import MapView from '../MapView/MapView'

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
      icon: <EnvironmentOutlined />,
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
        return (
          <div style={{ padding: '24px' }}>
            <h2>{t('menu.maps')}</h2>
            <p>地图管理功能</p>
          </div>
        )
      case 'waypoints':
        return (
          <div style={{ padding: '24px' }}>
            <h2>{t('menu.waypoints')}</h2>
            <p>路点管理功能</p>
          </div>
        )
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
        </Layout>
      </Layout>
    </Layout>
  )
}

export default MainLayout
