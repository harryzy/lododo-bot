/**
 * MapToolbar - 地图工具栏组件
 * 提供导航目标设置、视图控制等交互功能
 */

import { Button, Space, Tooltip, Switch } from 'antd'
import { 
  AimOutlined, 
  FullscreenOutlined, 
  EnvironmentOutlined,
  ClearOutlined,
  EyeOutlined,
  EyeInvisibleOutlined
} from '@ant-design/icons'
import { useState } from 'react'
import { useTranslation } from 'react-i18next'

export type ToolMode = 'none' | 'nav_goal' | 'waypoint'

interface MapToolbarProps {
  onModeChange: (mode: ToolMode) => void
  onResetView: () => void
  onFollowRobot: () => void
  onClearGoal: () => void
  onCostmapToggle: (visible: boolean) => void
  currentMode: ToolMode
  hasMap: boolean
  costmapVisible: boolean
}

function MapToolbar({
  onModeChange,
  onResetView,
  onFollowRobot,
  onClearGoal,
  onCostmapToggle,
  currentMode,
  hasMap,
  costmapVisible
}: MapToolbarProps) {
  const { t } = useTranslation()
  const [followMode, setFollowMode] = useState(false)

  const handleNavGoalClick = () => {
    const newMode = currentMode === 'nav_goal' ? 'none' : 'nav_goal'
    onModeChange(newMode)
  }

  const handleFollowRobot = () => {
    setFollowMode(!followMode)
    onFollowRobot()
  }

  return (
    <Space size="middle" style={{ marginBottom: 12 }}>
      {/* 导航目标设置 */}
      <Tooltip title={t('mapView.toolbar.navGoalTooltip')}>
        <Button 
          type={currentMode === 'nav_goal' ? 'primary' : 'default'}
          icon={<EnvironmentOutlined />} 
          disabled={!hasMap}
          onClick={handleNavGoalClick}
        >
          {currentMode === 'nav_goal' ? t('mapView.toolbar.cancelNavGoal') : t('mapView.toolbar.setNavGoal')}
        </Button>
      </Tooltip>

      {/* 清除目标 */}
      <Tooltip title={t('mapView.toolbar.clearGoalTooltip')}>
        <Button 
          icon={<ClearOutlined />} 
          disabled={!hasMap}
          onClick={onClearGoal}
        >
          {t('mapView.toolbar.clearGoal')}
        </Button>
      </Tooltip>

      {/* 跟随机器人 */}
      <Tooltip title={t('mapView.toolbar.followRobotTooltip')}>
        <Button 
          type={followMode ? 'primary' : 'default'}
          icon={<AimOutlined />} 
          disabled={!hasMap}
          onClick={handleFollowRobot}
        >
          {followMode ? t('mapView.toolbar.stopFollow') : t('mapView.toolbar.followRobot')}
        </Button>
      </Tooltip>

      {/* 重置视图 */}
      <Tooltip title={t('mapView.toolbar.resetViewTooltip')}>
        <Button 
          icon={<FullscreenOutlined />} 
          disabled={!hasMap}
          onClick={onResetView}
        >
          {t('mapView.toolbar.resetView')}
        </Button>
      </Tooltip>

      {/* Costmap显示开关 */}
      <Tooltip title={t('mapView.toolbar.costmapTooltip')}>
        <Space>
          {costmapVisible ? <EyeOutlined /> : <EyeInvisibleOutlined />}
          <Switch 
            checked={costmapVisible}
            onChange={onCostmapToggle}
            disabled={!hasMap}
          />
          <span>{t('mapView.toolbar.costmap')}</span>
        </Space>
      </Tooltip>
    </Space>
  )
}

export default MapToolbar
