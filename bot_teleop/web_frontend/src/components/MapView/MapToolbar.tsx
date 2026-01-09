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
      <Tooltip title="点击地图设置导航目标（类似RViz 2D Nav Goal）">
        <Button 
          type={currentMode === 'nav_goal' ? 'primary' : 'default'}
          icon={<EnvironmentOutlined />} 
          disabled={!hasMap}
          onClick={handleNavGoalClick}
        >
          {currentMode === 'nav_goal' ? '取消导航目标' : '设置导航目标'}
        </Button>
      </Tooltip>

      {/* 清除目标 */}
      <Tooltip title="清除当前导航目标">
        <Button 
          icon={<ClearOutlined />} 
          disabled={!hasMap}
          onClick={onClearGoal}
        >
          清除目标
        </Button>
      </Tooltip>

      {/* 跟随机器人 */}
      <Tooltip title="自动跟随机器人位置">
        <Button 
          type={followMode ? 'primary' : 'default'}
          icon={<AimOutlined />} 
          disabled={!hasMap}
          onClick={handleFollowRobot}
        >
          {followMode ? '停止跟随' : '跟随机器人'}
        </Button>
      </Tooltip>

      {/* 重置视图 */}
      <Tooltip title="重置地图视图到初始状态">
        <Button 
          icon={<FullscreenOutlined />} 
          disabled={!hasMap}
          onClick={onResetView}
        >
          重置视图
        </Button>
      </Tooltip>

      {/* Costmap显示开关 */}
      <Tooltip title="显示/隐藏代价地图（Costmap）">
        <Space>
          {costmapVisible ? <EyeOutlined /> : <EyeInvisibleOutlined />}
          <Switch 
            checked={costmapVisible}
            onChange={onCostmapToggle}
            disabled={!hasMap}
          />
          <span>Costmap</span>
        </Space>
      </Tooltip>
    </Space>
  )
}

export default MapToolbar
