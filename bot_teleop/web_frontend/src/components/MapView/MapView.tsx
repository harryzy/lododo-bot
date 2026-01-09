/**
 * 地图可视化组件 - Canvas实现
 * 使用原生Canvas + ROSLIB.js手动渲染地图
 */

import { Card, Spin, Alert, message } from 'antd'
import { useEffect, useRef, useState } from 'react'
import { useTranslation } from 'react-i18next'
import * as ROSLIB from 'roslib'
import rosConnection from '../../services/rosConnection'
import { MapRenderer } from '../../utils/MapRenderer'
import MapToolbar, { ToolMode } from './MapToolbar'

function MapView() {
  const { t } = useTranslation()
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const mapRendererRef = useRef<MapRenderer | null>(null)
  
  const [rosConnected, setRosConnected] = useState(false)
  const [error, setError] = useState<string>('')
  const [mapReceived, setMapReceived] = useState(false)
  const [toolMode, setToolMode] = useState<ToolMode>('none')
  const [costmapVisible, setCostmapVisible] = useState(false)
  const [navGoalStart, setNavGoalStart] = useState<{x: number, y: number} | null>(null)

  // 初始化 ROS 连接
  useEffect(() => {
    console.log('[MapView] 初始化 ROS 连接...')
    
    // 连接 rosbridge
    rosConnection.connect('ws://localhost:9090')
    
    // 监听连接状态
    const handleConnectionChange = (connected: boolean) => {
      setRosConnected(connected)
      if (connected) {
        setError('')
        console.log('[MapView] ✓ ROS 连接成功')
      } else {
        setError('与 rosbridge 断开连接，尝试重新连接...')
      }
    }
    
    rosConnection.onConnectionChange(handleConnectionChange)
    
    return () => {
      rosConnection.offConnectionChange(handleConnectionChange)
    }
  }, [])

  // 初始化 MapRenderer
  useEffect(() => {
    if (!canvasRef.current) return
    
    try {
      mapRendererRef.current = new MapRenderer(canvasRef.current)
      mapRendererRef.current.render()
      console.log('[MapView] ✓ MapRenderer 初始化成功')
    } catch (err) {
      console.error('[MapView] ✗ MapRenderer 初始化失败:', err)
      setError('地图渲染器初始化失败')
    }
  }, [])

  // 订阅地图数据 (/map)
  useEffect(() => {
    if (!rosConnected || !mapRendererRef.current) return
    
    console.log('[MapView] 订阅 /map 话题...')
    
    const mapTopic = new ROSLIB.Topic({
      ros: rosConnection.getRos()!,
      name: '/map',
      messageType: 'nav_msgs/OccupancyGrid'
    })
    
    mapTopic.subscribe((message: any) => {
      console.log('[MapView] ✓ 收到地图数据:', message.info)
      mapRendererRef.current?.updateMap(message)
      
      // 首次收到地图时自动适应画布
      if (!mapReceived) {
        console.log('[MapView] 首次收到地图，自动适应画布')
        setTimeout(() => {
          mapRendererRef.current?.resetView()
        }, 100)
      }
      
      setMapReceived(true)
    })
    
    return () => {
      console.log('[MapView] 取消订阅 /map')
      mapTopic.unsubscribe()
    }
  }, [rosConnected])

  // 订阅机器人位姿 (/localization_pose)
  useEffect(() => {
    if (!rosConnected || !mapRendererRef.current) return
    
    console.log('[MapView] 订阅 /localization_pose 话题...')
    
    const poseTopic = new ROSLIB.Topic({
      ros: rosConnection.getRos()!,
      name: '/localization_pose',
      messageType: 'geometry_msgs/PoseWithCovarianceStamped'
    })
    
    poseTopic.subscribe((message: any) => {
      const pos = message.pose.pose.position
      console.log('[MapView] ✓ 收到机器人位姿:', 
        `x=${pos.x.toFixed(2)}, y=${pos.y.toFixed(2)}`)
      mapRendererRef.current?.updateRobotPose(message)
    })
    
    return () => {
      console.log('[MapView] 取消订阅 /localization_pose')
      poseTopic.unsubscribe()
    }
  }, [rosConnected])

  // 鼠标事件处理
  const handleWheel = (e: React.WheelEvent<HTMLCanvasElement>) => {
    e.preventDefault()
    const rect = canvasRef.current?.getBoundingClientRect()
    if (!rect) return
    
    const mouseX = e.clientX - rect.left
    const mouseY = e.clientY - rect.top
    
    mapRendererRef.current?.zoom(-e.deltaY, mouseX, mouseY)
  }
  
  const handleMouseDown = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const rect = canvasRef.current?.getBoundingClientRect()
    if (!rect) return
    
    const x = e.clientX - rect.left
    const y = e.clientY - rect.top
    
    // Nav Goal模式：记录起始点
    if (toolMode === 'nav_goal') {
      console.log('[MapView] Nav Goal 模式 - 记录起始点:', x, y)
      setNavGoalStart({ x, y })
      
      // 立即显示箭头（初始朝向为0，即向上）
      const rosGoal = mapRendererRef.current?.screenToRos(x, y)
      if (rosGoal) {
        mapRendererRef.current?.setNavGoal(rosGoal.x, rosGoal.y, 0)
      }
      return
    }
    
    // 普通拖拽模式
    mapRendererRef.current?.startDrag(x, y)
  }
  
  const handleMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const rect = canvasRef.current?.getBoundingClientRect()
    if (!rect) return
    
    const x = e.clientX - rect.left
    const y = e.clientY - rect.top
    
    // Nav Goal模式：实时更新箭头朝向
    if (toolMode === 'nav_goal' && navGoalStart && mapRendererRef.current) {
      const dx = x - navGoalStart.x
      const dy = y - navGoalStart.y
      const yaw = Math.atan2(dy, dx) + Math.PI / 2  // Canvas Y轴向下，需要调整
      
      // 转换为ROS坐标
      const rosGoal = mapRendererRef.current.screenToRos(navGoalStart.x, navGoalStart.y)
      
      // 实时更新箭头显示
      mapRendererRef.current.setNavGoal(rosGoal.x, rosGoal.y, yaw)
      return
    }
    
    // 普通拖拽模式
    mapRendererRef.current?.drag(x, y)
  }
  
  const handleMouseUp = (e: React.MouseEvent<HTMLCanvasElement>) => {
    console.log('[MapView] 鼠标释放 - toolMode:', toolMode, 'navGoalStart:', navGoalStart)
    // Nav Goal模式下的鼠标释放
    if (toolMode === 'nav_goal' && navGoalStart && canvasRef.current) {
      const rect = canvasRef.current.getBoundingClientRect()
      const endX = e.clientX - rect.left
      const endY = e.clientY - rect.top
      console.log('[MapView] 结束点:', endX, endY)
      
      // 计算朝向
      const dx = endX - navGoalStart.x
      const dy = endY - navGoalStart.y
      const yaw = Math.atan2(dy, dx) + Math.PI / 2  // Canvas Y轴向下，需要调整
      
      // 转换为ROS坐标
      const rosGoal = mapRendererRef.current?.screenToRos(navGoalStart.x, navGoalStart.y)
      console.log('[MapView] 计算得到ROS坐标:', rosGoal, 'yaw:', yaw)
        console.log('[MapView] mapRendererRef.current:', mapRendererRef.current)
        
      if (rosGoal) {
        // 设置导航目标显示
        mapRendererRef.current?.setNavGoal(rosGoal.x, rosGoal.y, yaw)
        
        // 发送导航请求
        sendNavigationGoal(rosGoal.x, rosGoal.y, yaw)
        
        message.success(`导航目标已设置: (${rosGoal.x.toFixed(2)}, ${rosGoal.y.toFixed(2)})`)
      }
      
      // 退出Nav Goal模式
      setToolMode('none')
      setNavGoalStart(null)
    } else {
      // 普通拖拽结束
      mapRendererRef.current?.endDrag()
    }
  }
  
  const handleResetView = () => {
    mapRendererRef.current?.resetView()
  }
  
  const handleFollowRobot = () => {
    mapRendererRef.current?.followRobot()
  }
  
  const handleClearGoal = () => {
    mapRendererRef.current?.clearNavGoal()
    message.info('已清除导航目标')
  }
  
  const sendNavigationGoal = async (x: number, y: number, yaw: number) => {
    try {
      const goalTopic = new ROSLIB.Topic({
        ros: rosConnection.getRos()!,
        name: '/goal_pose',
        messageType: 'geometry_msgs/PoseStamped'
      })
      
      const goalMsg = {
        header: {
          frame_id: 'map',
          stamp: { sec: 0, nanosec: 0 }
        },
        pose: {
          position: { x, y, z: 0 },
          orientation: {
            x: 0,
            y: 0,
            z: Math.sin(yaw / 2),
            w: Math.cos(yaw / 2)
          }
        }
      }
      
      goalTopic.publish(goalMsg)
      console.log('[MapView] 发送导航目标:', x, y, 'yaw:', yaw)
    } catch (err) {
      console.error('[MapView] 发送导航目标失败:', err)
      message.error('发送导航目标失败')
    }
  }

  return (
    <Card 
      title={t('map.title', '地图视图')}
      style={{ width: '100%', height: 'calc(100vh - 120px)' }}
    >
      <MapToolbar 
        onModeChange={setToolMode}
        onResetView={handleResetView}
        onFollowRobot={handleFollowRobot}
        onClearGoal={handleClearGoal}
        onCostmapToggle={setCostmapVisible}
        currentMode={toolMode}
        hasMap={mapReceived}
        costmapVisible={costmapVisible}
      />
      
      {error && (
        <Alert 
          message="连接错误" 
          description={error} 
          type="warning" 
          showIcon 
          closable
          style={{ marginBottom: 16 }}
        />
      )}

      {!rosConnected && !error && (
        <div style={{ textAlign: 'center', padding: '60px 0' }}>
          <Spin size="large" />
          <p style={{ marginTop: 16 }}>正在连接 rosbridge...</p>
        </div>
      )}

      <canvas 
        ref={canvasRef}
        onWheel={handleWheel}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
        style={{
          width: '100%',
          height: 'calc(100% - 60px)',
          backgroundColor: '#f0f0f0',
          border: '1px solid #d9d9d9',
          borderRadius: '4px',
          cursor: toolMode === 'nav_goal' ? 'crosshair' : (mapReceived ? 'grab' : 'default')
        }}
      />

      {rosConnected && (
        <div style={{ 
          position: 'absolute', 
          bottom: 16, 
          right: 16, 
          backgroundColor: 'rgba(255, 255, 255, 0.9)',
          padding: '8px 12px',
          borderRadius: '4px',
          fontSize: '12px',
          boxShadow: '0 2px 8px rgba(0,0,0,0.15)'
        }}>
          <div style={{ color: '#52c41a' }}>● 已连接 rosbridge</div>
        </div>
      )}
    </Card>
  )
}

export default MapView
