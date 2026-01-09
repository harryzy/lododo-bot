/**
 * 地图可视化组件 - Canvas实现
 * 使用原生Canvas + ROSLIB.js手动渲染地图
 */

import { Card, Button, Space, Spin, Alert } from 'antd'
import { ReloadOutlined, FullscreenOutlined, AimOutlined } from '@ant-design/icons'
import { useEffect, useRef, useState } from 'react'
import { useTranslation } from 'react-i18next'
import * as ROSLIB from 'roslib'
import rosConnection from '../../services/rosConnection'
import { MapRenderer } from '../../utils/MapRenderer'

function MapView() {
  const { t } = useTranslation()
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const mapRendererRef = useRef<MapRenderer | null>(null)
  
  const [rosConnected, setRosConnected] = useState(false)
  const [error, setError] = useState<string>('')
  const [mapReceived, setMapReceived] = useState(false)

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
    
    mapRendererRef.current?.startDrag(x, y)
  }
  
  const handleMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const rect = canvasRef.current?.getBoundingClientRect()
    if (!rect) return
    
    const x = e.clientX - rect.left
    const y = e.clientY - rect.top
    
    mapRendererRef.current?.drag(x, y)
  }
  
  const handleMouseUp = () => {
    mapRendererRef.current?.endDrag()
  }
  
  const handleResetView = () => {
    mapRendererRef.current?.resetView()
  }
  
  const handleFollowRobot = () => {
    mapRendererRef.current?.followRobot()
  }

  return (
    <Card 
      title={t('map.title', '地图视图')}
      style={{ width: '100%', height: 'calc(100vh - 120px)' }}
      extra={
        <Space>
          <Button 
            icon={<AimOutlined />} 
            disabled={!mapReceived}
            onClick={handleFollowRobot}
          >
            跟随机器人
          </Button>
          <Button 
            icon={<FullscreenOutlined />} 
            disabled={!mapReceived}
            onClick={handleResetView}
          >
            重置视图
          </Button>
          <Button 
            icon={<ReloadOutlined />} 
            onClick={() => window.location.reload()}
          >
            刷新
          </Button>
        </Space>
      }
    >
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
          cursor: mapReceived ? 'grab' : 'default'
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
