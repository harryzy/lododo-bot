import { Card } from 'antd'
import { useEffect, useRef } from 'react'
import { useMapStore } from '../../stores/mapStore'

function MapView() {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const { mapData, robotPose } = useMapStore()

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    // 清空画布
    ctx.clearRect(0, 0, canvas.width, canvas.height)

    // 绘制背景
    ctx.fillStyle = '#f0f0f0'
    ctx.fillRect(0, 0, canvas.width, canvas.height)

    // 绘制网格
    ctx.strokeStyle = '#d9d9d9'
    ctx.lineWidth = 1
    const gridSize = 50
    for (let x = 0; x < canvas.width; x += gridSize) {
      ctx.beginPath()
      ctx.moveTo(x, 0)
      ctx.lineTo(x, canvas.height)
      ctx.stroke()
    }
    for (let y = 0; y < canvas.height; y += gridSize) {
      ctx.beginPath()
      ctx.moveTo(0, y)
      ctx.lineTo(canvas.width, y)
      ctx.stroke()
    }

    // 绘制机器人位置（如果有）
    if (robotPose) {
      const centerX = canvas.width / 2
      const centerY = canvas.height / 2
      
      // 将机器人坐标转换为画布坐标（简化版）
      const scale = 50 // 1米 = 50像素
      const robotX = centerX + robotPose.x * scale
      const robotY = centerY - robotPose.y * scale

      // 绘制机器人
      ctx.fillStyle = '#1890ff'
      ctx.beginPath()
      ctx.arc(robotX, robotY, 10, 0, 2 * Math.PI)
      ctx.fill()

      // 绘制朝向
      ctx.strokeStyle = '#1890ff'
      ctx.lineWidth = 3
      ctx.beginPath()
      ctx.moveTo(robotX, robotY)
      ctx.lineTo(
        robotX + Math.cos(0) * 20,
        robotY - Math.sin(0) * 20
      )
      ctx.stroke()
    }

    // 绘制坐标轴
    ctx.strokeStyle = '#000'
    ctx.lineWidth = 2
    const centerX = canvas.width / 2
    const centerY = canvas.height / 2
    
    // X轴
    ctx.beginPath()
    ctx.moveTo(0, centerY)
    ctx.lineTo(canvas.width, centerY)
    ctx.stroke()
    
    // Y轴
    ctx.beginPath()
    ctx.moveTo(centerX, 0)
    ctx.lineTo(centerX, canvas.height)
    ctx.stroke()

    // 标注原点
    ctx.fillStyle = '#000'
    ctx.font = '14px Arial'
    ctx.fillText('(0, 0)', centerX + 5, centerY - 5)

  }, [mapData, robotPose])

  return (
    <Card title="地图视图" style={{ width: '100%', marginTop: '16px' }}>
      <canvas
        ref={canvasRef}
        width={800}
        height={600}
        style={{
          border: '1px solid #d9d9d9',
          borderRadius: '4px',
        }}
      />
    </Card>
  )
}

export default MapView
