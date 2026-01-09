/**
 * MapRenderer - Canvas地图渲染工具类
 * 
 * 功能：
 * 1. OccupancyGrid → Canvas ImageData 转换
 * 2. ROS坐标系 ↔ 屏幕坐标系转换
 * 3. 缩放/平移变换矩阵管理
 * 4. 地图和机器人位姿渲染
 */

// ROS OccupancyGrid 消息类型定义
export interface OccupancyGridMsg {
  header: {
    stamp: { sec: number; nanosec: number }
    frame_id: string
  }
  info: {
    resolution: number       // 米/像素
    width: number           // 地图宽度（栅格数）
    height: number          // 地图高度（栅格数）
    origin: {
      position: { x: number; y: number; z: number }
      orientation: { x: number; y: number; z: number; w: number }
    }
  }
  data: Int8Array | number[]  // 占用值：-1=未知，0=空闲，100=占用
}

// 机器人位姿消息类型
export interface PoseMsg {
  header: {
    stamp: { sec: number; nanosec: number }
    frame_id: string
  }
  pose: {
    pose: {
      position: { x: number; y: number; z: number }
      orientation: { x: number; y: number; z: number; w: number }
    }
  }
}

// 变换矩阵类型
interface Transform {
  scale: number       // 缩放比例
  offsetX: number    // X偏移（屏幕坐标）
  offsetY: number    // Y偏移（屏幕坐标）
}

export class MapRenderer {
  private canvas: HTMLCanvasElement
  private ctx: CanvasRenderingContext2D
  private transform: Transform
  
  // 地图数据缓存
  private mapData: OccupancyGridMsg | null = null
  private mapImageData: ImageData | null = null
  
  // 机器人位姿缓存
  private robotPose: PoseMsg | null = null
  
  // 导航目标
  private navGoal: { x: number; y: number; yaw: number } | null = null
  
  // 拖拽状态
  private isDragging = false
  private dragStartX = 0
  private dragStartY = 0
  
  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas
    const ctx = canvas.getContext('2d', { alpha: false })
    if (!ctx) {
      throw new Error('Failed to get 2D context')
    }
    this.ctx = ctx
    
    // 使用设备像素比提高清晰度
    const dpr = window.devicePixelRatio || 1
    const rect = canvas.getBoundingClientRect()
    canvas.width = rect.width * dpr
    canvas.height = rect.height * dpr
    ctx.scale(dpr, dpr)
    
    // 禁用图像平滑以获得像素级清晰度
    ctx.imageSmoothingEnabled = false
    
    // 初始化变换矩阵（默认缩放比例1.0，居中显示）
    this.transform = {
      scale: 1.0,
      offsetX: rect.width / 2,
      offsetY: rect.height / 2
    }
    
    console.log('[MapRenderer] 初始化完成, DPR:', dpr, 'Canvas尺寸:', canvas.width, 'x', canvas.height)
  }
  
  /**
   * 更新地图数据
   */
  updateMap(mapMsg: OccupancyGridMsg) {
    this.mapData = mapMsg
    this.mapImageData = this.createMapImageData(mapMsg)
    this.render()
  }
  
  /**
   * 更新机器人位姿
   */
  updateRobotPose(poseMsg: PoseMsg) {
    this.robotPose = poseMsg
    this.render()
  }
  
  /**
   * 设置导航目标
   */
  setNavGoal(x: number, y: number, yaw: number) {
    this.navGoal = { x, y, yaw }
    console.log('[MapRenderer] 设置导航目标:', x.toFixed(2), y.toFixed(2), 'yaw:', yaw.toFixed(2))
    this.render()
  }
  
  /**
   * 清除导航目标
   */
  clearNavGoal() {
    this.navGoal = null
    this.render()
  }
  
  /**
   * 屏幕坐标 → ROS 坐标
   */
  screenToRos(screenX: number, screenY: number): { x: number; y: number } {
    if (!this.mapData) {
      return { x: 0, y: 0 }
    }
    
    const { resolution, height, origin } = this.mapData.info
    
    // 1. 屏幕坐标 → Canvas像素坐标（逆变换）
    const canvasX = (screenX - this.transform.offsetX) / this.transform.scale
    const canvasY = (screenY - this.transform.offsetY) / this.transform.scale
    
    // 2. Canvas像素坐标 → 地图栅格坐标（Y轴翻转）
    const gridX = canvasX
    const gridY = height - canvasY
    
    // 3. 地图栅格坐标 → ROS世界坐标
    const rosX = gridX * resolution + origin.position.x
    const rosY = gridY * resolution + origin.position.y
    
    return { x: rosX, y: rosY }
  }
  
  /**
   * 将 OccupancyGrid 转换为 Canvas ImageData
   */
  private createMapImageData(mapMsg: OccupancyGridMsg): ImageData {
    const { width, height } = mapMsg.info
    const data = mapMsg.data
    const imageData = this.ctx.createImageData(width, height)
    
    for (let i = 0; i < data.length; i++) {
      const occupancy = data[i]
      let color: [number, number, number]
      
      // 占用值映射到颜色（优化清晰度）
      if (occupancy < 0) {
        // 未知区域 - 浅灰色
        color = [205, 205, 205]
      } else if (occupancy <= 10) {
        // 空闲区域 - 白色（容忍10%的噪声）
        color = [255, 255, 255]
      } else if (occupancy >= 90) {
        // 占用区域 - 黑色（容忍10%的噪声）
        color = [0, 0, 0]
      } else {
        // 中间值 - 深灰色（不确定区域）
        const ratio = (occupancy - 10) / 80  // 归一化到[0,1]
        const gray = Math.round(255 * (1 - ratio))
        color = [gray, gray, gray]
      }
      
      // ImageData 数组索引（RGBA格式）
      const pixelIndex = i * 4
      imageData.data[pixelIndex] = color[0]     // R
      imageData.data[pixelIndex + 1] = color[1] // G
      imageData.data[pixelIndex + 2] = color[2] // B
      imageData.data[pixelIndex + 3] = 255      // A
    }
    
    return imageData
  }
  
  /**
   * ROS 坐标 → 屏幕坐标
   * ROS: 原点在地图左下角，X向右，Y向上
   * Canvas: 原点在左上角，X向右，Y向下
   */
  private rosToScreen(rosX: number, rosY: number): { x: number; y: number } {
    if (!this.mapData) {
      return { x: 0, y: 0 }
    }
    
    const { resolution, height, origin } = this.mapData.info
    
    // 1. ROS世界坐标 → 地图栅格坐标
    const gridX = (rosX - origin.position.x) / resolution
    const gridY = (rosY - origin.position.y) / resolution
    
    // 2. 地图栅格坐标 → Canvas像素坐标（Y轴翻转）
    const canvasX = gridX
    const canvasY = height - gridY
    
    // 3. 应用缩放和平移变换
    const screenX = canvasX * this.transform.scale + this.transform.offsetX
    const screenY = canvasY * this.transform.scale + this.transform.offsetY
    
    return { x: screenX, y: screenY }
  }
  
  /**
   * 四元数 → 欧拉角 (yaw)
   */
  private quaternionToYaw(q: { x: number; y: number; z: number; w: number }): number {
    // yaw = atan2(2(w*z + x*y), 1 - 2(y^2 + z^2))
    const siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return Math.atan2(siny_cosp, cosy_cosp)
  }
  
  /**
   * 渲染主函数
   */
  render() {
    const rect = this.canvas.getBoundingClientRect()
    // 清空画布
    this.ctx.fillStyle = '#f0f0f0'
    this.ctx.fillRect(0, 0, rect.width, rect.height)
    
    // 渲染地图
    if (this.mapImageData && this.mapData) {
      this.renderMap()
    } else {
      // 显示等待消息
      this.ctx.fillStyle = '#999'
      this.ctx.font = '16px sans-serif'
      this.ctx.textAlign = 'center'
      this.ctx.fillText(
        '等待接收地图数据 (/map)...',
        rect.width / 2,
        rect.height / 2
      )
    }
    
    // 渲染机器人
    if (this.robotPose && this.mapData) {
      this.renderRobot()
    }
    
    // 渲染导航目标
    if (this.navGoal && this.mapData) {
      this.renderNavGoal()
    }
  }
  
  /**
   * 渲染地图
   */
  private renderMap() {
    if (!this.mapData || !this.mapImageData) return
    
    const { width, height } = this.mapData.info
    
    // 创建临时 canvas 来存储地图图像
    const tempCanvas = document.createElement('canvas')
    tempCanvas.width = width
    tempCanvas.height = height
    const tempCtx = tempCanvas.getContext('2d', { alpha: false })!
    
    // 禁用图像平滑
    tempCtx.imageSmoothingEnabled = false
    tempCtx.putImageData(this.mapImageData, 0, 0)
    
    // 应用变换并绘制
    this.ctx.save()
    this.ctx.imageSmoothingEnabled = false  // 确保缩放时不模糊
    this.ctx.translate(this.transform.offsetX, this.transform.offsetY)
    this.ctx.scale(this.transform.scale, this.transform.scale)
    this.ctx.drawImage(tempCanvas, 0, 0)
    this.ctx.restore()
  }
  
  /**
   * 渲染机器人位姿（蓝色箭头）
   */
  private renderRobot() {
    if (!this.robotPose) return
    
    const { position, orientation } = this.robotPose.pose.pose
    const screenPos = this.rosToScreen(position.x, position.y)
    const yaw = this.quaternionToYaw(orientation)
    
    this.ctx.save()
    this.ctx.translate(screenPos.x, screenPos.y)
    this.ctx.rotate(-yaw)  // Canvas Y轴向下，需要取反
    
    // 绘制箭头（长度20像素）
    const arrowLength = 20 * this.transform.scale
    const arrowWidth = 10 * this.transform.scale
    
    this.ctx.fillStyle = '#1890ff'  // Ant Design 蓝色
    this.ctx.beginPath()
    this.ctx.moveTo(arrowLength, 0)                    // 箭头尖端
    this.ctx.lineTo(-arrowLength / 2, arrowWidth / 2)  // 左下
    this.ctx.lineTo(-arrowLength / 2, -arrowWidth / 2) // 左上
    this.ctx.closePath()
    this.ctx.fill()
    
    // 绘制圆形底座
    this.ctx.fillStyle = '#1890ff'
    this.ctx.beginPath()
    this.ctx.arc(0, 0, 5 * this.transform.scale, 0, Math.PI * 2)
    this.ctx.fill()
    
    this.ctx.restore()
  }
  
  /**
   * 渲染导航目标（绿色箭头）
   */
  private renderNavGoal() {
    if (!this.navGoal) return
    
    console.log('[MapRenderer] 开始渲染导航目标:', this.navGoal)
    const screenPos = this.rosToScreen(this.navGoal.x, this.navGoal.y)
    console.log('[MapRenderer] 屏幕坐标:', screenPos)
    const yaw = this.navGoal.yaw
    
    this.ctx.save()
    this.ctx.translate(screenPos.x, screenPos.y)
    this.ctx.rotate(-yaw)  // Canvas Y轴向下，需要取反
    
    // 绘制箭头（长度30像素，比机器人大）
    const arrowLength = 30 * this.transform.scale
    const arrowWidth = 15 * this.transform.scale
    
    // 外边框（黑色）
    this.ctx.strokeStyle = '#000'
    this.ctx.lineWidth = 2
    this.ctx.beginPath()
    this.ctx.moveTo(arrowLength, 0)
    this.ctx.lineTo(-arrowLength / 2, arrowWidth / 2)
    this.ctx.lineTo(-arrowLength / 2, -arrowWidth / 2)
    this.ctx.closePath()
    this.ctx.stroke()
    
    // 填充（绿色）
    this.ctx.fillStyle = '#52c41a'  // Ant Design 绿色
    this.ctx.fill()
    
    // 绘制圆形底座
    this.ctx.strokeStyle = '#000'
    this.ctx.lineWidth = 2
    this.ctx.beginPath()
    this.ctx.arc(0, 0, 7 * this.transform.scale, 0, Math.PI * 2)
    this.ctx.stroke()
    this.ctx.fillStyle = '#52c41a'
    this.ctx.fill()
    
    this.ctx.restore()
  }
  
  /**
   * 缩放功能（鼠标滚轮）
   */
  zoom(delta: number, mouseX: number, mouseY: number) {
    const zoomFactor = delta > 0 ? 1.1 : 0.9
    const newScale = this.transform.scale * zoomFactor
    
    // 限制缩放范围 [0.1, 10]
    if (newScale < 0.1 || newScale > 10) return
    
    // 以鼠标位置为中心缩放
    this.transform.offsetX = mouseX - (mouseX - this.transform.offsetX) * zoomFactor
    this.transform.offsetY = mouseY - (mouseY - this.transform.offsetY) * zoomFactor
    this.transform.scale = newScale
    
    this.render()
  }
  
  /**
   * 开始拖拽
   */
  startDrag(x: number, y: number) {
    this.isDragging = true
    this.dragStartX = x - this.transform.offsetX
    this.dragStartY = y - this.transform.offsetY
  }
  
  /**
   * 拖拽移动
   */
  drag(x: number, y: number) {
    if (!this.isDragging) return
    
    this.transform.offsetX = x - this.dragStartX
    this.transform.offsetY = y - this.dragStartY
    
    this.render()
  }
  
  /**
   * 结束拖拽
   */
  endDrag() {
    this.isDragging = false
  }
  
  /**
   * 重置视图（适应地图到画布）
   */
  resetView() {
    if (!this.mapData) return
    
    const { width, height } = this.mapData.info
    const rect = this.canvas.getBoundingClientRect()
    const canvasWidth = rect.width
    const canvasHeight = rect.height
    
    // 计算缩放比例（保持宽高比）
    const scaleX = canvasWidth / width
    const scaleY = canvasHeight / height
    this.transform.scale = Math.min(scaleX, scaleY) * 0.9  // 留10%边距
    
    // 居中显示
    this.transform.offsetX = (canvasWidth - width * this.transform.scale) / 2
    this.transform.offsetY = (canvasHeight - height * this.transform.scale) / 2
    
    console.log('[MapRenderer] 重置视图, scale:', this.transform.scale.toFixed(2))
    this.render()
  }
  
  /**
   * 跟随机器人（将机器人位置移到屏幕中心）
   */
  followRobot() {
    if (!this.robotPose) return
    
    const { position } = this.robotPose.pose.pose
    const screenPos = this.rosToScreen(position.x, position.y)
    
    const rect = this.canvas.getBoundingClientRect()
    // 计算需要的偏移量
    const dx = rect.width / 2 - screenPos.x
    const dy = rect.height / 2 - screenPos.y
    
    this.transform.offsetX += dx
    this.transform.offsetY += dy
    
    console.log('[MapRenderer] 跟随机器人到:', position.x.toFixed(2), position.y.toFixed(2))
    this.render()
  }
}
