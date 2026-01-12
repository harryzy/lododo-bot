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

// OccupancyGridUpdate 增量更新消息类型
export interface OccupancyGridUpdateMsg {
  header: {
    stamp: { sec: number; nanosec: number }
    frame_id: string
  }
  x: number           // 更新区域起始 x 坐标（栅格单位）
  y: number           // 更新区域起始 y 坐标（栅格单位）
  width: number       // 更新区域宽度
  height: number      // 更新区域高度
  data: Int8Array | number[]  // 更新的数据（width * height）
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
  
  // Costmap 数据缓存
  private localCostmap: OccupancyGridMsg | null = null
  private globalCostmap: OccupancyGridMsg | null = null
  private showLocalCostmap = true
  private showGlobalCostmap = true
  private costmapOpacity = 0.5  // 半透明度
  
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
   * 更新局部代价地图
   */
  updateLocalCostmap(costmapMsg: OccupancyGridMsg) {
    this.localCostmap = costmapMsg
    this.render()
  }
  
  /**
   * 更新全局代价地图
   */
  updateGlobalCostmap(costmapMsg: OccupancyGridMsg) {
    this.globalCostmap = costmapMsg
    this.render()
  }
  
  /**
   * 增量更新局部代价地图（接收 OccupancyGridUpdate 消息）
   */
  updateLocalCostmapIncremental(updateMsg: OccupancyGridUpdateMsg) {
    if (!this.localCostmap) {
      console.warn('[MapRenderer] 无法增量更新：缺少完整局部代价地图')
      return
    }
    
    // 验证更新区域边界
    const mapWidth = this.localCostmap.info.width
    const mapHeight = this.localCostmap.info.height
    if (updateMsg.x < 0 || updateMsg.y < 0 ||
        updateMsg.x + updateMsg.width > mapWidth ||
        updateMsg.y + updateMsg.height > mapHeight) {
      console.error('[MapRenderer] 增量更新区域越界:', {
        update: { x: updateMsg.x, y: updateMsg.y, w: updateMsg.width, h: updateMsg.height },
        map: { w: mapWidth, h: mapHeight }
      })
      return
    }
    
    // 确保 data 是数组
    const updateData = Array.isArray(updateMsg.data) ? updateMsg.data : Array.from(updateMsg.data)
    const costmapData = Array.isArray(this.localCostmap.data) 
      ? this.localCostmap.data 
      : Array.from(this.localCostmap.data)
    
    // 应用增量更新：将 updateMsg.data 的矩形区域复制到 costmap.data 对应位置
    for (let row = 0; row < updateMsg.height; row++) {
      const updateRowStart = row * updateMsg.width
      const costmapRowStart = (updateMsg.y + row) * mapWidth + updateMsg.x
      
      for (let col = 0; col < updateMsg.width; col++) {
        costmapData[costmapRowStart + col] = updateData[updateRowStart + col]
      }
    }
    
    // 更新 data（如果是 Int8Array 则转回）
    this.localCostmap.data = this.localCostmap.data instanceof Int8Array
      ? Int8Array.from(costmapData)
      : costmapData
    
    // 重新渲染
    this.render()
  }
  
  /**
   * 增量更新全局代价地图（接收 OccupancyGridUpdate 消息）
   */
  updateGlobalCostmapIncremental(updateMsg: OccupancyGridUpdateMsg) {
    if (!this.globalCostmap) {
      console.warn('[MapRenderer] 无法增量更新：缺少完整全局代价地图')
      return
    }
    
    // 验证更新区域边界
    const mapWidth = this.globalCostmap.info.width
    const mapHeight = this.globalCostmap.info.height
    if (updateMsg.x < 0 || updateMsg.y < 0 ||
        updateMsg.x + updateMsg.width > mapWidth ||
        updateMsg.y + updateMsg.height > mapHeight) {
      console.error('[MapRenderer] 增量更新区域越界:', {
        update: { x: updateMsg.x, y: updateMsg.y, w: updateMsg.width, h: updateMsg.height },
        map: { w: mapWidth, h: mapHeight }
      })
      return
    }
    
    // 确保 data 是数组
    const updateData = Array.isArray(updateMsg.data) ? updateMsg.data : Array.from(updateMsg.data)
    const costmapData = Array.isArray(this.globalCostmap.data) 
      ? this.globalCostmap.data 
      : Array.from(this.globalCostmap.data)
    
    // 应用增量更新
    for (let row = 0; row < updateMsg.height; row++) {
      const updateRowStart = row * updateMsg.width
      const costmapRowStart = (updateMsg.y + row) * mapWidth + updateMsg.x
      
      for (let col = 0; col < updateMsg.width; col++) {
        costmapData[costmapRowStart + col] = updateData[updateRowStart + col]
      }
    }
    
    // 更新 data
    this.globalCostmap.data = this.globalCostmap.data instanceof Int8Array
      ? Int8Array.from(costmapData)
      : costmapData
    
    // 重新渲染
    this.render()
  }
  
  /**
   * 设置 Costmap 显示状态
   */
  setCostmapVisibility(local: boolean, global: boolean) {
    this.showLocalCostmap = local
    this.showGlobalCostmap = global
    this.render()
  }
  
  /**
   * 设置 Costmap 透明度
   */
  setCostmapOpacity(opacity: number) {
    this.costmapOpacity = Math.max(0, Math.min(1, opacity))
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
    
    // Y轴翻转：ROS数据row=0在底部，Canvas ImageData row=0在顶部
    for (let row = 0; row < height; row++) {
      for (let col = 0; col < width; col++) {
        const rosIndex = row * width + col
        const canvasRow = height - 1 - row  // Y轴翻转
        const canvasIndex = canvasRow * width + col
        const occupancy = data[rosIndex]
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
        const pixelIndex = canvasIndex * 4
        imageData.data[pixelIndex] = color[0]     // R
        imageData.data[pixelIndex + 1] = color[1] // G
        imageData.data[pixelIndex + 2] = color[2] // B
        imageData.data[pixelIndex + 3] = 255      // A
      }
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
    
    // 渲染 Costmap（在地图之后，机器人之前）
    if (this.mapData) {
      if (this.showGlobalCostmap && this.globalCostmap) {
        console.log('[MapRenderer] 正在渲染全局代价地图')
        this.renderCostmap(this.globalCostmap, 'global')
      }
      if (this.showLocalCostmap && this.localCostmap) {
        console.log('[MapRenderer] 正在渲染局部代价地图')
        this.renderCostmap(this.localCostmap, 'local')
      }
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
   * 渲染代价地图（半透明彩色叠加）
   * 颜色映射：
   * - 0: 透明（无代价）
   * - 1-99: 蓝→绿→黄（低到中等代价）
   * - 100-254: 红色（高代价）
   * - 255: 黑色（致命障碍物）
   */
  private renderCostmap(costmapMsg: OccupancyGridMsg, _type: 'local' | 'global') {
    const { width, height, origin } = costmapMsg.info
    const data = costmapMsg.data
    const isLocal = _type === 'local'
    
    // 调试：输出 Costmap 信息
    console.log(`[MapRenderer] 渲染${_type}代价地图:`, {
      frame_id: costmapMsg.header.frame_id,
      width, height,
      resolution: costmapMsg.info.resolution,
      origin_x: origin.position.x,
      origin_y: origin.position.y,
      origin_orientation: origin.orientation
    })
    
    if (isLocal && this.robotPose) {
      console.log('[MapRenderer] 机器人当前位姿 (map):', {
        x: this.robotPose.pose.pose.position.x,
        y: this.robotPose.pose.pose.position.y
      })
    }
    
    // 创建临时 canvas
    const tempCanvas = document.createElement('canvas')
    tempCanvas.width = width
    tempCanvas.height = height
    const tempCtx = tempCanvas.getContext('2d', { alpha: true })!
    
    const imageData = tempCtx.createImageData(width, height)
    
    // 统计代价值分布（调试用）
    const stats = { zero: 0, low: 0, mid: 0, high: 0, lethal: 0 }
    
    // Y轴翻转：ROS数据row=0在底部，Canvas ImageData row=0在顶部
    for (let row = 0; row < height; row++) {
      for (let col = 0; col < width; col++) {
        const rosIndex = row * width + col
        const canvasRow = height - 1 - row  // Y轴翻转
        const canvasIndex = canvasRow * width + col
        const cost = data[rosIndex]
        const pixelIndex = canvasIndex * 4
      
        if (cost <= 0) {
          // 无代价或未知：完全透明
          stats.zero++
          imageData.data[pixelIndex] = 0
          imageData.data[pixelIndex + 1] = 0
          imageData.data[pixelIndex + 2] = 0
          imageData.data[pixelIndex + 3] = 0
        } else if (isLocal) {
          // 局部代价地图：粉色（外围）→ 红色（内部）渐变
          if (cost < 50) {
            // 低代价：浅粉色
            stats.low++
            imageData.data[pixelIndex] = 255
            imageData.data[pixelIndex + 1] = 192
            imageData.data[pixelIndex + 2] = 203
            imageData.data[pixelIndex + 3] = Math.floor(cost * 3 * this.costmapOpacity)  // 渐变透明度
          } else if (cost < 100) {
            // 中等代价：粉色→深粉色→红色渐变
            stats.mid++
            const ratio = (cost - 50) / 50  // 0→1
            imageData.data[pixelIndex] = 255  // R保持255
            imageData.data[pixelIndex + 1] = Math.floor(192 * (1 - ratio))  // G: 192→0
            imageData.data[pixelIndex + 2] = Math.floor(203 * (1 - ratio))  // B: 203→0
            imageData.data[pixelIndex + 3] = Math.floor(160 * this.costmapOpacity)
          } else if (cost < 255) {
            // 高代价：鲜红色（最内部）
            stats.high++
            imageData.data[pixelIndex] = 255
            imageData.data[pixelIndex + 1] = 0
            imageData.data[pixelIndex + 2] = 0
            imageData.data[pixelIndex + 3] = Math.floor(200 * this.costmapOpacity)
          } else {
            // 致命障碍物：深红色
            stats.lethal++
            imageData.data[pixelIndex] = 180
            imageData.data[pixelIndex + 1] = 0
            imageData.data[pixelIndex + 2] = 0
            imageData.data[pixelIndex + 3] = Math.floor(230 * this.costmapOpacity)
          }
        } else {
          // 全局代价地图：保持原有配色（蓝→绿→黄→红）
          if (cost < 50) {
            // 低代价：蓝色
            stats.low++
            imageData.data[pixelIndex] = 0
            imageData.data[pixelIndex + 1] = 0
            imageData.data[pixelIndex + 2] = 255
            imageData.data[pixelIndex + 3] = Math.floor(cost * 2.55 * this.costmapOpacity)
          } else if (cost < 100) {
            // 中等代价：蓝→绿→黄渐变
            stats.mid++
            const ratio = (cost - 50) / 50
            imageData.data[pixelIndex] = Math.floor(255 * ratio)      // R: 0→255
            imageData.data[pixelIndex + 1] = 255                       // G: 255
            imageData.data[pixelIndex + 2] = Math.floor(255 * (1 - ratio)) // B: 255→0
            imageData.data[pixelIndex + 3] = Math.floor(128 * this.costmapOpacity)
          } else if (cost < 255) {
            // 高代价：红色
            stats.high++
            imageData.data[pixelIndex] = 255
            imageData.data[pixelIndex + 1] = 0
            imageData.data[pixelIndex + 2] = 0
            imageData.data[pixelIndex + 3] = Math.floor(180 * this.costmapOpacity)
          } else {
            // 致命障碍物：黑色
            stats.lethal++
            imageData.data[pixelIndex] = 0
            imageData.data[pixelIndex + 1] = 0
            imageData.data[pixelIndex + 2] = 0
            imageData.data[pixelIndex + 3] = Math.floor(200 * this.costmapOpacity)
          }
        }
      }
    }
    
    console.log('[MapRenderer] Costmap 代价值统计:', {
      type: isLocal ? 'local' : 'global',
      total: data.length,
      zero: stats.zero,
      low: stats.low,
      mid: stats.mid,
      high: stats.high,
      lethal: stats.lethal,
      nonZero: stats.low + stats.mid + stats.high + stats.lethal
    })
    
    tempCtx.putImageData(imageData, 0, 0)
    
    // 计算 Costmap 在主 canvas 上的位置（与主地图相同的坐标系）
    if (!this.mapData) {
      return
    }
    
    const mapOriginX = this.mapData.info.origin.position.x
    const mapOriginY = this.mapData.info.origin.position.y
    const mapResolution = this.mapData.info.resolution
    const mapHeight = this.mapData.info.height
    
    let costmapOriginX: number
    let costmapOriginY: number
    
    if (isLocal) {
      // 局部代价地图:frame_id='odom',需要转换到 map 坐标系
      if (this.robotPose) {
        // 机器人在 map 坐标系的位置
        const robot_map_x = this.robotPose.pose.pose.position.x
        const robot_map_y = this.robotPose.pose.pose.position.y
        
        // 局部代价地图是以机器人为中心的,机器人在 odom 坐标系的位置 = origin + halfsize
        const costmapRes = costmapMsg.info.resolution
        const halfSizeX = (width * costmapRes) / 2
        const halfSizeY = (height * costmapRes) / 2
        const robot_odom_x = origin.position.x + halfSizeX
        const robot_odom_y = origin.position.y + halfSizeY
        
        // 计算 odom→map 的变换
        const odom_to_map_x = robot_map_x - robot_odom_x
        const odom_to_map_y = robot_map_y - robot_odom_y
        
        // 将 odom 坐标系的 origin 转换到 map 坐标系
        costmapOriginX = origin.position.x + odom_to_map_x
        costmapOriginY = origin.position.y + odom_to_map_y
        
        console.log('[MapRenderer] 局部代价地图 TF 变换:', {
          origin_odom: { x: origin.position.x, y: origin.position.y },
          robot_odom: { x: robot_odom_x, y: robot_odom_y },
          robot_map: { x: robot_map_x, y: robot_map_y },
          odom_to_map: { x: odom_to_map_x, y: odom_to_map_y },
          origin_map: { x: costmapOriginX, y: costmapOriginY }
        })
      } else {
        // 如果没有机器人位姿,降级为直接使用 origin(可能不准确)
        costmapOriginX = origin.position.x
        costmapOriginY = origin.position.y
        console.warn('[MapRenderer] 无机器人位姿,无法进行 TF 变换')
      }
    } else {
      // 全局代价地图：已经在 map 坐标系
      costmapOriginX = origin.position.x
      costmapOriginY = origin.position.y
    }
    
    // Costmap 在 map 坐标系中的物理位置（米）转换为地图栅格坐标
    const gridX = (costmapOriginX - mapOriginX) / mapResolution
    const gridY = (costmapOriginY - mapOriginY) / mapResolution
    
    // 应用 Y 轴翻转（ROS: 左下原点向上，Canvas: 左上原点向下）
    const canvasX = gridX
    const canvasY = mapHeight - gridY - height  // costmap 左上角的 Canvas Y 坐标
    
    console.log('[MapRenderer] Costmap 坐标转换:', {
      type: isLocal ? 'local' : 'global',
      ros_origin: { x: costmapOriginX, y: costmapOriginY },
      grid: { x: gridX, y: gridY },
      canvas: { x: canvasX, y: canvasY },
      canvas_center: { x: canvasX + width/2, y: canvasY + height/2 },
      screen_center: { 
        x: (canvasX + width/2) * this.transform.scale + this.transform.offsetX,
        y: (canvasY + height/2) * this.transform.scale + this.transform.offsetY
      },
      costmapSize: { w: width, h: height },
      mapSize: { w: this.mapData.info.width, h: mapHeight }
    })
    
    // 应用与主地图相同的变换并绘制
    this.ctx.save()
    this.ctx.globalAlpha = 1.0  // 透明度已在 imageData 中处理
    this.ctx.translate(this.transform.offsetX, this.transform.offsetY)
    this.ctx.scale(this.transform.scale, this.transform.scale)
    this.ctx.drawImage(tempCanvas, canvasX, canvasY)
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
    
    // 调试：输出机器人的屏幕坐标
    console.log('[MapRenderer] 机器人屏幕坐标:', {
      ros: { x: position.x, y: position.y },
      screen: screenPos
    })
    
    this.ctx.save()
    this.ctx.translate(screenPos.x, screenPos.y)
    this.ctx.rotate(-yaw)  // Canvas Y轴向下，需要取反
    
    // 绘制箭头（长度7像素，缩小到原来的1/3）
    const arrowLength = 7 * this.transform.scale
    const arrowWidth = 3.5 * this.transform.scale
    
    this.ctx.fillStyle = '#1890ff'  // Ant Design 蓝色
    this.ctx.beginPath()
    this.ctx.moveTo(arrowLength, 0)                    // 箭头尖端
    this.ctx.lineTo(-arrowLength / 2, arrowWidth / 2)  // 左下
    this.ctx.lineTo(-arrowLength / 2, -arrowWidth / 2) // 左上
    this.ctx.closePath()
    this.ctx.fill()
    
    // 绘制圆形底座（缩小到1/3，原5px）
    this.ctx.fillStyle = '#1890ff'
    this.ctx.beginPath()
    this.ctx.arc(0, 0, 1.7 * this.transform.scale, 0, Math.PI * 2)
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
    
    // 绘制箭头（长度10像素，缩小到原来的1/3）
    const arrowLength = 10 * this.transform.scale
    const arrowWidth = 5 * this.transform.scale
    
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
    
    // 绘制圆形底座（缩小到1/3，原7px）
    this.ctx.strokeStyle = '#000'
    this.ctx.lineWidth = 1
    this.ctx.beginPath()
    this.ctx.arc(0, 0, 2.3 * this.transform.scale, 0, Math.PI * 2)
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
