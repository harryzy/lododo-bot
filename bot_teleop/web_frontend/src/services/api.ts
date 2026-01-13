/**
 * API 服务
 * 封装所有 REST API 调用
 */

import axios from 'axios'

const API_BASE_URL = '/api'

// 创建 axios 实例
const apiClient = axios.create({
  baseURL: API_BASE_URL,
  timeout: 10000,
  headers: {
    'Content-Type': 'application/json',
  },
})

// 响应拦截器
apiClient.interceptors.response.use(
  (response) => response.data,
  (error) => {
    console.error('[API] Error:', error)
    return Promise.reject(error)
  }
)

// ============================================
// 任务管理 API
// ============================================

export interface NavigationRequest {
  x: number
  y: number
  yaw?: number
}

export interface ExplorationRequest {
  map_name?: string
  save_on_completion?: boolean
}

export interface PatrolRequest {
  waypoint_file: string
  mode?: string
}

export interface TaskResponse {
  success: boolean
  message: string
  request_id: string
  task_id?: string
  task_data?: any  // 任务完整数据（从缓存）
}

const tasks = {
  // 创建导航任务
  createNavigationTask: (data: NavigationRequest) =>
    apiClient.post<any, TaskResponse>('/tasks/navigate', data),

  // 创建探索任务
  createExplorationTask: (data: ExplorationRequest) =>
    apiClient.post<any, TaskResponse>('/tasks/exploration', data),

  // 创建巡逻任务
  createPatrolTask: (data: PatrolRequest) =>
    apiClient.post<any, TaskResponse>('/tasks/patrol', data),

  // 查询任务状态
  getTaskStatus: (task_id?: string) =>
    apiClient.get<any, any>('/tasks/status', { params: { task_id } }),
  
  // 查询单个任务状态（用户手动查询）
  queryTaskStatus: (task_id: string) =>
    apiClient.post<any, TaskResponse>(`/tasks/${task_id}/query`),

  // 取消任务
  cancelTask: (task_id: string | number) =>
    apiClient.post<any, TaskResponse>('/tasks/cancel', { task_id }),

  // 暂停任务
  pauseTask: (task_id: string) =>
    apiClient.post<any, TaskResponse>(`/tasks/${task_id}/pause`),

  // 恢复任务
  resumeTask: (task_id: string) =>
    apiClient.post<any, TaskResponse>(`/tasks/${task_id}/resume`),

  // 紧急停止
  emergencyStop: () =>
    apiClient.post<any, TaskResponse>('/tasks/emergency_stop'),

  // 获取任务列表（活跃 + 历史）
  listTasks: () =>
    apiClient.get<any, any>('/tasks/list'),
  
  // 获取活跃任务
  getActiveTasks: () =>
    apiClient.get<any, any>('/tasks/active'),
  
  // 获取任务历史
  getTaskHistory: (limit?: number) =>
    apiClient.get<any, any>('/tasks/history', { params: { limit } }),
}

// ============================================
// 地图管理 API
// ============================================

const maps = {
  // 获取地图列表
  listMaps: () =>
    apiClient.get<any, any>('/maps'),

  // 获取地图详情
  getMapInfo: (map_name: string) =>
    apiClient.get<any, any>(`/maps/${map_name}`),

  // 加载地图
  loadMap: (map_name: string) =>
    apiClient.post<any, any>('/maps/load', { map_name }),

  // 保存地图
  saveMap: (map_name: string, description?: string, tags?: string[]) =>
    apiClient.post<any, any>('/maps/save', { map_name, description, tags }),

  // 删除地图
  deleteMap: (map_name: string) =>
    apiClient.delete<any, any>(`/maps/${map_name}`),
}

// ============================================
// 路点管理 API
// ============================================

export interface Waypoint {
  name: string
  x: number
  y: number
  yaw: number
  dwell_time?: number
}

const waypoints = {
  // 获取路点路线列表
  listRoutes: () =>
    apiClient.get<any, any>('/waypoints'),

  // 获取路点路线详情
  getRoute: (route_name: string) =>
    apiClient.get<any, any>(`/waypoints/${route_name}`),

  // 保存路点路线
  saveRoute: (route_name: string, waypoints: Waypoint[], description?: string) =>
    apiClient.post<any, any>('/waypoints', { route_name, waypoints, description }),

  // 更新路点路线
  updateRoute: (route_name: string, waypoints: Waypoint[], description?: string) =>
    apiClient.put<any, any>(`/waypoints/${route_name}`, { waypoints, description }),

  // 删除路点路线
  deleteRoute: (route_name: string) =>
    apiClient.delete<any, any>(`/waypoints/${route_name}`),

  // 添加路点到路线
  addWaypoint: (route_name: string, waypoint: Waypoint) =>
    apiClient.post<any, any>(`/waypoints/${route_name}/add`, waypoint),
}

// ============================================
// 设置管理 API
// ============================================

const settings = {
  // 获取所有设置
  getSettings: () =>
    apiClient.get<any, any>('/settings'),

  // 获取指定类别设置
  getSettingsByCategory: (category: string) =>
    apiClient.get<any, any>(`/settings/${category}`),

  // 更新设置
  updateSettings: (settings: any) =>
    apiClient.put<any, any>('/settings', { settings }),

  // 更新指定类别设置
  updateSettingsByCategory: (category: string, settings: any) =>
    apiClient.put<any, any>(`/settings/${category}`, settings),

  // 重置设置
  resetSettings: () =>
    apiClient.post<any, any>('/settings/reset'),

  // 获取系统信息
  getSystemInfo: () =>
    apiClient.get<any, any>('/settings/system/info'),
}

// 导出 API 服务
export const apiService = {
  tasks,
  maps,
  waypoints,
  settings,
}

// 默认导出
export default apiService;

// 单独导出各模块（兼容性）
export { tasks, maps, waypoints, settings };
