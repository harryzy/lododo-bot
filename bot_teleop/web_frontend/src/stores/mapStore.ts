/**
 * 地图状态存储
 * 管理地图数据和机器人位姿
 */

import { create } from 'zustand'

interface MapState {
  mapData: any | null
  robotPose: {
    x: number
    y: number
    z: number
    orientation: {
      x: number
      y: number
      z: number
      w: number
    }
  } | null
  setMapData: (data: any) => void
  setRobotPose: (pose: any) => void
}

export const useMapStore = create<MapState>((set) => ({
  mapData: null,
  robotPose: null,

  setMapData: (data: any) => {
    set({ mapData: data })
  },

  setRobotPose: (pose: any) => {
    set({ robotPose: pose })
  },
}))
