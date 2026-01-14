/**
 * 地图管理相关类型定义
 */

export interface MapInfo {
  name: string;
  version: number;
  versions: number[];  // 所有可用版本列表
  size: string;
  resolution: number;
  created_at: string;
  updated_at?: string;
  description?: string;
  tags: string[];
  thumbnail_url?: string;
  has_rtabmap_db: boolean;
  has_pgm: boolean;
  has_yaml: boolean;
}

export interface MapListResponse {
  success: boolean;
  maps: MapInfo[];
  total_count: number;
  rtabmap_count: number;
  last_updated?: string;
}

export interface LoadMapRequest {
  map_name: string;
  version?: number;
}

export interface DeleteMapResponse {
  success: boolean;
  message: string;
}

export interface VersionInfo {
  version: number;
  is_current: boolean;
  size: string;
  has_rtabmap_db: boolean;
  has_pgm: boolean;
  has_yaml: boolean;
  created_at?: string;
}

export interface VersionListResponse {
  success: boolean;
  map_name: string;
  current_version: number;
  versions: VersionInfo[];
}

export interface SwitchVersionRequest {
  version: number;
}

export interface SwitchVersionResponse {
  success: boolean;
  message: string;
  new_version: number;
}
