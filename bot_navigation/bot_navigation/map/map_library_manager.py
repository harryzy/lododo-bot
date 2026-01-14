#!/usr/bin/env python3
"""
地图库管理器 / Map Library Manager

功能:
- 地图持久化 (保存到YAML/PGM)
- 地图元数据管理
- 版本控制
- 地图加载和切换
- 自动清理策略

Author: GitHub Copilot
Date: 2025-12-21
"""

import os
import yaml
import shutil
from dataclasses import dataclass, asdict, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import subprocess

from rclpy.node import Node

try:
    from PIL import Image
    import numpy as np
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False


@dataclass
class MapMetadata:
    """地图元数据 / Map metadata"""
    map_name: str
    version: int = 1
    created_at: str = field(default_factory=lambda: datetime.now().isoformat())
    updated_at: str = field(default_factory=lambda: datetime.now().isoformat())
    resolution: float = 0.05
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_z: float = 0.0
    width: int = 0
    height: int = 0
    occupied_thresh: float = 0.65
    free_thresh: float = 0.196
    negate: int = 0
    description: str = ""
    tags: List[str] = field(default_factory=list)
    file_path: str = ""  # 地图文件路径 (不含扩展名)
    
    def to_dict(self) -> dict:
        """转换为字典"""
        return asdict(self)
    
    @staticmethod
    def from_dict(data: dict) -> 'MapMetadata':
        """从字典创建"""
        return MapMetadata(**data)


class MapLibraryManager:
    """
    地图库管理器 / Map library manager
    
    负责地图的持久化存储、版本管理和加载
    """
    
    def __init__(self, node: Node, library_path: str):
        """
        初始化地图库管理器
        
        Args:
            node: ROS节点
            library_path: 地图库根目录
        """
        self.node = node
        self.library_path = Path(library_path).expanduser().resolve()
        self.metadata_file = self.library_path / "map_library.yaml"
        
        # 确保目录存在
        self.library_path.mkdir(parents=True, exist_ok=True)
        
        # 加载或初始化地图库索引
        self.map_index: Dict[str, MapMetadata] = self._load_index()
        
        self.node.get_logger().info(
            f"📚 地图库管理器已初始化: {self.library_path}"
        )
        self.node.get_logger().info(
            f"   - 已加载 {len(self.map_index)} 张地图"
        )
    
    def _load_index(self) -> Dict[str, MapMetadata]:
        """加载地图库索引 / Load map library index"""
        if not self.metadata_file.exists():
            self.node.get_logger().info("地图库索引文件不存在，创建新索引")
            return {}
        
        try:
            with open(self.metadata_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            
            index = {}
            for map_name, meta_dict in data.get('maps', {}).items():
                index[map_name] = MapMetadata.from_dict(meta_dict)
            
            return index
        
        except Exception as e:
            self.node.get_logger().error(f"加载地图库索引失败: {e}")
            return {}
    
    def _save_index(self) -> bool:
        """保存地图库索引 / Save map library index"""
        try:
            data = {
                'maps': {
                    name: meta.to_dict() 
                    for name, meta in self.map_index.items()
                }
            }
            
            with open(self.metadata_file, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, allow_unicode=True, default_flow_style=False)
            
            return True
        
        except Exception as e:
            self.node.get_logger().error(f"保存地图库索引失败: {e}")
            return False
    
    def save_map(
        self,
        map_name: str,
        map_topic: str = '/map',
        description: str = "",
        tags: List[str] = None
    ) -> Tuple[bool, str]:
        """
        保存当前地图到地图库 / Save current map to library
        
        Args:
            map_name: 地图名称
            map_topic: 地图话题
            description: 地图描述
            tags: 标签列表
        
        Returns:
            (success, message)
        """
        try:
            # 检查地图名称是否已存在
            if map_name in self.map_index:
                # 版本递增
                old_meta = self.map_index[map_name]
                version = old_meta.version + 1
                self.node.get_logger().info(
                    f"地图 '{map_name}' 已存在，版本升级: v{old_meta.version} → v{version}"
                )
            else:
                version = 1
            
            # 创建地图目录
            map_dir = self.library_path / map_name
            map_dir.mkdir(parents=True, exist_ok=True)
            
            # 生成文件路径 (不含扩展名)
            map_file = map_dir / f"{map_name}_v{version}"
            
            # 等待/map话题可用并有数据发布 / Wait for /map topic with data
            self.node.get_logger().info("⏳ 等待 /map 话题准备就绪...")
            
            # 首先检查话题是否存在
            import time
            max_wait_time = 15.0  # 最多等待15秒
            start_time = time.time()
            topic_found = False
            
            while (time.time() - start_time) < max_wait_time:
                # 检查话题是否存在
                check_cmd = ['ros2', 'topic', 'list']
                result = subprocess.run(check_cmd, capture_output=True, text=True)
                if map_topic in result.stdout:
                    topic_found = True
                    self.node.get_logger().info(f"✅ {map_topic} 话题已存在")
                    break
                time.sleep(0.5)
            
            if not topic_found:
                error_msg = f"{map_topic} 话题在{max_wait_time}秒内未出现"
                self.node.get_logger().error(error_msg)
                return False, error_msg
            
            # 检查是否有发布者 / Check if there are publishers
            # 使用详细模式获取QoS信息 / Use verbose mode for QoS info
            info_cmd = ['ros2', 'topic', 'info', map_topic, '-v']
            result = subprocess.run(info_cmd, capture_output=True, text=True, timeout=5)
            
            # 解析发布者数量 / Parse publisher count
            publisher_count = 0
            for line in result.stdout.split('\n'):
                if line.strip().startswith('Publisher count:'):
                    try:
                        publisher_count = int(line.split(':')[1].strip())
                        break
                    except (ValueError, IndexError):
                        pass
            
            if publisher_count == 0:
                # 警告但不失败：SLAM完成后发布者可能已关闭，但latched消息仍可用
                # Warning but not fatal: Publisher may be closed after SLAM completion, but latched message may still be available
                self.node.get_logger().warn(
                    f"⚠️ {map_topic} 话题当前无发布者，尝试订阅最后一次latched消息..."
                )
                self.node.get_logger().info(f"话题信息（用于诊断）:\n{result.stdout}")
                # 继续尝试保存，依赖TRANSIENT_LOCAL QoS的latched消息
            
            self.node.get_logger().info(f"✅ /map 话题已就绪，发布者数量: {publisher_count}")
            
            # 如果没有发布者，等待更长时间确保latched消息可用
            # If no publishers, wait longer to ensure latched message is available
            wait_time = 3.0 if publisher_count == 0 else 1.0
            self.node.get_logger().info(f"⏳ 等待{wait_time}秒以确保地图数据可用...")
            time.sleep(wait_time)
            
            # 调用 map_saver 保存地图 / Call map_saver to save map
            # 使用正确的ROS 2参数格式
            cmd = [
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', str(map_file),
                '-t', map_topic,  # 明确指定话题
                '--ros-args',
                '-p', 'save_map_timeout:=10.0',  # 增加超时
                '-p', 'free_thresh_default:=0.25',
                '-p', 'occupied_thresh_default:=0.65',
            ]
            
            self.node.get_logger().info(f"🗺️ 正在保存地图: {map_file}")
            self.node.get_logger().info(f"   命令: {' '.join(cmd)}")
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=60  # 增加超时到60秒
            )
            
            if result.returncode != 0:
                error_msg = f"地图保存失败: {result.stderr}"
                self.node.get_logger().error(error_msg)
                return False, error_msg
            
            # 检查文件是否生成
            yaml_file = Path(str(map_file) + '.yaml')
            pgm_file = Path(str(map_file) + '.pgm')
            
            if not yaml_file.exists() or not pgm_file.exists():
                error_msg = "地图文件未生成"
                self.node.get_logger().error(error_msg)
                return False, error_msg
            
            # 生成可视化PNG图片
            self._generate_visualization(pgm_file, map_file)
            
            # 🎯 关键：保存 RTABMap 数据库
            # RTABMap 默认使用 ~/.ros/rtabmap.db 作为数据库文件
            self._save_rtabmap_database(map_dir, map_name, version)
            
            # 读取地图 YAML 文件获取元数据
            with open(yaml_file, 'r') as f:
                map_yaml = yaml.safe_load(f)
            
            # 创建元数据
            metadata = MapMetadata(
                map_name=map_name,
                version=version,
                updated_at=datetime.now().isoformat(),
                resolution=map_yaml.get('resolution', 0.05),
                origin_x=map_yaml.get('origin', [0, 0, 0])[0],
                origin_y=map_yaml.get('origin', [0, 0, 0])[1],
                origin_z=map_yaml.get('origin', [0, 0, 0])[2],
                occupied_thresh=map_yaml.get('occupied_thresh', 0.65),
                free_thresh=map_yaml.get('free_thresh', 0.196),
                negate=map_yaml.get('negate', 0),
                description=description,
                tags=tags or [],
                file_path=str(map_file)
            )
            
            # 如果是第一个版本，设置created_at
            if map_name in self.map_index:
                metadata.created_at = self.map_index[map_name].created_at
            
            # 更新索引
            self.map_index[map_name] = metadata
            
            # 保存索引
            if not self._save_index():
                return False, "索引保存失败"
            
            self.node.get_logger().info(
                f"✅ 地图保存成功: {map_name} (v{version})"
            )
            return True, f"地图已保存: {map_name} (v{version})"
        
        except subprocess.TimeoutExpired:
            error_msg = "地图保存超时"
            self.node.get_logger().error(error_msg)
            return False, error_msg
        
        except Exception as e:
            error_msg = f"地图保存异常: {str(e)}"
            self.node.get_logger().error(error_msg)
            return False, error_msg
    
    def load_map(self, map_name: str, version: Optional[int] = None) -> Tuple[bool, str, Optional[str]]:
        """
        加载地图 / Load map
        
        Args:
            map_name: 地图名称
            version: 版本号 (None=最新版本)
        
        Returns:
            (success, message, map_yaml_path)
        """
        try:
            # 检查地图是否存在
            if map_name not in self.map_index:
                return False, f"地图 '{map_name}' 不存在", None
            
            metadata = self.map_index[map_name]
            
            # 确定要加载的版本
            if version is None:
                version = metadata.version
            
            # 构建文件路径
            map_dir = self.library_path / map_name
            map_file = map_dir / f"{map_name}_v{version}.yaml"
            
            if not map_file.exists():
                return False, f"地图文件不存在: {map_file}", None
            
            self.node.get_logger().info(
                f"📍 地图加载路径: {map_file}"
            )
            
            return True, f"地图 '{map_name}' (v{version}) 已准备", str(map_file)
        
        except Exception as e:
            error_msg = f"地图加载失败: {str(e)}"
            self.node.get_logger().error(error_msg)
            return False, error_msg, None
    
    def list_maps(self) -> List[MapMetadata]:
        """列出所有地图 / List all maps"""
        return list(self.map_index.values())
    
    def get_map_metadata(self, map_name: str) -> Optional[MapMetadata]:
        """获取地图元数据 / Get map metadata"""
        return self.map_index.get(map_name)
    
    def delete_map(self, map_name: str, all_versions: bool = False) -> Tuple[bool, str]:
        """
        删除地图 / Delete map
        
        Args:
            map_name: 地图名称
            all_versions: 是否删除所有版本
        
        Returns:
            (success, message)
        """
        try:
            if map_name not in self.map_index:
                return False, f"地图 '{map_name}' 不存在"
            
            map_dir = self.library_path / map_name
            
            if all_versions:
                # 删除整个目录
                if map_dir.exists():
                    shutil.rmtree(map_dir)
                
                # 从索引移除
                del self.map_index[map_name]
                self._save_index()
                
                self.node.get_logger().info(f"🗑️ 已删除地图所有版本: {map_name}")
                return True, f"已删除地图 '{map_name}' 的所有版本"
            
            else:
                # 只删除最新版本
                metadata = self.map_index[map_name]
                version = metadata.version
                
                map_file_base = map_dir / f"{map_name}_v{version}"
                yaml_file = Path(str(map_file_base) + '.yaml')
                pgm_file = Path(str(map_file_base) + '.pgm')
                
                if yaml_file.exists():
                    yaml_file.unlink()
                if pgm_file.exists():
                    pgm_file.unlink()
                
                # 如果只有一个版本，删除整个目录和索引
                if version == 1:
                    if map_dir.exists():
                        shutil.rmtree(map_dir)
                    del self.map_index[map_name]
                else:
                    # 更新版本号
                    metadata.version = version - 1
                    metadata.updated_at = datetime.now().isoformat()
                
                self._save_index()
                
                self.node.get_logger().info(
                    f"🗑️ 已删除地图版本: {map_name} v{version}"
                )
                return True, f"已删除地图 '{map_name}' 版本 v{version}"
        
        except Exception as e:
            error_msg = f"删除地图失败: {str(e)}"
            self.node.get_logger().error(error_msg)
            return False, error_msg
    
    def search_maps(self, keyword: str = "", tags: List[str] = None) -> List[MapMetadata]:
        """
        搜索地图 / Search maps
        
        Args:
            keyword: 关键词 (搜索名称和描述)
            tags: 标签列表
        
        Returns:
            匹配的地图列表
        """
        results = []
        
        for metadata in self.map_index.values():
            # 关键词匹配
            if keyword:
                keyword_lower = keyword.lower()
                if (keyword_lower not in metadata.map_name.lower() and
                    keyword_lower not in metadata.description.lower()):
                    continue
            
            # 标签匹配
            if tags:
                if not any(tag in metadata.tags for tag in tags):
                    continue
            
            results.append(metadata)
        
        return results
    
    def cleanup_old_versions(self, keep_versions: int = 3) -> int:
        """
        清理旧版本地图 / Cleanup old map versions
        
        Args:
            keep_versions: 保留的版本数量
        
        Returns:
            删除的文件数量
        """
        deleted_count = 0
        
        for map_name, metadata in self.map_index.items():
            map_dir = self.library_path / map_name
            
            if not map_dir.exists():
                continue
            
            # 获取所有版本文件
            version_files = []
            for f in map_dir.glob(f"{map_name}_v*.yaml"):
                try:
                    version = int(f.stem.split('_v')[1])
                    version_files.append((version, f))
                except:
                    continue
            
            # 按版本号排序
            version_files.sort(reverse=True)
            
            # 保留最新的N个版本
            for version, yaml_file in version_files[keep_versions:]:
                pgm_file = yaml_file.with_suffix('.pgm')
                
                try:
                    yaml_file.unlink()
                    if pgm_file.exists():
                        pgm_file.unlink()
                    deleted_count += 2
                    self.node.get_logger().info(
                        f"🗑️ 清理旧版本: {map_name} v{version}"
                    )
                except Exception as e:
                    self.node.get_logger().error(
                        f"删除文件失败 {yaml_file}: {e}"
                    )
        
        return deleted_count
    
    def _save_rtabmap_database(self, map_dir: Path, map_name: str, version: int):
        """
        保存 RTABMap 数据库 / Save RTABMap database
        
        Args:
            map_dir: 地图目录
            map_name: 地图名称
            version: 版本号
        """
        try:
            # RTABMap 数据库的常见位置
            possible_db_paths = [
                Path.home() / '.ros' / 'rtabmap.db',  # 默认位置
                Path('/tmp') / 'rtabmap.db',  # 临时文件位置
            ]
            
            # 尝试查找 RTABMap 数据库文件
            rtabmap_db_src = None
            for db_path in possible_db_paths:
                if db_path.exists():
                    rtabmap_db_src = db_path
                    self.node.get_logger().info(f"✅ 找到 RTABMap 数据库: {db_path}")
                    break
            
            if rtabmap_db_src is None:
                self.node.get_logger().warn(
                    "⚠️  未找到 RTABMap 数据库文件，跳过保存"
                )
                return
            
            # 目标文件名：<map_name>_v<version>.db
            rtabmap_db_dest = map_dir / f"{map_name}_v{version}.db"
            
            # 复制数据库文件
            import shutil
            shutil.copy2(rtabmap_db_src, rtabmap_db_dest)
            
            # 检查文件大小
            db_size_mb = rtabmap_db_dest.stat().st_size / (1024 * 1024)
            
            self.node.get_logger().info(
                f"💾 RTABMap 数据库已保存: {rtabmap_db_dest.name} ({db_size_mb:.2f} MB)"
            )
            
        except Exception as e:
            self.node.get_logger().error(
                f"❌ 保存 RTABMap 数据库失败: {str(e)}"
            )
    
    def _generate_visualization(self, pgm_file: Path, output_base: Path):
        """
        生成地图可视化PNG图片 / Generate map visualization PNG
        
        Args:
            pgm_file: PGM文件路径
            output_base: 输出文件基础路径（不含扩展名）
        """
        if not PIL_AVAILABLE:
            self.node.get_logger().warn(
                "PIL/Pillow未安装，无法生成可视化图片。"
                "安装: pip install Pillow"
            )
            return
        
        try:
            # 读取PGM文件
            img = Image.open(pgm_file)
            img_array = np.array(img)
            
            # 创建彩色可视化图像 (RGB)
            # PGM格式: 255=free(白), 0=occupied(黑), 205=unknown(灰)
            vis_array = np.zeros((*img_array.shape, 3), dtype=np.uint8)
            
            # 自由空间 (白色) - 值接近255
            free_mask = img_array >= 250
            vis_array[free_mask] = [255, 255, 255]
            
            # 占据空间 (黑色) - 值接近0
            occupied_mask = img_array <= 50
            vis_array[occupied_mask] = [0, 0, 0]
            
            # 未知空间 (灰色) - 中间值
            unknown_mask = (img_array > 50) & (img_array < 250)
            vis_array[unknown_mask] = [128, 128, 128]
            
            # 添加网格线（可选，每10像素一条淡线）
            grid_spacing = 20
            for i in range(0, img_array.shape[0], grid_spacing):
                vis_array[i, :] = np.clip(vis_array[i, :].astype(int) * 0.95, 0, 255).astype(np.uint8)
            for j in range(0, img_array.shape[1], grid_spacing):
                vis_array[:, j] = np.clip(vis_array[:, j].astype(int) * 0.95, 0, 255).astype(np.uint8)
            
            # 保存为PNG
            png_file = Path(str(output_base) + '_preview.png')
            vis_img = Image.fromarray(vis_array)
            vis_img.save(png_file)
            
            self.node.get_logger().info(f"🖼️ 可视化图片已生成: {png_file.name}")
            
        except Exception as e:
            self.node.get_logger().error(f"生成可视化图片失败: {e}")


def main(args=None):
    """测试用主函数 / Test main function"""
    import rclpy
    from rclpy.node import Node
    
    rclpy.init(args=args)
    node = Node('map_library_test')
    
    # 创建地图库管理器
    manager = MapLibraryManager(
        node=node,
        library_path='~/lododo_bot/maps'
    )
    
    # 测试功能
    node.get_logger().info("\n=== 地图库管理器测试 ===")
    
    # 列出所有地图
    maps = manager.list_maps()
    node.get_logger().info(f"\n当前地图数量: {len(maps)}")
    for meta in maps:
        node.get_logger().info(
            f"  - {meta.map_name} (v{meta.version}): {meta.description}"
        )
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
