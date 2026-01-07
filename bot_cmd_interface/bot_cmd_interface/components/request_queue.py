"""
请求队列模块 / Request queue module

实现线程安全的FIFO队列，支持去重和优先级管理
Implements thread-safe FIFO queue with deduplication and priority management
"""

import queue
import threading
import time
import hashlib
import json
from typing import Tuple, Optional
from datetime import datetime

from ..sdk.message import CommandRequest
from ..sdk.action_types import ActionType


class RequestQueue:
    """
    请求队列 - 基于请求ID的FIFO队列 / Request queue - FIFO queue based on request ID
    
    功能 / Features:
    1. 线程安全的队列操作 / Thread-safe queue operations
    2. 请求ID去重（5秒内相同内容）/ Request ID deduplication (5s window)
    3. 优先级管理（仅emergency_stop优先）/ Priority management (only emergency_stop has priority)
    4. 队列大小限制 / Queue size limit
    
    设计说明 / Design Notes:
    - emergency_stop请求插队到队首
    - 其他请求严格按FIFO顺序
    - 去重基于请求内容哈希（忽略request_id和timestamp）
    - 使用Python queue.Queue确保线程安全
    """
    
    def __init__(self, max_size: int = 100, dedup_window: float = 5.0):
        """
        初始化请求队列 / Initialize request queue
        
        Args:
            max_size: 队列最大容量 / Maximum queue size
            dedup_window: 去重时间窗口（秒）/ Deduplication time window (seconds)
        """
        self.max_size = max_size
        self.dedup_window = dedup_window
        
        # 使用两个队列：普通队列和紧急队列 / Use two queues: normal and emergency
        self.normal_queue = queue.Queue(maxsize=max_size)
        self.emergency_queue = queue.Queue(maxsize=10)  # 紧急队列容量较小
        
        # 去重缓存：{content_hash: (request_id, timestamp)} / Dedup cache
        self.dedup_cache = {}
        self.cache_lock = threading.Lock()
        
        # 统计信息 / Statistics
        self.total_enqueued = 0
        self.total_dequeued = 0
        self.total_rejected = 0
        
        # 启动缓存清理线程 / Start cache cleanup thread
        self.cleanup_thread = threading.Thread(
            target=self._cleanup_expired_cache,
            daemon=True
        )
        self.cleanup_thread.start()
    
    def enqueue(self, request: CommandRequest) -> Tuple[bool, str]:
        """
        入队请求 / Enqueue request
        
        Args:
            request: 命令请求对象 / Command request object
        
        Returns:
            Tuple[bool, str]: (成功标志, 消息) / (success flag, message)
        """
        # 1. 计算内容哈希（用于去重）/ Calculate content hash for deduplication
        content_hash = self._calculate_content_hash(request)
        
        # 2. 检查是否重复 / Check for duplicates
        with self.cache_lock:
            if content_hash in self.dedup_cache:
                cached_id, cached_time = self.dedup_cache[content_hash]
                time_diff = time.time() - cached_time
                
                if time_diff < self.dedup_window:
                    self.total_rejected += 1
                    return False, f"Duplicate request within {self.dedup_window}s window (original request_id: {cached_id})"
        
        # 3. 根据优先级选择队列 / Select queue based on priority
        try:
            if request.action == ActionType.EMERGENCY_STOP:
                # 紧急停止请求进入紧急队列 / Emergency stop goes to emergency queue
                self.emergency_queue.put(request, block=False)
            else:
                # 其他请求进入普通队列 / Other requests go to normal queue
                self.normal_queue.put(request, block=False)
            
            # 4. 更新去重缓存 / Update dedup cache
            with self.cache_lock:
                self.dedup_cache[content_hash] = (request.request_id, time.time())
            
            self.total_enqueued += 1
            return True, "Request enqueued successfully"
        
        except queue.Full:
            self.total_rejected += 1
            return False, f"Queue is full (max_size={self.max_size})"
    
    def dequeue(self, timeout: float = None) -> Optional[CommandRequest]:
        """
        出队请求 / Dequeue request
        
        优先级：emergency_queue > normal_queue
        Priority: emergency_queue > normal_queue
        
        Args:
            timeout: 等待超时时间（秒），None表示阻塞 / Wait timeout (seconds), None means blocking
        
        Returns:
            CommandRequest: 请求对象，无请求则返回None / Request object, or None if no request
        """
        try:
            # 1. 优先检查紧急队列 / Check emergency queue first
            if not self.emergency_queue.empty():
                request = self.emergency_queue.get(block=False)
                self.total_dequeued += 1
                return request
            
            # 2. 从普通队列取请求 / Get from normal queue
            request = self.normal_queue.get(block=True, timeout=timeout)
            self.total_dequeued += 1
            return request
        
        except queue.Empty:
            return None
    
    def size(self) -> int:
        """
        获取队列大小 / Get queue size
        
        Returns:
            int: 总队列大小（emergency + normal）/ Total queue size
        """
        return self.emergency_queue.qsize() + self.normal_queue.qsize()
    
    def clear(self):
        """清空队列 / Clear queue"""
        # 清空两个队列 / Clear both queues
        while not self.emergency_queue.empty():
            try:
                self.emergency_queue.get(block=False)
            except queue.Empty:
                break
        
        while not self.normal_queue.empty():
            try:
                self.normal_queue.get(block=False)
            except queue.Empty:
                break
        
        # 清空去重缓存 / Clear dedup cache
        with self.cache_lock:
            self.dedup_cache.clear()
    
    def get_statistics(self) -> dict:
        """
        获取统计信息 / Get statistics
        
        Returns:
            dict: 统计数据 / Statistics data
        """
        return {
            'current_size': self.size(),
            'emergency_queue_size': self.emergency_queue.qsize(),
            'normal_queue_size': self.normal_queue.qsize(),
            'total_enqueued': self.total_enqueued,
            'total_dequeued': self.total_dequeued,
            'total_rejected': self.total_rejected,
            'dedup_cache_size': len(self.dedup_cache)
        }
    
    def _calculate_content_hash(self, request: CommandRequest) -> str:
        """
        计算请求内容哈希（用于去重）/ Calculate request content hash for deduplication
        
        忽略request_id和timestamp，只基于action和params计算哈希
        Ignores request_id and timestamp, only hashes action and params
        
        Args:
            request: 命令请求对象 / Command request object
        
        Returns:
            str: 内容哈希值 / Content hash
        """
        content = {
            'action': request.action,
            'params': request.params,
            'priority': request.priority
        }
        
        # 使用JSON序列化后计算MD5哈希 / Calculate MD5 hash after JSON serialization
        content_json = json.dumps(content, sort_keys=True)
        return hashlib.md5(content_json.encode()).hexdigest()
    
    def _cleanup_expired_cache(self):
        """
        后台线程：定期清理过期的去重缓存 / Background thread: Periodically clean expired dedup cache
        
        每30秒清理一次超过dedup_window的缓存项
        Cleans cache entries older than dedup_window every 30 seconds
        """
        while True:
            time.sleep(30.0)  # 每30秒清理一次 / Clean every 30 seconds
            
            current_time = time.time()
            expired_keys = []
            
            with self.cache_lock:
                for content_hash, (request_id, timestamp) in self.dedup_cache.items():
                    if current_time - timestamp > self.dedup_window:
                        expired_keys.append(content_hash)
                
                for key in expired_keys:
                    del self.dedup_cache[key]
