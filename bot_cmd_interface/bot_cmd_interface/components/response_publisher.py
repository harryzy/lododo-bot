"""
响应发布器模块 / Response publisher module

负责将CommandResponse发布到/cmd/response Topic
Responsible for publishing CommandResponse to /cmd/response Topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ..sdk.message import CommandResponse


class ResponsePublisher:
    """
    响应发布器 - 将CommandResponse发布到/cmd/response / Response publisher - Publish CommandResponse to /cmd/response
    
    功能 / Features:
    1. 发布响应到Topic / Publish responses to topic
    2. 响应日志记录 / Response logging
    3. 统计信息收集 / Statistics collection
    
    设计说明 / Design Notes:
    - 使用std_msgs/String封装JSON格式响应
    - 自动记录发布日志（info级别）
    - 提供统计信息查询接口
    """
    
    def __init__(self, node: Node, topic_name: str = '/cmd/response', qos_profile: int = 10):
        """
        初始化响应发布器 / Initialize response publisher
        
        Args:
            node: ROS2节点实例 / ROS2 node instance
            topic_name: 响应Topic名称 / Response topic name
            qos_profile: QoS配置 / QoS profile
        """
        self.node = node
        self.topic_name = topic_name
        
        # 创建发布器 / Create publisher
        self.publisher = node.create_publisher(
            String,
            topic_name,
            qos_profile=qos_profile
        )
        
        # 统计信息 / Statistics
        self.total_published = 0
        self.published_by_status = {
            'queued': 0,
            'executing': 0,
            'completed': 0,
            'failed': 0,
            'cancelled': 0
        }
        
        node.get_logger().info(
            f"ResponsePublisher initialized on topic '{topic_name}'"
        )
    
    def publish(self, response: CommandResponse, log_level: str = 'info'):
        """
        发布响应 / Publish response
        
        Args:
            response: CommandResponse对象 / CommandResponse object
            log_level: 日志级别 (info/debug/warn) / Log level
        """
        try:
            # 1. 序列化为JSON / Serialize to JSON
            json_str = response.to_json()
            
            # 2. 封装为ROS消息 / Wrap in ROS message
            msg = String()
            msg.data = json_str
            
            # 3. 发布消息 / Publish message
            self.publisher.publish(msg)
            
            # 4. 更新统计信息 / Update statistics
            self.total_published += 1
            if response.status in self.published_by_status:
                self.published_by_status[response.status] += 1
            
            # 5. 记录日志 / Log
            self._log_response(response, log_level)
        
        except Exception as e:
            self.node.get_logger().error(
                f"Failed to publish response for request_id={response.request_id}: {e}"
            )
    
    def publish_error(self, request_id: str, message: str, code: int):
        """
        发布错误响应（便捷方法）/ Publish error response (convenience method)
        
        Args:
            request_id: 请求ID / Request ID
            message: 错误消息 / Error message
            code: 错误码 / Error code
        """
        response = CommandResponse(
            request_id=request_id,
            status='failed',
            message=message,
            code=code
        )
        self.publish(response, log_level='warn')
    
    def publish_success(self, request_id: str, message: str, result: dict = None):
        """
        发布成功响应（便捷方法）/ Publish success response (convenience method)
        
        Args:
            request_id: 请求ID / Request ID
            message: 成功消息 / Success message
            result: 结果数据 / Result data
        """
        response = CommandResponse(
            request_id=request_id,
            status='completed',
            message=message,
            code=0,
            result=result or {}
        )
        self.publish(response)
    
    def get_statistics(self) -> dict:
        """
        获取统计信息 / Get statistics
        
        Returns:
            dict: 统计数据 / Statistics data
        """
        return {
            'total_published': self.total_published,
            'by_status': self.published_by_status.copy(),
            'topic_name': self.topic_name
        }
    
    def reset_statistics(self):
        """重置统计信息 / Reset statistics"""
        self.total_published = 0
        self.published_by_status = {
            'queued': 0,
            'executing': 0,
            'completed': 0,
            'failed': 0,
            'cancelled': 0
        }
    
    def _log_response(self, response: CommandResponse, log_level: str):
        """
        记录响应日志 / Log response
        
        Args:
            response: 响应对象 / Response object
            log_level: 日志级别 / Log level
        """
        # 构建日志消息 / Build log message
        log_msg = (
            f"Published response: request_id={response.request_id}, "
            f"status={response.status}, code={response.code}, "
            f"message='{response.message}'"
        )
        
        # 添加result信息（如果存在）/ Add result info if exists
        if response.result:
            # 只记录关键字段，避免日志过长 / Only log key fields to avoid long logs
            if 'task_id' in response.result:
                log_msg += f", task_id={response.result['task_id']}"
            if 'progress' in response.result:
                log_msg += f", progress={response.result['progress']}"
        
        # 根据级别记录日志 / Log based on level
        logger = self.node.get_logger()
        if log_level == 'debug':
            logger.debug(log_msg)
        elif log_level == 'warn':
            logger.warn(log_msg)
        else:
            logger.info(log_msg)
