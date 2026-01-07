"""
CommandAdapter主节点 / CommandAdapter main node

统一命令接口的核心节点，负责接收请求、队列管理和响应发布
Core node of unified command interface, responsible for request reception, queue management, and response publishing
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import threading
import json
import asyncio
import time

from .components.request_queue import RequestQueue
from .components.response_publisher import ResponsePublisher
from .components.service_adapter import ServiceAdapter
from .sdk.message import CommandRequest, CommandResponse, ErrorCode, ResponseStatus
from .utils.config_loader import load_command_config


class CommandAdapter(Node):
    """
    命令适配器主节点 / Command adapter main node
    
    职责 / Responsibilities:
    1. 订阅/cmd/request，接收请求 / Subscribe to /cmd/request to receive requests
    2. 验证请求，入队 / Validate and enqueue requests
    3. 后台线程处理队列 / Background thread processes queue
    4. 使用ServiceAdapter调用实际的ROS2服务 / Uses ServiceAdapter to call actual ROS2 services
    5. 发布响应到/cmd/response / Publish responses to /cmd/response
    
    架构设计 / Architecture:
    - 使用RequestQueue管理请求队列 / Uses RequestQueue for request queue management
    - 使用ResponsePublisher发布响应 / Uses ResponsePublisher for response publishing
    - 使用ServiceAdapter调用ROS2服务 / Uses ServiceAdapter to call ROS2 services
    - 后台线程循环处理请求 / Background thread loops to process requests
    """
    
    def __init__(self):
        super().__init__('command_adapter')
        
        # 声明参数 / Declare parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('use_mock', False)  # 是否使用mock模式 / Whether to use mock mode
        self.declare_parameter('service_timeout', 10.0)  # 服务调用超时 / Service call timeout
        
        # 加载配置 / Load configuration
        self.config = self._load_config()
        
        # 初始化组件 / Initialize components
        queue_config = self.config.get('queue', {})
        self.queue = RequestQueue(
            max_size=queue_config.get('max_size', 100),
            dedup_window=queue_config.get('dedup_window', 5.0)
        )
        
        self.response_publisher = ResponsePublisher(
            self,
            topic_name='/cmd/response',
            qos_profile=10
        )
        
        # 初始化ServiceAdapter / Initialize ServiceAdapter
        self.use_mock = self.get_parameter('use_mock').value
        service_timeout = self.get_parameter('service_timeout').value
        
        if not self.use_mock:
            self.service_adapter = ServiceAdapter(self, timeout=service_timeout)
            # 等待服务可用 / Wait for services
            self.get_logger().info('Waiting for navigation services...')
            services_ready = self.service_adapter.wait_for_services(timeout=5.0)
            if not services_ready:
                self.get_logger().warn(
                    'Some services are not ready. Commands may fail until services become available.'
                )
        else:
            self.service_adapter = None
            self.get_logger().info('Running in MOCK mode (for testing without navigation services)')
        
        # 创建订阅器 / Create subscriber
        self.subscription = self.create_subscription(
            String,
            '/cmd/request',
            self._request_callback,
            qos_profile=10
        )
        
        # 处理线程控制 / Processing thread control
        self.running = True
        self.processing_thread = threading.Thread(
            target=self._processing_loop,
            daemon=True
        )
        self.processing_thread.start()
        
        # 统计信息 / Statistics
        self.total_requests_received = 0
        self.total_requests_processed = 0
        
        mode_str = 'MOCK' if self.use_mock else 'SERVICE'
        self.get_logger().info('='*60)
        self.get_logger().info('CommandAdapter initialized successfully')
        self.get_logger().info(f'  - Mode: {mode_str}')
        self.get_logger().info(f'  - Listening on: /cmd/request')
        self.get_logger().info(f'  - Publishing to: /cmd/response')
        self.get_logger().info(f'  - Queue max size: {queue_config.get("max_size", 100)}')
        self.get_logger().info(f'  - Dedup window: {queue_config.get("dedup_window", 5.0)}s')
        if not self.use_mock:
            self.get_logger().info(f'  - Service timeout: {service_timeout}s')
        self.get_logger().info('='*60)
    
    def destroy_node(self):
        """清理资源 / Cleanup resources"""
        self.get_logger().info('Shutting down CommandAdapter...')
        self.running = False
        if self.processing_thread.is_alive():
            self.processing_thread.join(timeout=2.0)
        super().destroy_node()
    
    def _load_config(self) -> dict:
        """
        加载配置文件 / Load configuration file
        
        Returns:
            dict: 配置字典 / Configuration dictionary
        """
        config_file = self.get_parameter('config_file').value
        
        if config_file:
            try:
                config = load_command_config(config_file)
                self.get_logger().info(f'Loaded config from: {config_file}')
                return config
            except Exception as e:
                self.get_logger().warn(
                    f'Failed to load config file {config_file}: {e}, using defaults'
                )
        
        # 返回默认配置 / Return default configuration
        return {
            'queue': {
                'max_size': 100,
                'dedup_window': 5.0
            },
            'timeout': {
                'default': 300.0
            }
        }
    
    def _request_callback(self, msg: String):
        """
        请求回调 / Request callback
        
        接收/cmd/request消息，解析、验证并入队
        Receives /cmd/request messages, parses, validates, and enqueues
        
        Args:
            msg: ROS2 String消息 / ROS2 String message
        """
        self.total_requests_received += 1
        
        try:
            # 1. 解析JSON / Parse JSON
            try:
                request = CommandRequest.from_json(msg.data)
            except json.JSONDecodeError as e:
                self.get_logger().error(f'Failed to parse request JSON: {e}')
                self._publish_error_response(
                    'unknown',
                    f'Invalid JSON format: {str(e)}',
                    ErrorCode.BAD_REQUEST
                )
                return
            except Exception as e:
                self.get_logger().error(f'Failed to create CommandRequest: {e}')
                self._publish_error_response(
                    'unknown',
                    f'Invalid request format: {str(e)}',
                    ErrorCode.BAD_REQUEST
                )
                return
            
            # 2. 验证请求 / Validate request
            valid, error_msg = request.validate()
            if not valid:
                self.get_logger().warn(
                    f'Invalid request from request_id={request.request_id}: {error_msg}'
                )
                self._publish_error_response(
                    request.request_id,
                    error_msg,
                    ErrorCode.BAD_REQUEST
                )
                return
            
            # 3. 入队 / Enqueue
            success, message = self.queue.enqueue(request)
            
            if success:
                # 发布queued响应 / Publish queued response
                self.get_logger().info(
                    f'Request queued: request_id={request.request_id}, '
                    f'action={request.action}, queue_size={self.queue.size()}'
                )
                
                response = CommandResponse(
                    request_id=request.request_id,
                    status=ResponseStatus.QUEUED,
                    message='Request queued successfully',
                    code=ErrorCode.SUCCESS
                )
                self.response_publisher.publish(response, log_level='debug')
            else:
                # 入队失败（重复或队列满）/ Enqueue failed
                self.get_logger().warn(
                    f'Failed to enqueue request_id={request.request_id}: {message}'
                )
                self._publish_error_response(
                    request.request_id,
                    message,
                    ErrorCode.CONFLICT
                )
        
        except Exception as e:
            self.get_logger().error(f'Request callback error: {e}')
            self._publish_error_response(
                'unknown',
                f'Internal error: {str(e)}',
                ErrorCode.INTERNAL_ERROR
            )
    
    def _processing_loop(self):
        """
        处理循环 / Processing loop
        
        后台线程循环从队列取请求并处理
        Background thread loops to dequeue and process requests
        """
        self.get_logger().info('Processing loop started')
        
        while self.running and rclpy.ok():
            try:
                # 从队列取请求 / Dequeue request
                request = self.queue.dequeue(timeout=1.0)
                
                if request is None:
                    continue
                
                # 处理请求 / Process request
                self._process_request(request)
                
            except Exception as e:
                self.get_logger().error(f'Processing loop error: {e}')
                time.sleep(0.5)  # 避免错误循环过快
        
        self.get_logger().info('Processing loop stopped')
    
    def _process_request(self, request: CommandRequest):
        """
        处理单个请求 / Process single request
        
        Args:
            request: 命令请求对象 / Command request object
        """
        self.total_requests_processed += 1
        
        try:
            # 1. 发布executing响应 / Publish executing response
            self.get_logger().info(
                f'Processing request: request_id={request.request_id}, action={request.action}'
            )
            
            executing_response = CommandResponse(
                request_id=request.request_id,
                status=ResponseStatus.EXECUTING,
                message=f'Processing {request.action}',
                code=ErrorCode.SUCCESS
            )
            self.response_publisher.publish(executing_response, log_level='debug')
            
            # 2. 调用处理逻辑 / Call processing logic
            if self.use_mock or self.service_adapter is None:
                # Mock模式 / Mock mode
                result = self._mock_process_request(request)
            else:
                # 使用ServiceAdapter处理 / Use ServiceAdapter
                result = self._service_process_request(request)
            
            # 3. 发布完成响应 / Publish completion response
            final_status = ResponseStatus.COMPLETED if result['success'] else ResponseStatus.FAILED
            final_response = CommandResponse(
                request_id=request.request_id,
                status=final_status,
                message=result['message'],
                code=result['code'],
                result=result.get('result', {})
            )
            self.response_publisher.publish(final_response)
            
            self.get_logger().info(
                f'Request completed: request_id={request.request_id}, '
                f'status={final_status}, code={result["code"]}'
            )
        
        except Exception as e:
            self.get_logger().error(
                f'Failed to process request_id={request.request_id}: {e}'
            )
            self._publish_error_response(
                request.request_id,
                f'Internal processing error: {str(e)}',
                ErrorCode.INTERNAL_ERROR
            )
    
    def _service_process_request(self, request: CommandRequest) -> dict:
        """
        使用ServiceAdapter处理请求 / Process request using ServiceAdapter
        
        Args:
            request: 命令请求对象 / Command request object
        
        Returns:
            dict: 处理结果 / Processing result
        """
        try:
            # 创建事件循环来运行async方法 / Create event loop to run async method
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            try:
                success, result_data, error_msg = loop.run_until_complete(
                    self.service_adapter.process_request(request)
                )
            finally:
                loop.close()
            
            if success:
                return {
                    'success': True,
                    'message': error_msg if error_msg else 'Request processed successfully',
                    'code': ErrorCode.SUCCESS,
                    'result': result_data
                }
            else:
                # 判断错误类型 / Determine error type
                error_code = ErrorCode.INTERNAL_ERROR
                if 'timeout' in error_msg.lower():
                    error_code = ErrorCode.GATEWAY_TIMEOUT
                elif 'not ready' in error_msg.lower() or 'not available' in error_msg.lower():
                    error_code = ErrorCode.SERVICE_UNAVAILABLE
                
                return {
                    'success': False,
                    'message': error_msg,
                    'code': error_code,
                    'result': {}
                }
        
        except Exception as e:
            self.get_logger().error(f'Service processing failed: {e}')
            return {
                'success': False,
                'message': f'Service processing error: {str(e)}',
                'code': ErrorCode.INTERNAL_ERROR,
                'result': {}
            }
    
    def _mock_process_request(self, request: CommandRequest) -> dict:
        """
        模拟请求处理 / Mock request processing
        
        用于测试或在没有实际服务时运行 /
        Used for testing or running without actual services
        
        Args:
            request: 命令请求对象 / Command request object
        
        Returns:
            dict: 处理结果 / Processing result
        """
        # 模拟处理延迟 / Simulate processing delay
        time.sleep(0.5)
        
        # 根据action返回不同的模拟结果 / Return different mock results based on action
        from .sdk.action_types import ActionType
        
        if request.action == ActionType.GET_ROBOT_STATUS:
            # 查询状态类请求立即成功 / Status query requests succeed immediately
            return {
                'success': True,
                'message': 'Robot status retrieved (mock)',
                'code': ErrorCode.SUCCESS,
                'result': {
                    'battery_level': 85.5,
                    'current_task': None,
                    'status': 'idle'
                }
            }
        
        elif request.action == ActionType.EMERGENCY_STOP:
            # 紧急停止立即成功 / Emergency stop succeeds immediately
            return {
                'success': True,
                'message': 'Emergency stop executed (mock)',
                'code': ErrorCode.SUCCESS,
                'result': {
                    'cancelled_task_count': 0
                }
            }
        
        elif request.action in [ActionType.NAVIGATE_TO_POSE, ActionType.START_PATROL, 
                                 ActionType.START_EXPLORATION]:
            # 任务类请求返回task_id / Task requests return task_id
            mock_task_id = f'mock-task-{request.request_id[:8]}'
            return {
                'success': True,
                'message': f'Task started (mock): {request.action}',
                'code': ErrorCode.SUCCESS,
                'result': {
                    'task_id': mock_task_id
                }
            }
        
        else:
            # 其他请求默认成功 / Other requests succeed by default
            return {
                'success': True,
                'message': f'Request processed (mock): {request.action}',
                'code': ErrorCode.SUCCESS,
                'result': {}
            }
    
    def _publish_error_response(self, request_id: str, message: str, code: int):
        """
        发布错误响应 / Publish error response
        
        Args:
            request_id: 请求ID / Request ID
            message: 错误消息 / Error message
            code: 错误码 / Error code
        """
        response = CommandResponse(
            request_id=request_id,
            status=ResponseStatus.FAILED,
            message=message,
            code=code
        )
        self.response_publisher.publish(response, log_level='warn')
    
    def get_statistics(self) -> dict:
        """
        获取统计信息 / Get statistics
        
        Returns:
            dict: 统计数据 / Statistics data
        """
        return {
            'total_requests_received': self.total_requests_received,
            'total_requests_processed': self.total_requests_processed,
            'queue': self.queue.get_statistics(),
            'publisher': self.response_publisher.get_statistics()
        }


def main(args=None):
    """主函数 / Main function"""
    rclpy.init(args=args)
    
    try:
        node = CommandAdapter()
        
        # 使用MultiThreadedExecutor支持回调和处理线程并发
        # Use MultiThreadedExecutor to support concurrent callbacks and processing thread
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard interrupt, shutting down...')
        finally:
            node.destroy_node()
            executor.shutdown()
    
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
