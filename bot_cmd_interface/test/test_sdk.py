"""
SDK模块单元测试 / SDK module unit tests

测试CommandRequest, CommandResponse, ActionType, validators等核心类 /
Tests CommandRequest, CommandResponse, ActionType, validators and other core classes
"""

import pytest
import json
from bot_cmd_interface.sdk import (
    CommandRequest,
    CommandResponse,
    ActionType,
    ResponseStatus,
    ErrorCode,
    create_navigate_request,
    create_patrol_request,
    create_exploration_request,
    create_emergency_stop_request,
    create_get_status_request,
    create_cancel_task_request,
    validate_request,
    validate_response,
)


class TestActionType:
    """测试ActionType类 / Test ActionType class"""
    
    def test_all_actions_defined(self):
        """测试所有动作常量已定义 / Test all action constants defined"""
        assert len(ActionType.ALL_ACTIONS) == 12  # 移除NAVIGATE_TO_LOCATION后为12个
        assert ActionType.NAVIGATE_TO_POSE in ActionType.ALL_ACTIONS
        assert ActionType.EMERGENCY_STOP in ActionType.ALL_ACTIONS
    
    def test_is_valid_action(self):
        """测试动作验证 / Test action validation"""
        assert ActionType.is_valid_action(ActionType.NAVIGATE_TO_POSE)
        assert ActionType.is_valid_action("start_exploration")
        assert not ActionType.is_valid_action("invalid_action")
    
    def test_get_category(self):
        """测试动作分类 / Test action category"""
        assert ActionType.get_category(ActionType.NAVIGATE_TO_POSE) == "navigation"
        assert ActionType.get_category(ActionType.START_PATROL) == "task"
        assert ActionType.get_category(ActionType.GET_TASK_STATUS) == "query"
        assert ActionType.get_category(ActionType.EMERGENCY_STOP) == "control"
        assert ActionType.get_category(ActionType.SAVE_MAP) == "map"


class TestErrorCode:
    """测试ErrorCode类 / Test ErrorCode class"""
    
    def test_error_codes_defined(self):
        """测试错误码定义 / Test error codes defined"""
        assert ErrorCode.SUCCESS == 0
        assert ErrorCode.BAD_REQUEST == 400
        assert ErrorCode.NOT_FOUND == 404
        assert ErrorCode.SERVICE_UNAVAILABLE == 503
    
    def test_get_message(self):
        """测试错误信息获取 / Test get error message"""
        assert "Success" in ErrorCode.get_message(0)
        assert "Bad request" in ErrorCode.get_message(400)
        assert "not found" in ErrorCode.get_message(404)
        assert "Unknown" in ErrorCode.get_message(9999)
    
    def test_is_retryable(self):
        """测试错误可重试判断 / Test error retryable check"""
        assert ErrorCode.is_retryable(503)
        assert ErrorCode.is_retryable(504)
        assert not ErrorCode.is_retryable(400)
        assert not ErrorCode.is_retryable(404)


class TestCommandRequest:
    """测试CommandRequest类 / Test CommandRequest class"""
    
    def test_create_request(self):
        """测试创建请求 / Test create request"""
        request = CommandRequest(
            action=ActionType.NAVIGATE_TO_POSE,
            params={"goal_pose": {"position": {"x": 1.0, "y": 2.0}}}
        )
        
        assert request.action == ActionType.NAVIGATE_TO_POSE
        assert request.request_id is not None
        assert request.priority == 3
        assert request.timeout == 300.0
    
    def test_to_json(self):
        """测试JSON序列化 / Test JSON serialization"""
        request = CommandRequest(
            action=ActionType.NAVIGATE_TO_POSE,
            params={"x": 1.0},
            request_id="test-001"
        )
        
        json_str = request.to_json()
        data = json.loads(json_str)
        
        assert data['header']['request_id'] == "test-001"
        assert data['body']['action'] == ActionType.NAVIGATE_TO_POSE
        assert data['body']['params']['x'] == 1.0
    
    def test_from_json(self):
        """测试JSON反序列化 / Test JSON deserialization"""
        json_str = json.dumps({
            "header": {
                "request_id": "test-002",
                "timestamp": "2026-01-07T10:00:00",
                "priority": 2
            },
            "body": {
                "action": "navigate_to_pose",
                "params": {"x": 1.5},
                "timeout": 200.0
            }
        })
        
        request = CommandRequest.from_json(json_str)
        
        assert request.request_id == "test-002"
        assert request.action == "navigate_to_pose"
        assert request.params['x'] == 1.5
        assert request.priority == 2
        assert request.timeout == 200.0
    
    def test_json_round_trip(self):
        """测试JSON往返转换 / Test JSON round trip"""
        original = CommandRequest(
            action=ActionType.START_EXPLORATION,
            params={"map_name": "test_map"},
            priority=1
        )
        
        json_str = original.to_json()
        restored = CommandRequest.from_json(json_str)
        
        assert restored.action == original.action
        assert restored.params == original.params
        assert restored.priority == original.priority
    
    def test_validate_success(self):
        """测试验证成功 / Test validation success"""
        request = CommandRequest(
            action=ActionType.NAVIGATE_TO_POSE,
            params={"goal_pose": {"position": {"x": 1.0, "y": 2.0}}}
        )
        
        valid, msg = request.validate()
        assert valid
        assert msg == "OK"
    
    def test_validate_missing_action(self):
        """测试缺少action / Test missing action"""
        request = CommandRequest(
            action="",
            params={}
        )
        
        valid, msg = request.validate()
        assert not valid
        assert "action" in msg.lower()
    
    def test_validate_invalid_action(self):
        """测试非法action / Test invalid action"""
        request = CommandRequest(
            action="invalid_action",
            params={}
        )
        
        valid, msg = request.validate()
        assert not valid
        assert "Invalid action" in msg
    
    def test_validate_invalid_priority(self):
        """测试非法优先级 / Test invalid priority"""
        request = CommandRequest(
            action=ActionType.NAVIGATE_TO_POSE,
            params={},
            priority=10
        )
        
        valid, msg = request.validate()
        assert not valid
        assert "Priority" in msg


class TestCommandResponse:
    """测试CommandResponse类 / Test CommandResponse class"""
    
    def test_create_response(self):
        """测试创建响应 / Test create response"""
        response = CommandResponse(
            request_id="test-001",
            status=ResponseStatus.COMPLETED,
            message="Success",
            code=ErrorCode.SUCCESS
        )
        
        assert response.request_id == "test-001"
        assert response.status == ResponseStatus.COMPLETED
        assert response.code == ErrorCode.SUCCESS
    
    def test_to_json(self):
        """测试JSON序列化 / Test JSON serialization"""
        response = CommandResponse(
            request_id="test-002",
            status=ResponseStatus.COMPLETED,
            message="Navigation completed",
            code=0,
            result={"task_id": "task-001"}
        )
        
        json_str = response.to_json()
        data = json.loads(json_str)
        
        assert data['header']['request_id'] == "test-002"
        assert data['header']['status'] == ResponseStatus.COMPLETED
        assert data['body']['message'] == "Navigation completed"
        assert data['body']['result']['task_id'] == "task-001"
    
    def test_from_json(self):
        """测试JSON反序列化 / Test JSON deserialization"""
        json_str = json.dumps({
            "header": {
                "request_id": "test-003",
                "timestamp": "2026-01-07T10:00:00",
                "status": "failed"
            },
            "body": {
                "message": "Service unavailable",
                "code": 503,
                "warnings": ["Connection timeout"]
            }
        })
        
        response = CommandResponse.from_json(json_str)
        
        assert response.request_id == "test-003"
        assert response.status == "failed"
        assert response.code == 503
        assert len(response.warnings) == 1
    
    def test_json_round_trip(self):
        """测试JSON往返转换 / Test JSON round trip"""
        original = CommandResponse(
            request_id="test-004",
            status=ResponseStatus.EXECUTING,
            message="Processing",
            code=0,
            progress=0.5
        )
        
        json_str = original.to_json()
        restored = CommandResponse.from_json(json_str)
        
        assert restored.request_id == original.request_id
        assert restored.status == original.status
        assert restored.progress == original.progress
    
    def test_is_success(self):
        """测试成功判断 / Test success check"""
        success_response = CommandResponse(
            request_id="test-005",
            status=ResponseStatus.COMPLETED,
            message="Success",
            code=ErrorCode.SUCCESS
        )
        assert success_response.is_success()
        
        failed_response = CommandResponse(
            request_id="test-006",
            status=ResponseStatus.FAILED,
            message="Failed",
            code=500
        )
        assert not failed_response.is_success()
    
    def test_is_final(self):
        """测试最终响应判断 / Test final response check"""
        assert CommandResponse("r1", ResponseStatus.COMPLETED, "OK").is_final()
        assert CommandResponse("r2", ResponseStatus.FAILED, "Error").is_final()
        assert CommandResponse("r3", ResponseStatus.CANCELLED, "Cancelled").is_final()
        
        assert not CommandResponse("r4", ResponseStatus.QUEUED, "Queued").is_final()
        assert not CommandResponse("r5", ResponseStatus.EXECUTING, "Running").is_final()


class TestConvenienceConstructors:
    """测试便捷构造函数 / Test convenience constructors"""
    
    def test_create_navigate_request(self):
        """测试导航请求构造 / Test navigate request constructor"""
        request = create_navigate_request(1.5, 2.0, yaw=0.785)
        
        assert request.action == ActionType.NAVIGATE_TO_POSE
        assert request.params['goal_pose']['position']['x'] == 1.5
        assert request.params['goal_pose']['position']['y'] == 2.0
        assert request.params['goal_pose']['orientation']['yaw'] == 0.785
    
    def test_create_navigate_request_no_yaw(self):
        """测试无朝向的导航请求 / Test navigate request without yaw"""
        request = create_navigate_request(1.0, 2.0)
        
        assert 'orientation' not in request.params['goal_pose']
    
    def test_create_patrol_request(self):
        """测试巡航请求构造 / Test patrol request constructor"""
        request = create_patrol_request("route1.yaml", mode="loop")
        
        assert request.action == ActionType.START_PATROL
        assert request.params['waypoint_file'] == "route1.yaml"
        assert request.params['mode'] == "loop"
        assert request.timeout == 3600.0
    
    def test_create_exploration_request(self):
        """测试探索请求构造 / Test exploration request constructor"""
        request = create_exploration_request(map_name="office")
        
        assert request.action == ActionType.START_EXPLORATION
        assert request.params['map_name'] == "office"
        assert request.params['save_on_completion'] is True
    
    def test_create_emergency_stop_request(self):
        """测试紧急停止请求 / Test emergency stop request"""
        request = create_emergency_stop_request(force_immediate=True)
        
        assert request.action == ActionType.EMERGENCY_STOP
        assert request.params['force_immediate'] is True
        assert request.priority == 1
        assert request.timeout == 5.0
    
    def test_create_get_status_request_with_task_id(self):
        """测试状态查询请求（task_id）/ Test status query with task_id"""
        request = create_get_status_request(task_id="task-001")
        
        assert request.action == ActionType.GET_TASK_STATUS
        assert request.params['task_id'] == "task-001"
    
    def test_create_get_status_request_with_request_id(self):
        """测试状态查询请求（request_id）/ Test status query with request_id"""
        request = create_get_status_request(request_id="req-uuid")
        
        assert request.action == ActionType.GET_TASK_STATUS
        assert request.params['request_id'] == "req-uuid"
    
    def test_create_get_status_request_no_params(self):
        """测试状态查询请求缺少参数 / Test status query without params"""
        with pytest.raises(ValueError):
            create_get_status_request()
    
    def test_create_cancel_task_request(self):
        """测试取消任务请求 / Test cancel task request"""
        request = create_cancel_task_request(task_id="task-002")
        
        assert request.action == ActionType.CANCEL_TASK
        assert request.params['task_id'] == "task-002"
        assert request.priority == 2


class TestValidators:
    """测试验证器 / Test validators"""
    
    def test_validate_request_success(self):
        """测试请求验证成功 / Test request validation success"""
        request = create_navigate_request(1.0, 2.0)
        valid, msg = validate_request(request)
        
        assert valid
        assert msg == "OK"
    
    def test_validate_request_missing_param(self):
        """测试缺少必需参数 / Test missing required parameter"""
        request = CommandRequest(
            action=ActionType.NAVIGATE_TO_POSE,
            params={}
        )
        
        valid, msg = validate_request(request)
        assert not valid
        assert "goal_pose" in msg
    
    def test_validate_request_invalid_navigate_params(self):
        """测试非法导航参数 / Test invalid navigation parameters"""
        request = CommandRequest(
            action=ActionType.NAVIGATE_TO_POSE,
            params={"goal_pose": {"position": {"x": "invalid"}}}
        )
        
        valid, msg = validate_request(request)
        assert not valid
    
    def test_validate_request_patrol_mode(self):
        """测试巡航模式验证 / Test patrol mode validation"""
        request = CommandRequest(
            action=ActionType.START_PATROL,
            params={"waypoint_file": "route.yaml", "mode": "invalid_mode"}
        )
        
        valid, msg = validate_request(request)
        assert not valid
        assert "mode" in msg
    
    def test_validate_response_success(self):
        """测试响应验证成功 / Test response validation success"""
        response = CommandResponse(
            request_id="test-001",
            status=ResponseStatus.COMPLETED,
            message="Success",
            code=0
        )
        
        valid, msg = validate_response(response)
        assert valid
    
    def test_validate_response_invalid_status(self):
        """测试非法响应状态 / Test invalid response status"""
        response = CommandResponse(
            request_id="test-002",
            status="invalid_status",
            message="Test",
            code=0
        )
        
        valid, msg = validate_response(response)
        assert not valid
        assert "status" in msg.lower()
    
    def test_validate_response_invalid_progress(self):
        """测试非法进度值 / Test invalid progress value"""
        response = CommandResponse(
            request_id="test-003",
            status=ResponseStatus.EXECUTING,
            message="Running",
            code=0,
            progress=1.5  # 超出范围 / Out of range
        )
        
        valid, msg = validate_response(response)
        assert not valid
        assert "progress" in msg.lower()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
