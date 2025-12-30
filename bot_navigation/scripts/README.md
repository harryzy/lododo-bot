# Bot Navigation Scripts

测试脚本和工具集合。

## 测试脚本

### test_mission_services.py

**MissionPlanner服务接口全面测试脚本**

- **功能**: 测试所有mission服务接口，验证功能正确性
- **覆盖**: 12个服务接口，包括导航、巡航、探索、任务管理等
- **输出**: 彩色终端输出 + JSON格式测试报告
- **位置**: `src/bot_navigation/scripts/test_mission_services.py`

**使用方法**:
```bash
# 方法1: 使用启动脚本（推荐）
cd ~/lododo_bot
./run_mission_tests.sh

# 方法2: 直接运行
python3 src/bot_navigation/scripts/test_mission_services.py
```

**详细文档**: [MISSION_TEST_SCRIPT_USAGE.md](../../docs/MISSION_TEST_SCRIPT_USAGE.md)

### test_mission_planner.py

**交互式测试脚本**（已有）

- 用于手动逐步测试
- 需要xterm终端

## 其他脚本

（待添加）

---

**更多信息**: 
- [MissionPlanner测试指南](../../docs/MISSION_PLANNER_TEST_GUIDE.md)
- [测试脚本使用文档](../../docs/MISSION_TEST_SCRIPT_USAGE.md)
