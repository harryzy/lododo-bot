# Web控制界面 - 故障排除指南

**版本**: v1.0.0  
**更新日期**: 2026-01-08

---

## 连接问题

### ❌ WebSocket连接失败

**症状**: 
- 浏览器控制台报错 `WebSocket connection to 'ws://localhost:8000/ws' failed`
- 右上角显示红色"已断开"

**可能原因与解决方案**:

1. **Web服务器未启动**
   ```bash
   # 检查进程
   ps aux | grep uvicorn
   
   # 启动服务器
   cd ~/lododo_bot/src/bot_teleop
   bash scripts/start_web_server.sh
   ```

2. **端口被占用**
   ```bash
   # 检查端口
   lsof -i :8000
   
   # 杀死占用进程
   kill -9 <PID>
   ```

3. **虚拟环境问题**
   ```bash
   # 重新创建虚拟环境
   rm -rf ~/lododo_bot/venv_web
   bash scripts/setup_web_env.sh
   ```

4. **连接数限制**（超过3个连接）
   - 关闭其他浏览器标签页
   - 修改 `config/web_config.yaml` 中的 `max_connections`

---

### ❌ ROS Bridge连接失败

**症状**: 
- 浏览器控制台报错 `Failed to connect to rosbridge`
- 地图不更新

**解决方案**:

1. **rosbridge未启动**
   ```bash
   # 检查节点
   ros2 node list | grep rosbridge
   
   # 启动rosbridge
   ros2 run rosbridge_server rosbridge_websocket
   ```

2. **端口冲突**
   ```bash
   # 检查9090端口
   lsof -i :9090
   
   # 使用其他端口
   ros2 run rosbridge_server rosbridge_websocket --ros-args \
     -p port:=9091
   
   # 修改前端配置
   # 编辑 web_frontend/src/services/rosbridge.ts
   # 改为: new RosBridgeService('ws://localhost:9091')
   ```

3. **防火墙阻止**
   ```bash
   # 允许端口
   sudo ufw allow 9090/tcp
   ```

---

## 界面显示问题

### ❌ 地图不显示

**症状**: MapCanvas区域空白或灰色

**排查步骤**:

1. **检查/map话题**
   ```bash
   # 查看话题列表
   ros2 topic list | grep /map
   
   # 查看数据
   ros2 topic echo /map --once
   
   # 检查频率
   ros2 topic hz /map
   ```

2. **RTABMap未发布地图**
   ```bash
   # 检查RTABMap节点
   ros2 node list | grep rtabmap
   
   # 重启RTABMap
   ros2 lifecycle set /rtabmap/rtabmap configure
   ros2 lifecycle set /rtabmap/rtabmap activate
   ```

3. **浏览器控制台错误**
   - 按F12打开开发者工具
   - 查看Console标签页
   - 搜索"map"相关错误

---

### ❌ 机器人位姿不更新

**症状**: 地图上机器人标记不移动

**解决方案**:

1. **检查定位话题**
   ```bash
   # 查看位姿话题
   ros2 topic echo /rtabmap/localization_pose --once
   
   # 检查频率
   ros2 topic hz /rtabmap/localization_pose
   ```

2. **RTABMap定位失败**
   ```bash
   # 查看RTABMap状态
   ros2 topic echo /rtabmap/info
   
   # 检查地图是否加载
   ls ~/lododo_bot/maps/<map_name>/rtabmap.db
   ```

3. **rosbridge订阅问题**
   - 在浏览器控制台输入：`rosBridgeService.isConnected()`
   - 应返回 `true`

---

## 任务执行问题

### ❌ 导航任务创建失败

**症状**: 点击"开始导航"后报错

**排查**:

1. **后端API错误**
   ```bash
   # 查看Web服务器日志
   tail -f ~/lododo_bot/log/web_server.log
   ```

2. **MissionPlanner未运行**
   ```bash
   # 检查节点
   ros2 node list | grep mission_planner
   
   # 启动MissionPlanner（如使用）
   ros2 launch bot_bringup simulation_mission_planner_localization.launch.py
   ```

3. **网络请求失败**
   - F12 → Network标签页
   - 查看 `/api/tasks/navigate` 请求状态
   - 如果404：检查后端路由注册

---

### ❌ 探索任务无法启动

**症状**: "开始探索"按钮无响应或报错

**解决方案**:

1. **SLAM模式未启用**
   ```bash
   # 使用SLAM模式启动
   ros2 launch bot_bringup simulation_web_full.launch.py slam:=true
   ```

2. **地图名称无效**
   - 确保输入了地图名称
   - 不要使用特殊字符

3. **exploration服务不可用**
   ```bash
   # 检查服务
   ros2 service list | grep exploration
   ```

---

## 性能问题

### ⚠️ 界面卡顿

**可能原因**:

1. **更新频率过高**
   - 编辑 `config/web_config.yaml`
   - 降低 `update_rates` 下的频率值

2. **浏览器性能**
   - 关闭其他标签页
   - 启用硬件加速
   - 尝试Chrome/Edge（性能优于Firefox）

3. **网络延迟**
   ```bash
   # 检查延迟
   ping localhost  # 本地应<1ms
   ping robot-pi   # 局域网应<10ms
   ```

---

### ⚠️ 地图渲染慢

**优化方案**:

1. **使用地图压缩**（默认已启用）
   - 代码已实现栅格索引压缩
   - 减少传输数据量

2. **降低地图分辨率**
   - 修改RTABMap配置
   - `Grid/CellSize: 0.1` → `0.15`（更粗糙但更快）

3. **Canvas性能优化**
   - 限制重绘频率
   - 使用离屏Canvas缓存

---

## 开发问题

### 🔧 前端修改不生效

**原因**: 未重新构建

**解决**:
```bash
cd ~/lododo_bot/src/bot_teleop
bash scripts/dev_rebuild.sh
```

**开发模式**（实时热更新）:
```bash
# 启动Vite开发服务器
cd web_frontend
npm run dev

# 访问 http://localhost:3000（自动代理到后端8000端口）
```

---

### 🔧 后端修改不生效

**Python代码**:
- 使用 `--symlink-install` 后无需重建
- 直接重启Web服务器

**setup.py修改**:
```bash
colcon build --packages-select bot_teleop --symlink-install
source ~/lododo_bot/install/setup.bash
```

---

### 🔧 依赖冲突

**Python依赖**:
```bash
# 检查虚拟环境
source ~/lododo_bot/venv_web/bin/activate
pip list

# 重新安装
pip install --force-reinstall -r requirements_web.txt
```

**Node.js依赖**:
```bash
cd web_frontend
rm -rf node_modules package-lock.json
npm install
```

---

## 日志分析

### 查看关键日志

**Web服务器日志**:
```bash
tail -f ~/lododo_bot/log/web_server.log | grep ERROR
```

**浏览器控制台**:
- F12 → Console
- 过滤 `Error` 或 `Failed`

**ROS2日志**:
```bash
ros2 topic echo /rosout | grep -i error
```

---

## 常见错误代码

| 错误代码 | 含义 | 解决方案 |
|---------|------|---------|
| 1008 | WebSocket连接数超限 | 关闭其他标签页 |
| 404 | API端点不存在 | 检查后端路由 |
| 500 | 后端内部错误 | 查看服务器日志 |
| ECONNREFUSED | 连接被拒绝 | 检查服务是否启动 |
| ETIMEDOUT | 连接超时 | 检查网络/防火墙 |

---

## 紧急恢复

### 完全重置

```bash
# 停止所有服务
pkill -f uvicorn
pkill -f rosbridge

# 清理构建
cd ~/lododo_bot
rm -rf build/ install/ log/

# 重新构建
colcon build --symlink-install

# 重新安装Web环境
cd src/bot_teleop
bash scripts/setup_web_env.sh

# 重启
ros2 launch bot_bringup simulation_web_full.launch.py
```

---

## 获取帮助

1. **查看设计文档**: [WEB_DESIGN.md](WEB_DESIGN.md)
2. **查看API文档**: [API.md](API.md)
3. **查看架构文档**: [bot_cmd_interface/docs/ARCHITECTURE.md](../../bot_cmd_interface/docs/ARCHITECTURE.md)

---

**如问题仍未解决，请收集以下信息并提交Issue**:
- 完整错误日志（浏览器Console + 服务器日志）
- ROS2版本和系统信息
- 复现步骤
