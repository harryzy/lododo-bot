# Web控制界面快速启动指南

**目标**: 5分钟内启动Web控制界面  
**前提**: 已完成ROS2 Humble环境配置

---

## 📦 一键环境搭建

```bash
# 进入项目目录
cd ~/lododo_bot/src/bot_teleop

# 执行环境搭建脚本
bash scripts/setup_web_env.sh
```

**脚本会自动**:
1. 创建Python虚拟环境 (`~/lododo_bot/venv_web`)
2. 安装Python依赖
3. 安装Node.js依赖
4. 构建前端 (`npm run build`)
5. 构建ROS2包 (`colcon build`)

**预计耗时**: 3-5分钟

---

## 🚀 启动Web控制界面

### 方式1: 完整仿真环境（推荐）

```bash
cd ~/lododo_bot
source install/setup.bash

# 启动完整环境（Gazebo + Nav2 + CommandAdapter + rosbridge + Web服务器）
ros2 launch bot_bringup simulation_web_full.launch.py \
  slam:=false \
  map_name:=exploration_test
```

**包含组件**:
- ✅ Gazebo仿真环境
- ✅ Nav2导航栈
- ✅ RTABMap定位
- ✅ CommandAdapter
- ✅ rosbridge_server (端口9090)
- ✅ FastAPI Web服务器 (端口8000)

---

### 方式2: 仅启动Web服务器（已有运行的机器人）

```bash
cd ~/lododo_bot
source install/setup.bash

# 启动Web服务器
ros2 launch bot_teleop web_server.launch.py
```

**适用场景**: 机器人已在运行，只需启动Web界面

---

## 🌐 访问Web界面

打开浏览器，访问:
```
http://localhost:8000
```

**推荐浏览器**: Chrome、Edge、Firefox  
**最小分辨率**: 1280x720

---

## ✅ 验证功能

### 1. 检查连接状态
查看底部状态栏:
- ROS: ● 绿色 (已连接)
- WebSocket: ● 绿色 (已连接)
- 延迟: < 100ms

### 2. 测试地图显示
- 地图区域应显示灰色地图
- 机器人图标（蓝色三角形）应可见
- 拖拽地图可平移，滚轮可缩放

### 3. 测试导航功能
1. 点击工具栏的🎯按钮
2. 在地图上点击并拖拽设置目标
3. 释放鼠标，机器人开始移动
4. 右侧面板显示任务状态

---

## 🛠️ 开发模式（快速重建）

### 修改前端代码后

```bash
cd ~/lododo_bot/src/bot_teleop

# 快速重建（npm build + colcon build）
bash scripts/dev_rebuild.sh
```

### 修改后端代码后

```bash
cd ~/lododo_bot

# 重新编译
colcon build --packages-select bot_teleop --symlink-install

# 重启服务器（Ctrl+C停止launch，然后重新启动）
ros2 launch bot_bringup simulation_web_full.launch.py slam:=false map_name:=exploration_test
```

---

## 📝 常见问题

### 问题1: 地图不显示
**检查**:
- rosbridge是否启动 (`ros2 node list | grep rosbridge`)
- 浏览器控制台是否有错误 (F12)

**解决**:
```bash
# 手动启动rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

### 问题2: WebSocket连接失败
**检查**:
- FastAPI是否启动 (`curl http://localhost:8000/api/status`)
- 防火墙是否阻止端口8000

**解决**:
```bash
# 检查端口占用
netstat -tulnp | grep 8000

# 杀死占用进程
sudo kill -9 <PID>
```

---

### 问题3: 前端修改不生效
**原因**: 前端代码需要重新构建

**解决**:
```bash
cd ~/lododo_bot/src/bot_teleop/web_frontend
npm run build
```

---

## 🎯 下一步

- 查看 [USER_GUIDE.md](USER_GUIDE.md) 学习完整功能
- 查看 [TROUBLESHOOTING.md](TROUBLESHOOTING.md) 解决其他问题
- 查看 [DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md) 部署到真实机器人

---

**快速启动完成！开始使用Web控制界面吧！** 🎉
