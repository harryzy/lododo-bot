cd ~/workDisk/lododo_bot
source install/setup.bash

# 测试1号轮（舵机7）
python3 src/bot_hardware/scripts/test_servo_control.py --wheel-id 1 --speed 10 --duration 5

# 测试2号轮（舵机8）
python3 src/bot_hardware/scripts/test_servo_control.py --wheel-id 2 --speed 10 --duration 5

# 测试3号轮（舵机9）
python3 src/bot_hardware/scripts/test_servo_control.py --wheel-id 3 --speed 10 --duration 5

# 或一次性测试所有轮子
python3 src/bot_hardware/scripts/test_servo_control.py --test-all --speed 10 --duration 3