#!/bin/bash
###############################################################################
# monitor_memory.sh - 内存和Swap使用监控脚本
# 
# 功能：实时显示内存、swap、zram使用情况
# 使用：bash monitor_memory.sh
###############################################################################

echo "========================================================================"
echo "内存监控（按Ctrl+C退出）"
echo "Memory Monitor (Press Ctrl+C to exit)"
echo "========================================================================"
echo ""

while true; do
    clear
    echo "========================================================================"
    echo "时间: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "========================================================================"
    echo ""
    
    echo "📊 内存使用 / Memory Usage:"
    free -h
    echo ""
    
    echo "💾 Swap详情 / Swap Details:"
    swapon --show
    echo ""
    
    if command -v zramctl &> /dev/null; then
        echo "📦 zram状态 / zram Status:"
        zramctl
        echo ""
    fi
    
    echo "🔥 Top 5进程（内存占用）/ Top 5 Memory Consumers:"
    ps aux --sort=-%mem | head -6
    echo ""
    
    # 计算swap使用率
    SWAP_TOTAL=$(free | grep Swap | awk '{print $2}')
    SWAP_USED=$(free | grep Swap | awk '{print $3}')
    if [ "$SWAP_TOTAL" -gt 0 ]; then
        SWAP_PERCENT=$(awk "BEGIN {printf \"%.1f\", ($SWAP_USED/$SWAP_TOTAL)*100}")
        echo "⚠️  Swap使用率: $SWAP_PERCENT%"
        if (( $(echo "$SWAP_PERCENT > 50" | bc -l) )); then
            echo "   ⚠️  警告：Swap使用率超过50%，建议优化内存使用或升级RAM"
        fi
    fi
    echo ""
    
    echo "更新间隔: 5秒（可用Ctrl+C退出）"
    sleep 5
done
