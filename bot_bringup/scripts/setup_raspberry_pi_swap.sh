#!/bin/bash
###############################################################################
# setup_raspberry_pi_swap.sh - 树莓派内存优化脚本
# 
# 功能：
#   1. 创建2GB永久swap文件（避免OOM崩溃）
#   2. 启用zram压缩内存（1GB物理RAM → 2GB可用）
#   3. 优化swappiness参数（减少SD卡写入）
#   4. 配置开机自动挂载
#
# 适用场景：
#   - 树莓派4B 4GB RAM版本
#   - 运行RTABMap SLAM + Nav2导航
#   - 使用SD卡作为系统盘
#
# 使用方法：
#   sudo bash setup_raspberry_pi_swap.sh
#
# 注意事项：
#   - 需要root权限
#   - 会占用SD卡2GB空间
#   - 建议使用高速SD卡（A1/A2等级）
#
# Author: LeKiwi Bot Development Team
# Date: 2026-01-30
###############################################################################

set -e  # 遇到错误立即退出

echo "========================================================================"
echo "树莓派内存优化配置脚本"
echo "Raspberry Pi Memory Optimization Script"
echo "========================================================================"
echo ""

# 检查是否有root权限
if [ "$EUID" -ne 0 ]; then 
    echo "❌ 错误：需要root权限"
    echo "Error: Please run as root (sudo bash $0)"
    exit 1
fi

# 获取当前内存信息
echo "📊 当前内存状态 / Current Memory Status:"
free -h
echo ""

# 检查是否已有swap
EXISTING_SWAP=$(swapon --show | grep -v "NAME" | wc -l)
if [ "$EXISTING_SWAP" -gt 0 ]; then
    echo "⚠️  检测到已存在的swap分区："
    swapon --show
    echo ""
    read -p "是否继续创建新的swap文件？ (y/n): " CONTINUE
    if [ "$CONTINUE" != "y" ]; then
        echo "操作已取消"
        exit 0
    fi
fi

###############################################################################
# STEP 1: 创建2GB Swap文件
###############################################################################
echo ""
echo "========================================================================"
echo "STEP 1: 创建2GB Swap文件"
echo "========================================================================"

SWAP_FILE="/swapfile"
SWAP_SIZE="2G"  # 2GB，平衡性能和寿命

# 检查是否已存在swap文件
if [ -f "$SWAP_FILE" ]; then
    echo "⚠️  检测到已存在的swap文件: $SWAP_FILE"
    swapoff "$SWAP_FILE" 2>/dev/null || true
    rm -f "$SWAP_FILE"
    echo "✅ 旧swap文件已删除"
fi

# 创建swap文件（使用fallocate更快）
echo "📝 创建${SWAP_SIZE}的swap文件（可能需要1-2分钟）..."
if command -v fallocate &> /dev/null; then
    fallocate -l "$SWAP_SIZE" "$SWAP_FILE"
else
    # 备用方法（较慢）
    dd if=/dev/zero of="$SWAP_FILE" bs=1M count=2048 status=progress
fi

# 设置正确权限（安全）
chmod 600 "$SWAP_FILE"
echo "✅ Swap文件创建完成"

# 格式化为swap
echo "🔧 格式化swap文件..."
mkswap "$SWAP_FILE"

# 启用swap
echo "🚀 启用swap..."
swapon "$SWAP_FILE"
echo "✅ Swap已启用"

# 验证swap
echo ""
echo "📊 新的内存状态："
free -h
swapon --show

###############################################################################
# STEP 2: 配置开机自动挂载
###############################################################################
echo ""
echo "========================================================================"
echo "STEP 2: 配置开机自动挂载"
echo "========================================================================"

# 备份fstab
cp /etc/fstab /etc/fstab.backup.$(date +%Y%m%d_%H%M%S)
echo "✅ 已备份/etc/fstab"

# 检查fstab中是否已有swap配置
if grep -q "$SWAP_FILE" /etc/fstab; then
    echo "⚠️  /etc/fstab中已有swap配置，跳过添加"
else
    echo "$SWAP_FILE none swap sw 0 0" >> /etc/fstab
    echo "✅ 已添加swap到/etc/fstab"
fi

###############################################################################
# STEP 3: 优化swappiness参数
###############################################################################
echo ""
echo "========================================================================"
echo "STEP 3: 优化swappiness参数（减少SD卡写入）"
echo "========================================================================"

# swappiness参数说明：
# - 0: 尽量不使用swap（可能导致OOM）
# - 10: 只在内存严重不足时使用swap（推荐SD卡）
# - 60: 默认值（适合机械硬盘）
# - 100: 激进使用swap（适合SSD）

SWAPPINESS=10
echo "vm.swappiness=$SWAPPINESS" > /etc/sysctl.d/99-swappiness.conf
sysctl vm.swappiness=$SWAPPINESS
echo "✅ 已设置swappiness=$SWAPPINESS（优先使用物理RAM）"

# 优化cache压力（减少cache占用，为应用程序留更多内存）
echo "vm.vfs_cache_pressure=50" >> /etc/sysctl.d/99-swappiness.conf
sysctl vm.vfs_cache_pressure=50
echo "✅ 已优化cache压力参数"

###############################################################################
# STEP 4: 启用zram压缩内存（可选但推荐）
###############################################################################
echo ""
echo "========================================================================"
echo "STEP 4: 启用zram压缩内存（推荐）"
echo "========================================================================"

read -p "是否启用zram压缩内存？（将1GB RAM压缩成2GB，性能好且不损坏SD卡）(y/n): " ENABLE_ZRAM

if [ "$ENABLE_ZRAM" = "y" ]; then
    # 安装zram-tools
    if ! dpkg -l | grep -q zram-tools; then
        echo "📦 安装zram-tools..."
        apt-get update
        apt-get install -y zram-tools
    else
        echo "✅ zram-tools已安装"
    fi
    
    # 配置zram（1GB压缩内存）
    cat > /etc/default/zramswap <<EOF
# zram压缩内存配置
# 分配1GB物理RAM用于压缩（可压缩到2-3倍）
ALGO=lz4          # 压缩算法（lz4速度快）
PERCENT=25        # 使用25%物理RAM（4GB * 0.25 = 1GB）
PRIORITY=100      # 优先级高于文件swap（先用zram，再用文件swap）
EOF
    
    # 重启zram服务
    systemctl restart zramswap
    echo "✅ zram已启用"
    
    echo ""
    echo "📊 zram状态："
    zramctl
else
    echo "⏭️  跳过zram配置"
fi

###############################################################################
# STEP 5: 显示最终配置
###############################################################################
echo ""
echo "========================================================================"
echo "✅ 内存优化配置完成！"
echo "========================================================================"
echo ""
echo "📊 最终内存配置："
free -h
echo ""
echo "💾 Swap配置："
swapon --show
echo ""
echo "🎛️  内核参数："
sysctl vm.swappiness
sysctl vm.vfs_cache_pressure
echo ""

if [ "$ENABLE_ZRAM" = "y" ]; then
    echo "📦 zram状态："
    zramctl
    echo ""
fi

echo "========================================================================"
echo "配置说明 / Configuration Summary"
echo "========================================================================"
echo "1. 文件Swap: 2GB（应急内存，避免OOM崩溃）"
echo "   位置: $SWAP_FILE"
echo "   开机自动挂载: 是"
echo ""
echo "2. swappiness: $SWAPPINESS（优先使用物理RAM，减少SD卡写入）"
echo ""
if [ "$ENABLE_ZRAM" = "y" ]; then
    echo "3. zram: 已启用（~1GB压缩内存，性能高且不损坏SD卡）"
else
    echo "3. zram: 未启用"
fi
echo ""
echo "预期总内存："
echo "  - 物理RAM: 4GB"
if [ "$ENABLE_ZRAM" = "y" ]; then
    echo "  - zram压缩: ~1-2GB（高速）"
fi
echo "  - 文件Swap: 2GB（低速，应急用）"
echo "  - 总计: ~7-8GB可用内存"
echo ""
echo "⚠️  注意事项："
echo "  1. 定期检查SD卡健康状态（sudo smartctl -a /dev/mmcblk0）"
echo "  2. 监控swap使用率（free -h），如经常>50%则物理内存不足"
echo "  3. 建议使用高速SD卡（A1/A2等级）或改用SSD"
echo "  4. 长期运行建议升级到8GB RAM版本"
echo ""
echo "监控命令："
echo "  - 实时内存监控: watch -n 1 free -h"
echo "  - swap使用率: swapon --show"
echo "  - zram状态: zramctl"
echo "========================================================================"
echo ""
echo "✅ 配置已生效，无需重启！"
echo ""
