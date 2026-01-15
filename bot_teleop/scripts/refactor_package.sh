#!/bin/bash
# bot_teleop Package Refactoring Script
# Purpose: 
#   1. Replace all "lododo/Lododo" to "lododo/Lododo"  
#   2. Convert Chinese log messages to English

set -e

BOT_TELEOP_DIR="/home/hurry/workDisk/lododo_bot/src/bot_teleop"

echo "========================================"
echo "   bot_teleop Package Refactoring"
echo "========================================"
echo ""
echo "Target: $BOT_TELEOP_DIR"
echo ""

# Phase 1: Replace lododo/Lododo to lododo/Lododo
echo "Phase 1: Replacing lododo → lododo..."
echo "--------------------------------------"

# Case-insensitive replacement in Python files
find "$BOT_TELEOP_DIR" -name "*.py" -type f -exec sed -i 's/lododo/lododo/g' {} +
find "$BOT_TELEOP_DIR" -name "*.py" -type f -exec sed -i 's/Lododo/Lododo/g' {} +
find "$BOT_TELEOP_DIR" -name "*.py" -type f -exec sed -i 's/LEKIWI/LODODO/g' {} +

# Shell scripts
find "$BOT_TELEOP_DIR" -name "*.sh" -type f -exec sed -i 's/lododo/lododo/g' {} +
find "$BOT_TELEOP_DIR" -name "*.sh" -type f -exec sed -i 's/Lododo/Lododo/g' {} +

# XML files (package.xml)
find "$BOT_TELEOP_DIR" -name "*.xml" -type f -exec sed -i 's/lododo/lododo/g' {} +
find "$BOT_TELEOP_DIR" -name "*.xml" -type f -exec sed -i 's/Lododo/Lododo/g' {} +

# Markdown files
find "$BOT_TELEOP_DIR" -name "*.md" -type f -exec sed -i 's/lododo/lododo/g' {} +
find "$BOT_TELEOP_DIR" -name "*.md" -type f -exec sed -i 's/Lododo/Lododo/g' {} +

# HTML files
find "$BOT_TELEOP_DIR/web_frontend" -name "*.html" -type f -exec sed -i 's/lododo/lododo/g' {} +
find "$BOT_TELEOP_DIR/web_frontend" -name "*.html" -type f -exec sed -i 's/Lododo/Lododo/g' {} +

# TypeScript/JavaScript files
find "$BOT_TELEOP_DIR/web_frontend" -name "*.ts" -o -name "*.tsx" -o -name "*.js" -type f -exec sed -i 's/lododo/lododo/g' {} +
find "$BOT_TELEOP_DIR/web_frontend" -name "*.ts" -o -name "*.tsx" -o -name "*.js" -type f -exec sed -i 's/Lododo/Lododo/g' {} +

# JSON files (package.json, package-lock.json)
find "$BOT_TELEOP_DIR/web_frontend" -name "*.json" -type f -exec sed -i 's/lododo/lododo/g' {} +
find "$BOT_TELEOP_DIR/web_frontend" -name "*.json" -type f -exec sed -i 's/Lododo/Lododo/g' {} +

echo "✓ lododo → lododo replacement completed"
echo ""

# Phase 2: List files with Chinese characters (for manual review)
echo "Phase 2: Identifying Python files with Chinese log messages..."
echo "--------------------------------------"

CHINESE_LOG_FILES=$(find "$BOT_TELEOP_DIR" -name "*.py" -type f -exec grep -l "get_logger()" {} + | xargs grep -l $'[\u4e00-\u9fff]' || true)

if [ -n "$CHINESE_LOG_FILES" ]; then
    echo "Files with Chinese log messages found:"
    echo "$CHINESE_LOG_FILES"
    echo ""
    echo "⚠️  These files need manual English log conversion"
else
    echo "✓ No Chinese log messages found in Python logger calls"
fi

echo ""
echo "========================================"
echo "   Refactoring Summary"
echo "========================================"
echo "✓ Phase 1: Text replacement completed"
echo "⚠️  Phase 2: Please manually convert Chinese logs to English"
echo ""
echo "Files processed:"
find "$BOT_TELEOP_DIR" -name "*.py" -type f | wc -l | xargs echo "  - Python files:"
find "$BOT_TELEOP_DIR" -name "*.sh" -type f | wc -l | xargs echo "  - Shell scripts:"
find "$BOT_TELEOP_DIR" -name "*.xml" -type f | wc -l | xargs echo "  - XML files:"
find "$BOT_TELEOP_DIR/web_frontend" \( -name "*.ts" -o -name "*.tsx" -o -name "*.js" \) -type f | wc -l | xargs echo "  - TS/JS files:"
find "$BOT_TELEOP_DIR/web_frontend" -name "*.json" -type f | wc -l | xargs echo "  - JSON files:"
echo ""
echo "Done!"
