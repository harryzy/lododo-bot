#!/usr/bin/env python3
"""
Find Python files with Chinese characters and list Chinese log messages
"""
import os
import re
from pathlib import Path

BOT_TELEOP_DIR = Path("/home/hurry/workDisk/lododo_bot/src/bot_teleop")

# Pattern to match Chinese characters
CHINESE_PATTERN = re.compile(r'[\u4e00-\u9fff]+')

def has_chinese(text):
    """Check if text contains Chinese characters"""
    return bool(CHINESE_PATTERN.search(text))

def find_chinese_in_file(file_path):
    """Find lines with Chinese characters in a file"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            chinese_lines = []
            for i, line in enumerate(lines, 1):
                if has_chinese(line):
                    chinese_lines.append((i, line.strip()))
            return chinese_lines
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return []

def main():
    print("=" * 60)
    print("  Finding Chinese text in bot_teleop Python files")
    print("=" * 60)
    print()
    
    python_files = list(BOT_TELEOP_DIR.rglob("*.py"))
    files_with_chinese = []
    
    for py_file in python_files:
        chinese_lines = find_chinese_in_file(py_file)
        if chinese_lines:
            files_with_chinese.append((py_file, chinese_lines))
    
    if not files_with_chinese:
        print("✓ No Chinese text found in Python files!")
        return
    
    print(f"Found {len(files_with_chinese)} files with Chinese text:\n")
    
    for file_path, chinese_lines in files_with_chinese:
        rel_path = file_path.relative_to(BOT_TELEOP_DIR)
        print(f"\n📄 {rel_path}")
        print("─" * 60)
        for line_num, line in chinese_lines[:5]:  # Show first 5 lines
            print(f"  Line {line_num}: {line[:80]}")
        if len(chinese_lines) > 5:
            print(f"  ... and {len(chinese_lines) - 5} more lines")
    
    print("\n" + "=" * 60)
    print(f"Total: {len(files_with_chinese)} files need Chinese→English conversion")
    print("=" * 60)

if __name__ == "__main__":
    main()
