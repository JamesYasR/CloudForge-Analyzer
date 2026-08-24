#!/usr/bin/env bash
# CloudForgeAnalyzer 启动脚本（Linux）
# 用法: ./run.sh 或 双击运行
cd "$(dirname "$0")"

# Qt 已启用 fontconfig，自动发现系统字体（含中文 Noto CJK / 宋体）
# 如遇个别环境字体异常，可取消下一行注释强制指定字体目录
# export QT_QPA_FONTDIR=/usr/share/fonts

exec ./build/CloudForgeAnalyzer "$@"
