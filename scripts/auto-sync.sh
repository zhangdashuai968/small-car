#!/bin/bash
# 自动同步脚本 —— 每 30 秒检查变更，自动 add/commit/push
# 用法：在 Git Bash 中运行 bash scripts/auto-sync.sh
# 停止：Ctrl+C

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_DIR"

echo "========================================"
echo "  小车项目 - 自动同步已启动"
echo "  仓库: $REPO_DIR"
echo "  每 30 秒检查一次文件变更"
echo "  按 Ctrl+C 停止"
echo "========================================"

while true; do
  # 检查是否有变更
  if [[ -n $(git status --porcelain) ]]; then
    TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$TIMESTAMP] 检测到文件变更，正在同步..."

    git add -A
    git commit -m "auto: $TIMESTAMP" --quiet
    git push origin main --quiet

    echo "[$TIMESTAMP] 同步完成 ✓"
  fi
  sleep 30
done
