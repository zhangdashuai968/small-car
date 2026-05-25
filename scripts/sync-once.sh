#!/bin/bash
# 单次同步脚本 —— 由 Windows 定时任务每 5 分钟调用一次
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_DIR"

if [[ -n $(git status --porcelain) ]]; then
  git add -A
  git commit -m "auto: $(date '+%Y-%m-%d %H:%M:%S')" --quiet
  git push origin main --quiet
fi
