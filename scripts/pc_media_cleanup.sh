#!/usr/bin/env bash
# PC 录像清理：车机由 deleter.py 按“剩余空间阈值”清理，但 PC 上 915G 盘的剩余
# 空间永远高于阈值，realdata 会无限增长。这里按“保留最近 N 段 + 总容量上限”
# 清理，供 start_carrotpilot_with_layout.sh 在启动前调用。
#
# 可调环境变量：
#   PC_MEDIA_KEEP_SEGMENTS  保留的最近段数（默认 300）
#   PC_MEDIA_MAX_BYTES      总容量上限（默认 20G，支持 K/M/G 后缀）

set -uo pipefail

MEDIA_ROOT="${MEDIA_ROOT:-$HOME/.comma/media/0/realdata}"
KEEP_SEGMENTS="${PC_MEDIA_KEEP_SEGMENTS:-300}"
MAX_BYTES="${PC_MEDIA_MAX_BYTES:-20G}"

if [ ! -d "$MEDIA_ROOT" ]; then
  exit 0
fi

# 按创建时间排序：ls -1 的目录名即 segment，stat 取创建时间不可靠，直接用 -t（修改时间）
segments="$(ls -1t "$MEDIA_ROOT" 2>/dev/null || true)"
if [ -z "$segments" ]; then
  exit 0
fi

total="$(du -sb "$MEDIA_ROOT" 2>/dev/null | awk '{print $1}')"
limit_bytes="$(numfmt --from=iec "$MAX_BYTES" 2>/dev/null || echo 21474836480)"

removed=0
idx=0
while IFS= read -r seg; do
  [ -n "$seg" ] || continue
  idx=$((idx + 1))
  if [ "$idx" -le "$KEEP_SEGMENTS" ] && [ "${total:-0}" -le "$limit_bytes" ]; then
    break
  fi
  rm -rf "${MEDIA_ROOT:?}/${seg}"
  removed=$((removed + 1))
  total="$(du -sb "$MEDIA_ROOT" 2>/dev/null | awk '{print $1}')"
done <<< "$segments"

if [ "$removed" -gt 0 ]; then
  echo "[media] 清理 $removed 个旧段（保留最近 $KEEP_SEGMENTS 段 / 上限 $MAX_BYTES）"
fi
