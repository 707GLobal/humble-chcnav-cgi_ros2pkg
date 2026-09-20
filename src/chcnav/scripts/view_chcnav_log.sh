#!/usr/bin/env bash
# view_chcnav_log.sh — 查看 ChcnavDataLogger 落下的文本日志（薄封装）
#
# 日志位置：<包>/log/<YYYYmmdd_HHMMSS>/<话题>.log，latest 软链指向最近一次
#
# 用法:
#   ./view_chcnav_log.sh                  # 列出所有会话
#   ./view_chcnav_log.sh devpvt           # 看最近一次 devpvt（less 翻页）
#   ./view_chcnav_log.sh events           # 看最近一次事件/告警流
#   ./view_chcnav_log.sh devpvt -f        # 实时跟随（tail -f，边跑边看）
#   ./view_chcnav_log.sh devpvt -n 500    # 只看最后 500 行
#   ./view_chcnav_log.sh -s 20250920_193000 devpvt   # 指定某次会话
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_ROOT="${CHCNAV_LOG_DIR:-$PKG_DIR/log}"

usage() {
  echo "用法: $0 [话题] [-f] [-n 行数] [-s 会话] [-l]"
  echo "  话题: devpvt | devimu | hc_sentence | nmea_sentence | bestpos | heading"
  echo "        | odometry | velocity | events   （缺省=列出会话）"
  echo "  -f        实时跟随（tail -f）"
  echo "  -n N      只看最后 N 行"
  echo "  -s 会话   指定会话目录名，如 20250920_193000（默认 latest）"
  echo "  -l        列出所有会话（含大小）"
  echo "  -h        显示本帮助"
  echo "日志根目录: $LOG_ROOT（可用环境变量 CHCNAV_LOG_DIR 覆盖）"
}

SESSION="latest"
FOLLOW="0"
LINES=""
TOPIC=""
LIST="0"

while [ $# -gt 0 ]; do
  case "$1" in
    -f)   FOLLOW="1" ;;
    -n)   shift; LINES="${1:-200}" ;;
    -s)   shift; SESSION="${1:-latest}" ;;
    -l)   LIST="1" ;;
    -h|--help) usage; exit 0 ;;
    -*)   echo "!! 未知选项: $1（用 -h 查看用法）" >&2; exit 1 ;;
    *)    TOPIC="$1" ;;
  esac
  shift
done

if [ ! -d "$LOG_ROOT" ]; then
  echo "!! 还没有日志目录: $LOG_ROOT" >&2
  echo "   先启动一次链路（scripts/start_chcnav_fsd.sh），日志默认开启" >&2
  exit 1
fi

if [ "$LIST" = "1" ] || [ -z "$TOPIC" ]; then
  echo "==> 日志根目录: $LOG_ROOT"
  if [ -L "$LOG_ROOT/latest" ]; then
    echo "    latest -> $(basename "$(readlink -f "$LOG_ROOT/latest")")"
  fi
  found="0"
  while IFS= read -r d; do
    found="1"
    printf '    %s  ( %s )\n' "$(basename "$d")" "$(du -sh "$d" 2>/dev/null | cut -f1)"
    ls -1 "$d" 2>/dev/null | sed 's/^/        /'
  done < <(find "$LOG_ROOT" -mindepth 1 -maxdepth 1 -type d -name '20*' | sort)
  if [ "$found" = "0" ]; then
    echo "    （暂无会话目录）"
  fi
  [ -z "$TOPIC" ] && exit 0
fi

SESSION_DIR="$LOG_ROOT/$SESSION"
if [ ! -d "$SESSION_DIR" ]; then
  echo "!! 找不到会话目录: $SESSION_DIR" >&2
  exit 1
fi

FILE="$SESSION_DIR/$TOPIC.log"
if [ ! -f "$FILE" ]; then
  echo "!! 该会话没有 $TOPIC 日志，现有文件:" >&2
  ls -1 "$SESSION_DIR" >&2
  exit 1
fi

if [ "$FOLLOW" = "1" ]; then
  echo "==> 实时跟随 $FILE（Ctrl-C 退出）"
  exec tail -f -n "${LINES:-50}" "$FILE"
fi

if [ -n "$LINES" ]; then
  exec tail -n "$LINES" "$FILE"
fi

exec less "$FILE"
