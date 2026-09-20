#!/usr/bin/env bash
# start_chcnav_fsd.sh — 一键启动「华测 CGI 组合导航 -> FSD 桥」
#
# 链路（与已在设备跑通的 demo_1 一致，RS232 / USB 转串口）：
#   CGI-410 --串口--> HcMsgParserLaunchNode --/chcnav/hc_sentence--> HcCgiProtocolProcessNode
#           --/chcnav/devpvt--> ChcnavFsdBridge --/chcnav/odometry + /chcnav/velocity--> WUTA-FSD
#
# 参数来源：命令行 > config/chcnav_fsd_bridge.yaml > 本脚本内置默认
#
# 用法:
#   ./start_chcnav_fsd.sh                          # 按 config/chcnav_fsd_bridge.yaml 启动
#   ./start_chcnav_fsd.sh --port=/dev/ttyUSB1      # 临时换串口（不改 yaml）
#   ./start_chcnav_fsd.sh --baud=230400            # 临时换波特率
#   ./start_chcnav_fsd.sh --min-gnss-status=0      # 无天线台架调试，不过滤定位状态
#   ./start_chcnav_fsd.sh --yaw-offset=1.5         # 临时做安装角标定
#   ./start_chcnav_fsd.sh --cfg=/path/other.yaml   # 换用另一份参数文件
#   ./start_chcnav_fsd.sh --no-log                 # 本次不记录数据日志（默认是记的）
#   ./start_chcnav_fsd.sh --log-style=full         # 日志记全字段（默认 brief 关键字段）
#   ./start_chcnav_fsd.sh --log-topics=devpvt,devimu,odometry,velocity
#   ./start_chcnav_fsd.sh --log-dir=/data/chcnav_log
#   ./start_chcnav_fsd.sh --no-build               # 跳过构建，直接启动
#   ./start_chcnav_fsd.sh --dry-run                # 只做检查与构建，不启动
#
# 它会:
#   1. 检查 ROS 环境与串口设备（存在、可读写）
#   2. 按需 colcon build（缺产物 / 桥源码或 CMakeLists 有更新）
#   3. 前台运行整条链路（Ctrl-C 退出）
#
# 之后另开终端查看:
#   source <工作空间>/install/setup.bash
#   ros2 topic hz   /chcnav/devpvt      # 驱动解析输出
#   ros2 topic echo /chcnav/odometry    # 给 FSD 的位姿
#   ros2 topic echo /chcnav/velocity    # 给 FSD 的车体速度
#
# 数据日志（默认开启，只订阅不影响链路）:
#   less  <包>/log/latest/devpvt.log    # 关键字段文本回放
#   tail -f <包>/log/latest/events.log  # 事件/告警流
#   scripts/view_chcnav_log.sh -h       # 查看脚本
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # chcnav/scripts
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"                     # chcnav 功能包目录
CFG="$PKG_DIR/config/chcnav_fsd_bridge.yaml"                # 参数文件默认位置
BIN_NAME="ChcnavFsdBridge"
LAUNCH_NAME="chcnav_fsd_bridge.launch.py"

# ---------- yaml 读取（内嵌 Python，键写法: /节点名.参数名） ----------
cfg_get() {
  python3 - "$1" "$2" <<'PYEOF' 2>/dev/null || true
import sys
try:
    import yaml
except Exception:
    sys.exit(0)
try:
    with open(sys.argv[1], encoding='utf-8') as f:
        val = yaml.safe_load(f)
except Exception:
    sys.exit(0)
for part in sys.argv[2].split('.'):
    # ROS2 约定键为 "<节点名>: ros__parameters:"，此处允许省略 ros__parameters 一层
    if isinstance(val, dict) and 'ros__parameters' in val and part != 'ros__parameters':
        val = val['ros__parameters']
    if not isinstance(val, dict) or part not in val:
        sys.exit(0)
    val = val[part]
if val is not None:
    print(val)
PYEOF
}

usage() {
  echo "用法: $0 [--cfg=FILE] [--type=serial] [--port=/dev/ttyUSB0] [--baud=460800] [--rate=1000]"
  echo "          [--min-gnss-status=1] [--roll-offset=0] [--pitch-offset=0] [--yaw-offset=0]"
  echo "          [--log|--no-log] [--log-dir=DIR] [--log-style=brief|full] [--log-topics=LIST]"
  echo "          [--build|--no-build] [--dry-run]"
  echo "  --cfg:             参数文件，默认 <包>/config/chcnav_fsd_bridge.yaml"
  echo "  --type:            数据源类型，默认 serial"
  echo "  --port:            串口路径（RS232 / USB 转串口）"
  echo "  --baud:            波特率，需与 CGI-410 输出配置一致"
  echo "  --min-gnss-status: 低于该 GNSS 状态不发布；4/8 为 RTK 固定解，0 为不过滤"
  echo "  --*-offset:        安装角标定偏置(deg)"
  echo "  --no-log:          本次不记录数据日志（默认随桥开启，只订阅不影响链路）"
  echo "  --log-dir:         日志根目录，默认 <包>/log"
  echo "  --log-style:       brief 关键字段（默认，人看优先）| full 全字段"
  echo "  --log-topics:      记录话题短名，逗号分隔"
  echo "                     可选: devpvt devimu hc_sentence nmea_sentence bestpos heading odometry velocity"
  echo "  --build/--no-build: 强制/跳过构建；默认缺产物或源码更新时自动构建"
  echo "  --dry-run:         只做检查与构建，不启动"
}

# ---------- 内置默认（优先级最低） ----------
TYPE="serial"
PORT="/dev/ttyUSB0"
BAUD="460800"
RATE="1000"
MIN_GNSS_STATUS="1"
ROLL_OFFSET="0.0"
PITCH_OFFSET="0.0"
YAW_OFFSET="0.0"
LOG_ON="1"
LOG_DIR="$PKG_DIR/log"
LOG_STYLE="brief"
LOG_TOPICS="devpvt,odometry,velocity"
DO_BUILD="auto"
DRY_RUN="0"

ARGV=("$@")

# ---------- 第一遍: 先解析 --cfg=，保证后续覆盖顺序正确 ----------
for a in "${ARGV[@]}"; do
  case "$a" in
    --cfg=*) CFG="${a#*=}" ;;
  esac
done
if [ ! -f "$CFG" ]; then
  echo "!! 未找到参数文件: $CFG" >&2
  exit 1
fi

# ---------- 第二遍: yaml 覆盖内置默认 ----------
v="$(cfg_get "$CFG" '/chcnav_msg_parser.type')";           TYPE="${v:-$TYPE}"
v="$(cfg_get "$CFG" '/chcnav_msg_parser.port')";           PORT="${v:-$PORT}"
v="$(cfg_get "$CFG" '/chcnav_msg_parser.baudrate')";       BAUD="${v:-$BAUD}"
v="$(cfg_get "$CFG" '/chcnav_msg_parser.rate')";           RATE="${v:-$RATE}"
v="$(cfg_get "$CFG" '/chcnav_fsd_bridge.min_gnss_status')"; MIN_GNSS_STATUS="${v:-$MIN_GNSS_STATUS}"
v="$(cfg_get "$CFG" '/chcnav_fsd_bridge.roll_offset_deg')"; ROLL_OFFSET="${v:-$ROLL_OFFSET}"
v="$(cfg_get "$CFG" '/chcnav_fsd_bridge.pitch_offset_deg')"; PITCH_OFFSET="${v:-$PITCH_OFFSET}"
v="$(cfg_get "$CFG" '/chcnav_fsd_bridge.yaw_offset_deg')";  YAW_OFFSET="${v:-$YAW_OFFSET}"
# 数据日志：output_dir 只在给出绝对路径时才采用（相对路径以 <包>/log 为准）
v="$(cfg_get "$CFG" '/chcnav_data_logger.output_dir')"
if [ -n "$v" ] && [ "${v#/}" != "$v" ]; then LOG_DIR="$v"; fi
v="$(cfg_get "$CFG" '/chcnav_data_logger.style')";         LOG_STYLE="${v:-$LOG_STYLE}"
v="$(cfg_get "$CFG" '/chcnav_data_logger.topics')";        LOG_TOPICS="${v:-$LOG_TOPICS}"

# ---------- 第三遍: 命令行覆盖 yaml ----------
for a in "${ARGV[@]}"; do
  case "$a" in
    --cfg=*)              : ;;   # 第一遍已处理
    --type=*)             TYPE="${a#*=}" ;;
    --port=*)             PORT="${a#*=}" ;;
    --baud=*)             BAUD="${a#*=}" ;;
    --rate=*)             RATE="${a#*=}" ;;
    --min-gnss-status=*)  MIN_GNSS_STATUS="${a#*=}" ;;
    --roll-offset=*)      ROLL_OFFSET="${a#*=}" ;;
    --pitch-offset=*)     PITCH_OFFSET="${a#*=}" ;;
    --yaw-offset=*)       YAW_OFFSET="${a#*=}" ;;
    --log)                LOG_ON="1" ;;
    --no-log)             LOG_ON="0" ;;
    --log-dir=*)          LOG_DIR="${a#*=}" ;;
    --log-style=*)        LOG_STYLE="${a#*=}" ;;
    --log-topics=*)       LOG_TOPICS="${a#*=}" ;;
    --build)              DO_BUILD="yes" ;;
    --no-build)           DO_BUILD="no" ;;
    --dry-run)            DRY_RUN="1" ;;
    -h|--help)            usage; exit 0 ;;
    *) echo "!! 未知参数: $a（用 -h 查看用法）" >&2; exit 1 ;;
  esac
done

echo "==> 参数来源: $CFG（命令行优先）"
echo "    type=$TYPE port=$PORT baud=$BAUD rate=$RATE"
echo "    min_gnss_status=$MIN_GNSS_STATUS 安装角偏置 r=$ROLL_OFFSET p=$PITCH_OFFSET y=$YAW_OFFSET"
if [ "$LOG_ON" = "1" ]; then
  echo "    数据日志: 开  目录=$LOG_DIR  样式=$LOG_STYLE  话题=$LOG_TOPICS"
else
  echo "    数据日志: 关（--no-log）"
fi

echo "==> 1/3 检查 ROS 环境与设备"
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
else
  ROS_SETUP="/opt/ros/humble/setup.bash"
fi
if [ ! -f "$ROS_SETUP" ]; then
  echo "!! 未找到 $ROS_SETUP，请确认已安装 ROS2 Humble" >&2
  exit 1
fi
# ROS 的 setup.bash 会引用未定义变量（如 AMENT_TRACE_SETUP_FILES），需临时关掉 -u
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u
echo "    ROS: ${ROS_DISTRO:-unknown} ($ROS_SETUP)"

# 工作空间根: 优先按源码树位置推断（脚本通常从源码树运行），否则用 ros2 pkg prefix 兜底
WS_DIR="$(cd "$PKG_DIR/../.." && pwd)"
if [ ! -f "$WS_DIR/src/chcnav/package.xml" ]; then
  _prefix="$(ros2 pkg prefix chcnav 2>/dev/null || true)"
  if [ -n "$_prefix" ]; then
    WS_DIR="$(dirname "$(dirname "$_prefix")")"
  fi
fi
if [ ! -f "$WS_DIR/src/chcnav/package.xml" ]; then
  echo "!! 无法定位工作空间（未找到 <WS>/src/chcnav/package.xml），请从源码树运行本脚本" >&2
  exit 1
fi
echo "    工作空间: $WS_DIR"
BIN="$WS_DIR/install/chcnav/lib/chcnav/$BIN_NAME"

if [ "$TYPE" = "serial" ]; then
  if [ ! -e "$PORT" ]; then
    echo "!! 串口 $PORT 不存在。确认设备已上电且串口线已插入:" >&2
    echo "     ls -l /dev/ttyUSB*   dmesg | tail -20" >&2
    exit 1
  fi
  if [ ! -r "$PORT" ] || [ ! -w "$PORT" ]; then
    echo "!! 串口 $PORT 无读写权限，当前用户: $(id -un)" >&2
    echo "     临时: sudo chmod a+rw $PORT" >&2
    echo "     永久: sudo usermod -aG dialout $(id -un) 然后重新登录" >&2
    exit 1
  fi
  echo "    串口: $PORT 就绪 ($(ls -l "$PORT" | awk '{print $1, $3, $4}'))"
else
  echo "    type=$TYPE，跳过串口检查（请自行确认数据源可达）"
fi

echo "==> 2/3 构建工作空间"
need_build="0"
case "$DO_BUILD" in
  yes) need_build="1" ;;
  no)  need_build="0" ;;
  auto)
    if [ ! -x "$BIN" ]; then
      need_build="1"
    elif [ "$PKG_DIR/src/chcnav_fsd_bridge/ChcnavFsdBridge.cpp" -nt "$BIN" ] \
      || [ "$PKG_DIR/src/chcnav_data_logger/ChcnavDataLogger.cpp" -nt "$BIN" ] \
      || [ "$PKG_DIR/CMakeLists.txt" -nt "$BIN" ]; then
      need_build="1"
    fi
    ;;
esac

if [ "$need_build" = "1" ]; then
  echo "    colcon build --packages-select msg_interfaces chcnav"
  (cd "$WS_DIR" && colcon build --packages-select msg_interfaces chcnav)
else
  echo "    跳过（产物已是最新；强制重建用 --build）"
fi

set +u
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"
set -u

# 优先用源码树里的 launch（与当前源码一致），装了包也能跑
LAUNCH_FILE="$PKG_DIR/launch/$LAUNCH_NAME"
if [ ! -f "$LAUNCH_FILE" ]; then
  # 本包 install(DIRECTORY) 会把 launch/ 内容平铺到 share/chcnav/
  LAUNCH_FILE="$WS_DIR/install/chcnav/share/chcnav/$LAUNCH_NAME"
fi
if [ ! -f "$LAUNCH_FILE" ]; then
  echo "!! 未找到 launch 文件 $LAUNCH_FILE" >&2
  exit 1
fi

LAUNCH_ARGS=(
  "cfg_file:=$CFG"
  "type:=$TYPE"
  "port:=$PORT"
  "baudrate:=$BAUD"
  "rate:=$RATE"
  "min_gnss_status:=$MIN_GNSS_STATUS"
  "roll_offset_deg:=$ROLL_OFFSET"
  "pitch_offset_deg:=$PITCH_OFFSET"
  "yaw_offset_deg:=$YAW_OFFSET"
)

if [ "$LOG_ON" = "1" ]; then
  LAUNCH_ARGS+=(
    "log:=true"
    "log_dir:=$LOG_DIR"
    "log_style:=$LOG_STYLE"
    "log_topics:=$LOG_TOPICS"
  )
else
  LAUNCH_ARGS+=("log:=false")
fi

echo "==> 3/3 启动链路"
echo "============================================================"
echo " CGI-410($PORT@$BAUD) -> /chcnav/devpvt -> 桥 -> /chcnav/odometry"
echo "                                              /chcnav/velocity"
if [ "$LOG_ON" = "1" ]; then
  echo " 数据日志: $LOG_DIR/<时间戳>/ (样式 $LOG_STYLE, 话题 $LOG_TOPICS)"
  echo "   回放: less $LOG_DIR/latest/devpvt.log"
  echo "   告警: tail -f $LOG_DIR/latest/events.log"
else
  echo " 数据日志: 本次不记录"
fi
echo " 保持此终端运行（Ctrl-C 退出）。另开终端查看:"
echo "   source $WS_DIR/install/setup.bash"
echo "   ros2 topic hz   /chcnav/devpvt"
echo "   ros2 topic echo /chcnav/odometry"
echo " 排查: 无 /chcnav/devpvt 说明串口/波特率/设备输出协议需确认;"
echo "       有 devpvt 无 odometry 多为 GNSS 状态低于 min_gnss_status（可试 --min-gnss-status=0）"
echo "============================================================"

if [ "$DRY_RUN" = "1" ]; then
  echo "==> --dry-run: 检查与构建已通过，未启动"
  echo "    将执行: ros2 launch $LAUNCH_FILE ${LAUNCH_ARGS[*]}"
  exit 0
fi

exec ros2 launch "$LAUNCH_FILE" "${LAUNCH_ARGS[@]}"
