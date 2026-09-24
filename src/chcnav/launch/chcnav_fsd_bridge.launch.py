# chcnav_fsd_bridge.launch.py — 华测 CGI 组合导航 -> FSD 桥接 launch
#
# 一条命令起全链路（RS232 / USB 转串口，与已在设备跑通的 demo_1.py 一致）：
#   CGI-410 --串口--> HcMsgParserLaunchNode --/chcnav/hc_sentence--> HcCgiProtocolProcessNode
#           --/chcnav/devpvt--> ChcnavFsdBridge --/chcnav/odometry + /chcnav/velocity--> WUTA-FSD
#
# 前两个节点为厂商驱动，源码未改动，直接复用其可执行文件；
# 第三、四个为本项目自定义开发节点（src/chcnav_fsd_bridge/、src/chcnav_data_logger/）。
# 厂商示例 launch（demo_0..9.py）保持原样，可另行单独 ros2 launch 做测试。
#
# 第四个节点 ChcnavDataLogger 为运行期数据日志：只订阅话题、不发布、不占 TF，
# 把数据落成可读文本（<包>/log/<时间戳>/ …），默认随桥启动，log:=false 可关闭。
#
# 参数唯一入口：config/chcnav_fsd_bridge.yaml
# 推荐用一键脚本启动（含环境/串口检查与按需构建）：
#   src/chcnav/scripts/start_chcnav_fsd.sh --port=/dev/ttyUSB1 --yaw-offset=1.5
#
# 直接 ros2 launch 亦可，命令行参数优先于 yaml：
#   ros2 launch chcnav chcnav_fsd_bridge.launch.py port:=/dev/ttyUSB1 min_gnss_status:=0
#   ros2 launch chcnav chcnav_fsd_bridge.launch.py log:=false          # 本次不记日志
# 注：若换用 cfg_file:=/other.yaml，请同时显式给出下面 13 个可调参数
#     （用启动脚本 --cfg= 则无需操心，脚本会把解析结果全部传入）。

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'chcnav'
CFG_BASENAME = 'chcnav_fsd_bridge.yaml'

# yaml 中的节点段名（同时用作运行时节点名）
MSG_PARSER_NODE = '/chcnav_msg_parser'
BRIDGE_NODE = '/chcnav_fsd_bridge'
LOGGER_NODE = '/chcnav_data_logger'


def default_cfg_file():
    """yaml 查找顺序：源码树同级 config/ -> 已安装的 share/chcnav/"""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        # 源码树运行: <包>/launch/../config/
        os.path.normpath(os.path.join(here, os.pardir, 'config', CFG_BASENAME)),
        # 已安装: 本包 install(DIRECTORY) 会把 launch/ 与 config/ 的内容平铺到 share/chcnav/
        os.path.join(here, CFG_BASENAME),
    ]
    try:
        share = get_package_share_directory(PKG)
        candidates.insert(2, os.path.join(share, 'config', CFG_BASENAME))
    except Exception:
        pass
    for path in candidates:
        if os.path.isfile(path):
            return path
    return candidates[0]


def default_log_dir():
    """日志默认落在华测包内：优先源码树 <ws>/src/chcnav/log（从 install 的 share 启动也能找到）"""
    here = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.normpath(os.path.join(here, os.pardir))
    # 情况一：直接从源码树启动，<包>/ 同时有 src/ 与 config/
    if os.path.isdir(os.path.join(pkg_dir, 'src')) and os.path.isdir(
            os.path.join(pkg_dir, 'config')):
        return os.path.join(pkg_dir, 'log')
    # 情况二：从 install/chcnav/share/chcnav 启动，向上找工作空间的 src/chcnav
    cur = here
    for _ in range(6):
        probe = os.path.join(cur, 'src', 'chcnav')
        if os.path.isfile(os.path.join(probe, 'package.xml')):
            return os.path.join(probe, 'log')
        cur = os.path.dirname(cur)
    # 兜底：装到哪就写到哪
    try:
        return os.path.join(get_package_share_directory(PKG), 'log')
    except Exception:
        return os.path.join(pkg_dir, 'log')


def load_node_params(cfg_file, node_name):
    """读取 yaml 中 "<节点名>: ros__parameters:" 段"""
    with open(cfg_file, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    return ((data.get(node_name) or {}).get('ros__parameters')) or {}


def _launch_setup(context, *args, **kwargs):
    cfg_file = context.launch_configurations.get('cfg_file') or default_cfg_file()
    if not os.path.isfile(cfg_file):
        raise RuntimeError('未找到参数配置文件: {}'.format(cfg_file))

    parser_params = load_node_params(cfg_file, MSG_PARSER_NODE)
    bridge_params = load_node_params(cfg_file, BRIDGE_NODE)
    logger_params = load_node_params(cfg_file, LOGGER_NODE)

    # 1. 协议二次解析：/chcnav/hc_sentence -> /chcnav/devpvt（厂商节点，无参数）
    node_protocol = Node(
        package=PKG,
        executable='HcCgiProtocolProcessNode',
        name='hc_topic_driver',
        output='screen',
    )

    # 2. 数据源接入：串口/UDP/CAN -> /chcnav/hc_sentence（厂商节点）
    #    8 个可调参数中的前 4 个由命令行/yaml 给到，此处用 LaunchConfiguration 承接
    node_parser = Node(
        package=PKG,
        executable='HcMsgParserLaunchNode',
        name=MSG_PARSER_NODE.lstrip('/'),
        output='screen',
        parameters=[dict(
            parser_params,
            type=ParameterValue(LaunchConfiguration('type'), value_type=str),
            port=ParameterValue(LaunchConfiguration('port'), value_type=str),
            baudrate=ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
            rate=ParameterValue(LaunchConfiguration('rate'), value_type=int),
        )],
    )

    # 3. 自定义桥：/chcnav/devpvt -> /chcnav/odometry + /chcnav/velocity
    node_bridge = Node(
        package=PKG,
        executable='ChcnavFsdBridge',
        name=BRIDGE_NODE.lstrip('/'),
        output='screen',
        parameters=[dict(
            bridge_params,
            min_gnss_status=ParameterValue(
                LaunchConfiguration('min_gnss_status'), value_type=int),
            roll_offset_deg=ParameterValue(
                LaunchConfiguration('roll_offset_deg'), value_type=float),
            pitch_offset_deg=ParameterValue(
                LaunchConfiguration('pitch_offset_deg'), value_type=float),
            yaw_offset_deg=ParameterValue(
                LaunchConfiguration('yaw_offset_deg'), value_type=float),
        )],
    )

    # 4. 运行期数据日志：只订阅 /chcnav 话题，把数据落成可读文本（本项目开发）
    #    log:=false（或脚本 --no-log）时不启动该节点，其余链路不受影响
    log_enabled = context.launch_configurations.get('log', 'true').strip().lower()
    if log_enabled in ('false', '0', 'no', 'off'):
        return [node_protocol, node_parser, node_bridge]

    node_logger = Node(
        package=PKG,
        executable='ChcnavDataLogger',
        name=LOGGER_NODE.lstrip('/'),
        output='screen',
        parameters=[dict(
            logger_params,
            output_dir=ParameterValue(LaunchConfiguration('log_dir'), value_type=str),
            style=ParameterValue(LaunchConfiguration('log_style'), value_type=str),
            topics=ParameterValue(LaunchConfiguration('log_topics'), value_type=str),
        )],
    )

    return [node_protocol, node_parser, node_bridge, node_logger]


def generate_launch_description():
    cfg_default = default_cfg_file()
    try:
        cfg_values = load_node_params(cfg_default, MSG_PARSER_NODE)
        cfg_values.update(load_node_params(cfg_default, BRIDGE_NODE))
        logger_values = load_node_params(cfg_default, LOGGER_NODE)
    except Exception:
        cfg_values = {}
        logger_values = {}

    def d(key, fallback):
        value = cfg_values.get(key, fallback)
        return str(value)

    def dl(key, fallback):
        value = logger_values.get(key, fallback)
        return str(value)

    # 日志目录：yaml 里给了绝对路径就用它，否则默认 <包>/log
    _log_dir_cfg = logger_values.get('output_dir')
    log_dir_default = (
        _log_dir_cfg.strip()
        if isinstance(_log_dir_cfg, str) and os.path.isabs(_log_dir_cfg.strip())
        else default_log_dir()
    )

    # 表驱动声明全部 launch 参数：('参数名', 默认值, 说明)
    arg_specs = [
        ('type', d('type', 'serial'),
         '数据源类型，默认 serial'),
        ('port', d('port', '/dev/ttyUSB0'),
         '串口路径（RS232 / USB 转串口）'),
        ('baudrate', d('baudrate', '460800'),
         '波特率，需与 CGI-410 输出配置一致'),
        ('rate', d('rate', '1000'),
         '节点每秒最大解析协议数'),

        ('min_gnss_status', d('min_gnss_status', '1'),
         '低于该 GNSS 状态不发布 FSD 话题；4/8 为 RTK 固定解，0 为不过滤'),
        ('roll_offset_deg', d('roll_offset_deg', '0.0'),
         '横滚安装角标定偏置(deg)'),
        ('pitch_offset_deg', d('pitch_offset_deg', '0.0'),
         '俯仰安装角标定偏置(deg)'),
        ('yaw_offset_deg', d('yaw_offset_deg', '0.0'),
         '航向安装角标定偏置(deg)'),

        # --- 运行期数据日志（ChcnavDataLogger）---
        ('log', 'true',
         '是否随桥启动数据日志节点；false 表示本次不记录'),
        ('log_dir', log_dir_default,
         '日志根目录，本次会话自动建 <log_dir>/YYYYmmdd_HHMMSS/'),
        ('log_style', dl('style', 'brief'),
         '记录风格：brief 关键字段（默认）| full 全字段'),
        ('log_topics', dl('topics', 'devpvt,odometry,velocity'),
         '要记录的话题短名，逗号分隔'),
    ]

    return LaunchDescription(
        [DeclareLaunchArgument(
            'cfg_file', default_value=cfg_default,
            description='参数配置文件，默认 config/' + CFG_BASENAME)]
        + [DeclareLaunchArgument(name, default_value=value, description=desc)
           for name, value, desc in arg_specs]
        + [OpaqueFunction(function=_launch_setup)]
    )
