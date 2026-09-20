# 华测 CGI 组合导航 ROS2 驱动包

本仓库 = **华测（CHCNAV）原厂 CGI 驱动** + **本项目为 WUTA-FSD 开发的自定义扩展包**。

- 原厂部分：串口/TCP/UDP/CAN 接入 CGI-410，解析成 ROS2 话题。**源码未改动**，用法见
  [readme/README_ZH.md](readme/README_ZH.md)（原厂文档）与 [docs/CGI-410.md](docs/CGI-410.md)（设备手册）。
- 自定义部分：把定位结果转成 FSD 需要的 `odometry` / `velocity`，并把运行数据落成可读文本日志。

```
CGI-410 ──串口──> HcMsgParserLaunchNode ──/chcnav/hc_sentence──> HcCgiProtocolProcessNode
        ──/chcnav/devpvt──> ChcnavFsdBridge ──/chcnav/odometry + /chcnav/velocity──> WUTA-FSD
                              └── 旁路订阅 ──> ChcnavDataLogger ──> log/<时间戳>/*.log
```

---

## 1. 目录结构

| 路径 | 说明 |
| --- | --- |
| `src/chcnav/` | 驱动功能包（原厂节点 + 自定义节点 + launch + 脚本 + 配置） |
| `src/msg_interfaces/` | 自定义消息（`HcSentence`、`DevPvt` 等） |
| `readme/README_ZH.md` | **原厂中文文档**（话题、参数、demo、排查） |
| `docs/CGI-410.md` | CGI-410 设备手册 |
| `launch/demo_*.py` `demo_*.xml` | 原厂示例（串口/CAN/TCP/UDP/NTRIP/时间均匀度…） |

`src/chcnav/` 内：

| 路径 | 说明 |
| --- | --- |
| `src/hc_msg_parser_node/`、`src/hc_cgi_protocol_process_node/`、`src/ntrip_server_node/` | 原厂驱动节点（未改动） |
| `src/chcnav_fsd_bridge/ChcnavFsdBridge.cpp` | **自定义**：devpvt → odometry/velocity |
| `src/chcnav_data_logger/ChcnavDataLogger.cpp` | **自定义**：运行期数据文本日志 |
| `config/chcnav_fsd_bridge.yaml` | **唯一调参入口**（三段参数） |
| `launch/chcnav_fsd_bridge.launch.py` | 一键起全链路（4 个节点） |
| `scripts/start_chcnav_fsd.sh` | 一键启动脚本（环境/串口检查 + 按需构建 + 前台运行） |
| `scripts/view_chcnav_log.sh` | 日志回看脚本 |
| `log/` | 日志输出根目录 |

---

## 2. 环境与编译

```bash
# 依赖：ROS2 Humble / Ubuntu 22.04
cd /home/ubuntu22/WUTA_HIL_TEST/humble-chcnav-cgi_ros2pkg
source /opt/ros/humble/setup.bash
colcon build --packages-select msg_interfaces chcnav
source install/setup.bash
```

串口权限（只需一次，重新插拔或换 USB 口后要重做）：

```bash
ls -l /dev/ttyUSB*                  # 确认设备存在
sudo chmod 666 /dev/ttyUSB0         # 临时放开读写权限
# 或长期方案：把当前用户加入 dialout 组后重新登录
sudo usermod -aG dialout $USER
```

---

## 3. 快速开始

```bash
# 一键启动（默认 /dev/ttyUSB0 @460800，自动构建，前台运行，Ctrl-C 退出）
src/chcnav/scripts/start_chcnav_fsd.sh

# 常用变体
src/chcnav/scripts/start_chcnav_fsd.sh --port=/dev/ttyUSB1      # 临时换串口
src/chcnav/scripts/start_chcnav_fsd.sh --baud=230400            # 临时换波特率
src/chcnav/scripts/start_chcnav_fsd.sh --min-gnss-status=0      # 无天线台架调试，不过滤定位状态
src/chcnav/scripts/start_chcnav_fsd.sh --yaw-offset=1.5         # 临时做安装角标定
src/chcnav/scripts/start_chcnav_fsd.sh --no-log                 # 本次不记日志
src/chcnav/scripts/start_chcnav_fsd.sh --dry-run                # 只检查+构建，不启动
```

另开终端验证数据：

```bash
source install/setup.bash
ros2 topic hz   /chcnav/devpvt     # 驱动解析输出
ros2 topic echo /chcnav/odometry   # 给 FSD 的位姿
ros2 topic echo /chcnav/velocity   # 给 FSD 的车体速度
```

---

## 4. 原厂家调试流程

> 完整细节见 [readme/README_ZH.md](readme/README_ZH.md)，此处只列主干。

**4.1 节点组成**

| 节点 | 作用 |
| --- | --- |
| `HcMsgParserLaunchNode` | 把串口/TCP/UDP/CAN/文件的原始字节流切分成协议句，发 `/chcnav/hc_sentence` |
| `HcCgiProtocolProcessNode` | 解析句，产出 `/chcnav/devpvt`、`/chcnav/devimu` 等 |
| `NtripServerLaunchNode` | 可选，NTRIP 差分账号接入 |

**4.2 通用参数**（原厂节点）

| 参数 | 说明 |
| --- | --- |
| `type` | 数据源：`serial` / `tcp` / `tcp_server` / `udp` / `can` / `file` |
| `port` / `baudrate` | `type=serial` 时的串口路径与波特率（须与 CGI-410 输出一致） |
| `rate` | 每秒最大解析协议数 |
| `enable_read` / `enable_write` | 是否开读/写通道 |

> 注意：节点名不要以数字或下划线开头，否则原厂 launch 会报错。

**4.3 设备端连接配置**：串口 / TCP / UDP / CAN 四种接入，需在 CGI-410 侧把对应接口和输出协议打开、与 ROS 端 `type` 对齐。

**4.4 官方示例**（`src/chcnav/launch/`）

| demo | 场景 |
| --- | --- |
| `demo_0` | CAN 接入 |
| `demo_1` | 串口接入（本项目链路同此） |
| `demo_2` / `demo_3` | TCP / UDP 接入 |
| `demo_4` | 混合接入 |
| `demo_5` | 文件回放 |
| `demo_6` | NTRIP 差分 |
| `demo_7` / `demo_8` | 时间均匀度测试 |
| `demo_9` | fix / imu 输出 |

```bash
ros2 launch chcnav demo_1.py     # 单独验证原厂串口链路
```

**4.5 话题说明**（原厂）

| 话题 | 说明 | 类型 |
| --- | --- | --- |
| `/chcnav/nmea_sentence` | NMEA 语句（GPGGA/GPCHC/GPRMC…），已做 XOR 校验 | `msg_interfaces/String` |
| `/chcnav/hc_sentence` | 华测 CGI 原生协议原句（HCINSPVATZCB/HCRAWIMUB…） | `msg_interfaces/HcSentence` |
| `/chcnav/devpvt` | 组合导航定位结果（**本项目桥的输入**） | `msg_interfaces/Hcinspvatzcb` |
| `/chcnav/devimu` | 原始 IMU | `msg_interfaces/Hcrawimub` |
| `/chcnav/bestpos` | NovAtel GNSS 定位（仅收到 NovAtel 协议时发布） | `msg_interfaces/BESTPOS` |
| `/chcnav/heading` | NovAtel 双天线航向 | `msg_interfaces/HEADING` |
| `/fix` / `/imu` | ROS 标准消息（demo_9 输出的 fix/imu） | `sensor_msgs/*` |

> `/chcnav/*` 为自定义消息，需 `source install/setup.bash` 后方可订阅；`/fix`、`/imu` 为 ROS 标准消息。

**4.6 坐标系与时间戳**

- 角度遵循 ROS REP-103（右手系，x 前 y 左 z 上）。
- 时间戳为 GPS 时间换算：`ros_time = gps_week*7*24*3600 + gps_seconds + 315964800 - leaps`，**不是**本地墙钟。

**4.7 常见问题**（摘要）

| 现象 | 排查方向 |
| --- | --- |
| 没数据 | 串口是否被占用 / 波特率是否与设备一致 / `type` 是否配对 |
| 无 `/chcnav/devpvt` | 设备输出协议未开 / `hc_sentence` 无数据 |
| 时间戳跳变 | GPS 周数/闰秒配置 |
| launch 报错 | 节点名命名不规范 |

更多见原厂文档「常见问题排查」章节。

---

## 5. 自定义包介绍

### 5.1 ChcnavFsdBridge

**职责**：订阅厂商 `/chcnav/devpvt`，转成 FSD 需要的两条话题。

| 方向 | 话题 | 类型 |
| --- | --- | --- |
| 输入 | `/chcnav/devpvt` | `msg_interfaces/Hcinspvatzcb` |
| 输出 | `/chcnav/odometry` | `nav_msgs/Odometry` |
| 输出 | `/chcnav/velocity` | `geometry_msgs/TwistStamped` |

**设计约束**

- **不发布 TF**：`odom → base_link` 由 FSD 的 `robot_localization` EKF 独占发布，避免冲突。
- **时间戳沿用 GPS 时间**，不替换为本地时钟。
- 姿态/角速度统一 REP-103。
- 协方差取设备上报的标准差，并设下限，防止 EKF 门限失效。
- 按 `min_gnss_status` 过滤：低于门限不发布（台架无天线时设为 0）。

### 5.2 ChcnavDataLogger

**职责**：只订阅、不发布、不占 TF，把运行期数据落成**人可直接阅读的文本日志**。

- 默认随桥一起启动，`--no-log` / `log:=false` 可关闭。
- 输出：`log/<YYYYmmdd_HHMMSS>/<话题>.log` + `events.log`（rosout 告警）+ `latest` 软链。
- 两种风格：`brief`（关键字段，默认）| `full`（全字段，排查用）。
- 支持单文件轮转（`rotate_max_mb`）与定期 flush（`flush_interval_s`）。
- 可记录话题：`devpvt` `devimu` `hc_sentence` `nmea_sentence` `bestpos` `heading` `odometry` `velocity`。

### 5.3 参数入口

唯一入口 **[config/chcnav_fsd_bridge.yaml](src/chcnav/config/chcnav_fsd_bridge.yaml)**，三段：

| 段名 | 对应节点 |
| --- | --- |
| `/chcnav_msg_parser` | 原厂解析节点（`type` / `port` / `baudrate` / `rate`） |
| `/chcnav_fsd_bridge` | 自定义桥（话题名 / 坐标系 / 原点经纬高 / `min_gnss_status` / 安装角偏置） |
| `/chcnav_data_logger` | 自定义日志（`output_dir` / `style` / `topics` / `flush_interval_s` / `rotate_max_mb` / `rosout`） |

优先级：**命令行 > 本文件 > launch 内置默认**。改 yaml **无需重新编译**，重跑启动脚本即可生效。

---

## 6. 自定义包调试方法

### 6.1 使用一键脚本（推荐）

参数来源：命令行 > yaml > 脚本内置默认。常用参数：

| 参数 | 说明 | 默认 |
| --- | --- | --- |
| `--type` | 数据源类型 | `serial` |
| `--port` | 串口路径 | `/dev/ttyUSB0` |
| `--baud` | 波特率 | `460800` |
| `--rate` | 每秒最大解析协议数 | `1000` |
| `--min-gnss-status` | 定位质量门限，`0` 不过滤，`4/8` RTK 固定解 | `1` |
| `--roll/pitch/yaw-offset` | 安装角标定偏置(deg) | `0.0` |
| `--log` / `--no-log` | 本次是否记日志 | 记（`--log`） |
| `--log-dir` | 日志根目录 | `<包>/log` |
| `--log-style` | `brief` / `full` | `brief` |
| `--log-topics` | 记录话题短名，逗号分隔 | `devpvt,odometry,velocity` |
| `--cfg` | 换用另一份参数文件 | `<包>/config/chcnav_fsd_bridge.yaml` |
| `--build` / `--no-build` | 强制/跳过构建 | 缺产物或源码更新时自动构建 |
| `--dry-run` | 只检查+构建，不启动 | 关 |

### 6.2 直接用 launch

```bash
ros2 launch chcnav chcnav_fsd_bridge.launch.py port:=/dev/ttyUSB1 min_gnss_status:=0
ros2 launch chcnav chcnav_fsd_bridge.launch.py log:=false        # 本次不记日志
ros2 launch chcnav chcnav_fsd_bridge.launch.py --show-args       # 查看全部可调参数
```

> 换用 `cfg_file:=/other.yaml` 时需同时显式给出全部可调参数；用启动脚本 `--cfg=` 则无需操心。

### 6.3 查看数据日志

```bash
src/chcnav/scripts/view_chcnav_log.sh                  # 列出所有会话
src/chcnav/scripts/view_chcnav_log.sh devpvt           # 看最近一次 devpvt（less 翻页）
src/chcnav/scripts/view_chcnav_log.sh events           # 看事件/告警流
src/chcnav/scripts/view_chcnav_log.sh devpvt -f        # 实时跟随
src/chcnav/scripts/view_chcnav_log.sh devpvt -n 500    # 只看最后 500 行
src/chcnav/scripts/view_chcnav_log.sh -s 20250920_193000 devpvt   # 指定某次会话
```

日志根目录可用环境变量 `CHCNAV_LOG_DIR` 覆盖。

### 6.4 联调排障清单

| 现象 | 排查 |
| --- | --- |
| 启动即报串口不可读写 | `ls -l /dev/ttyUSB*`；`sudo chmod 666` 或加入 `dialout` 组 |
| `/chcnav/devpvt` 无频率 | 设备未输出该协议 / 波特率不符 / 串口接错 |
| `odometry` 不发布 | `min_gnss_status` 门限过高（无天线时设 0） |
| 航向/水平度有固定偏差 | 用 `--yaw-offset` 等微调安装角偏置 |
| 坐标跳变 | 原点 `origin_lat/lon/alt` 设成 `.nan`（自动取首个有效定位）或有统一原点 |
| 时间戳异常 | 检查 GPS 时间换算与闰秒配置 |
| 想看某话题原始值 | 加 `--log-style=full` 与 `--log-topics=` 对应话题 |

---

## 7. 常用命令速查

```bash
# 编译
colcon build --packages-select msg_interfaces chcnav && source install/setup.bash

# 启动整条链路（默认记日志）
src/chcnav/scripts/start_chcnav_fsd.sh

# 只验证原厂串口链路
ros2 launch chcnav demo_1.py

# 检查话题
ros2 topic list | grep chcnav
ros2 topic hz   /chcnav/devpvt
ros2 topic echo /chcnav/odometry

# 回看日志
src/chcnav/scripts/view_chcnav_log.sh devpvt
```

---

原厂完整文档：[readme/README_ZH.md](readme/README_ZH.md)
