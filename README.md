# Wuji Hand 一/二代手套遥操 · 2.3.0-test.1

> **测试版，尚未进行实体手/手套验收。** 本次仅发布 `v2.3.0-test.1` 预发布版；待实体网络和两代遥操实测通过后再发布正式 `v2.3.0`。Debian 包版本使用 `2.3.0~test.1-1`，后续可正常升级到 `2.3.0-1`。

支持一代 USB 手、二代以太网手使用同一款 Wuji 手套。2.3.0 新增手套网络自动匹配：发现手套后，为机器人有线网卡补充同网段地址，解决“扫描到了手套，连接却误走 Wi-Fi 而超时”的问题。保留机器人原 IP，手套 IP 不变。二代另外提供优化遥操、ROS、示教、姿态切换和网页功能。目标环境为 Ubuntu 22.04 / Jetson AGX Orin 或 Linux x86_64。

安装不会连接设备、上使能、启动驱动或配置开机自启。所有会运动的命令都由操作员手动执行。一个时刻仅使用一个控制入口：SDK 直控、ROS 遥操、姿态播放或 bag 回放。

## 四步开始遥操（一代、二代共用）

先接好电源、手和手套。一代手使用 USB；二代手通过以太网连接，手端网段需已可达。手套接机器人的有线接口或同一交换机。默认支持自动发现、每侧一只手和一只手套；只接单侧也可以。先退出 Studio 或其他控制进程。

下载 [v2.3.0-test.1 测试安装包](https://github.com/Gento-Teleoperation-Apex/wuji-hand2-glove-teleop/releases/tag/v2.3.0-test.1)。Jetson 用 `arm64`，PC 通常用 `amd64`（`dpkg --print-architecture` 查看）。在安装包所在目录执行：

```bash
# 1. 安装应用（PC 将 arm64 换成 amd64）
sudo apt install ./wuji-hand2-glove-teleop_2.3.0~test.1-1_arm64.deb

# 2. 安装 SDK 环境，需要联网；普通用户运行，不加 sudo
wuji-setup-runtime

# 3. 自动匹配手套网络并验证连接；按提示输入 sudo 密码
wuji-glove-network --apply

# 4. 自动识别一代/二代与左右侧，开始遥操（会使能机械手）
wuji-teleop
```

第 3 步出现每只手套“收到 21 点骨架”和“手套网络就绪”后再运行第 4 步；失败时先按输出处理，不应跳过。第 4 步无脚踏，启动匹配后立即跟随，`Ctrl+C` 停止并尝试下使能。无新骨架超过 1 秒会退出控制。不会启动 ROS 或网页，不需要填写 IP/SN。

需要明确代际/侧别时，**替换第 4 步**：

```bash
wuji-hand1-sdk                              # 仅一代 USB 手，单侧/双侧自动匹配
wuji-hand1-sdk --side right                 # 必须有右手和右手套
wuji-teleop --hand-model wujihand2           # 仅二代以太网手
wuji-teleop --hand-model wujihand2 --side both # 必须双侧齐全
```

`wuji-teleop --side auto`（默认）运行所有完整配对，缺失的一侧会提示并跳过；`--side both` 则缺任一侧就失败。自动模式遇到同侧两只手（例如一、二代同时接入）或同侧多个手套时拒绝启动，请明确选择代际或断开多余设备。旧命令 `wujihand2-teleop-real` 等价于 `wuji-teleop`。

安装路径为 `/opt/kernelmind/wuji-hand2/`，与从 Desktop 还是 Downloads 执行安装无关。命令安装到 `/usr/bin/`，SDK 环境默认在 `~/.venvs/wuji/`。2.3.0 同时打包一代基础示例，例如只读反馈检查：

```bash
~/.venvs/wuji/bin/python /opt/kernelmind/wuji-hand2/wuji_hand/0.subscribe.py
```

重启或有线连接重建后，重新执行第 3、4 步即可；不用重新安装 deb/SDK。

## 手套网络自动匹配的行为

`wuji-glove-network` 一、二代共用，**为机器人网卡添加地址，手套的 IP 不变**。它不会配置二代手本身的 IP，也不会启用机械手。

- SDK 扫描手套的真实 SN 和 IPv4 地址，不依赖 README 中的示例地址。
- 在已连接的独立物理有线接口上做 ARP 探测，定位手套所在接口；不会从当前默认路由猜测接口，也不会改 Wi-Fi、Docker、桥接或 VLAN。
- 若该接口缺少手套网段，选择未被发现设备/本机使用且通过 ARP 冲突检测的主机地址。默认按 `/24` 补充，优先选择 `.200`，占用则尝试下一个。SDK 发现信息不含掩码，特殊掩码请显式传 `--prefix-length`。
- 检查路由和源地址，再使用 SDK 直连每只手套，核对 SN 并接收有效 21 点骨架。不同手套同 IP、接口不唯一、其他网卡/路由网段重叠时停止并说明原因。
- 只添加临时地址，保留原有地址、默认网关和网络管理器配置，不写手套固件、不重启设备。失败/中断会撤回本次添加的地址；已经存在的地址不删除。成功后打印撤销命令。重复运行会复用可用地址。

例如机器人 `eth0=6.6.7.100/24`，手套为 `192.168.1.100` / `.101`，原路由误走 Wi-Fi：工具会确认手套在 `eth0`，为其补充例如 `192.168.1.200/24`，随后验证手套连接。`.200` 只是候选，不是写死的部署地址。ARP 无法发现离线设备的占用；生产网络应预留可用地址，避免其他主机之后使用同一地址。

```bash
wuji-glove-network                         # 只预览；ARP 探测仍可能要求 sudo，不改地址
wuji-glove-network --apply                 # 自动匹配 + 连接验证
wuji-glove-network --apply --interface eth0 # 多有线接口时指定，仍需通过 ARP 验证
wuji-glove-network --apply --sn YOUR_GLOVE_SN # 只处理选定手套，可重复 --sn
wuji-glove-network --help
```

普通用户运行，工具仅对 `arping` 和 `ip address add/del` 调用 sudo，SDK 仍使用当前用户的数据。依赖 `iproute2`、`iputils-arping`、`sudo`，deb 自动安装。网络准备成功只证明手套连接和骨架可读，不代表手端连接/运动已验收。若 ARP 无响应、手套未被 SDK 发现或网络不属于直接连接的有线场景，工具会报错，需要先处理接线、供电或现场网络。

## 一代与二代的参数设置

两代使用相同手套骨架输入，但 SDK 手模型、控制接口和平滑方式不同。以下是 `wuji-teleop` / `wuji-hand1-sdk` / `wujihand2-teleop-real` 的默认值；默认电流相同，不代表控制参数可以互换。

| 参数 | 一代 USB 手 | 二代以太网手 |
| --- | --- | --- |
| SDK 手模型 | `HandModel.WujiHand` | `HandModel.WujiHand2` |
| 指令接口 | `realtime_controller().set_target_position` | `joint_command()` + `JointCommand` |
| 电流限值 | `--hand1-effort-limit 1.5`，A | `--hand2-effort-limit 1.5`，A |
| 平滑 | `--hand1-lowpass-hz 5.0`，SDK LowPass 截止频率 Hz | `--hand2-ema 0.35`，应用层 EMA 系数 |
| MIT 参数 | 这个入口不设置一代 kp/kd | `--hand2-kp 5.0 --hand2-kd 0.15` |
| 循环目标上限 | `--fps 120` | `--fps 120` |
| 骨架超时退出 | `--stale-timeout 1.0`，秒 | 同左 |

一代截止频率越低通常越平滑、响应越慢；二代 EMA 使用 `q = (1-alpha)*q_old + alpha*q_new`，alpha 越小越平滑。电流/MIT 参数按设备允许范围和负载调试；这里的数值是项目默认配置，不是硬件额定上限。程序拒绝非有限数、非正值及超过 1 的 EMA。

以下是显式写出默认参数的运行示例（会启动运动），用户可按设备情况修改数值：

```bash
wuji-hand1-sdk --side right --hand1-effort-limit 1.5 --hand1-lowpass-hz 5.0 --fps 120
wuji-teleop --hand-model wujihand2 --side right \
  --hand2-effort-limit 1.5 --hand2-kp 5.0 --hand2-kd 0.15 --hand2-ema 0.35
```

上述 CLI 参数仅作用于通用入口，选中哪代手就使用哪代参数；它们不会修改另一进程或 ROS 驱动。查看全部选项：`wuji-teleop --help`。

源码调参位置：

- 通用一/二代入口：`examples/python/retargeting/1.teleop_real.py` 的 `parse_args()` 是 CLI 默认值来源，电机配置分别在 `configure_wuji_hand()` / `configure_wuji_hand_2()`。安装后位于 `/opt/kernelmind/wuji-hand2/retargeting/`。优先使用 CLI，升级不会丢失你保存的启动命令；直接修改已安装源码会在升级时被替换。
- 二代高级 `wuji-hand2-sdk` / `wuji-hand2-tuned`：`examples/python/retargeting/2.teleop_tuned.py` 的 `HAND2_KP=5.0`、`HAND2_KD=0.15`、`HAND2_EFFORT_LIMIT=1.5`、`HAND2_QPOS_EMA=0.35` 和 `FPS=120`。这些入口不接受上述 `--hand2-*` 参数，修改这里的常量后重启对应进程。
- 二代 ROS 驱动 `wuji-hand2-driver`：`examples/python/retargeting/wujihand2_ros_driver.py` 中同名 `HAND2_*` 常量，需单独修改，手套发送端不会替驱动设置电机参数。
- 二代优化映射参数：`wuji-hand2-tuned --help`，可改 `--finger-scaling`、`--pinky-scale`、`--pinky-flex-gain`、`--pinky-open-bias`、`--opposition-enable-distance`、`--opposition-close`、`--pinch-flex-boost`，默认值与运行示例见第 3 节。通用入口不带这些增强。

下面第 0–8 节保留二代高级部署流程；一代首次使用以上四步即可。一代 ROS 控制仍依赖外部 `wujihandros2` 驱动，本包的二代 ROS 驱动、网页/示教不自动兼容一代。

## 0. 安装与首次配置

从本仓库 [Releases](https://github.com/Gento-Teleoperation-Apex/wuji-hand2-glove-teleop/releases) 下载 `wuji-hand2-glove-teleop_2.3.0~test.1-1_arm64.deb`（Jetson）。PC x86_64 使用 amd64 包。

```bash
sudo apt install ./wuji-hand2-glove-teleop_2.3.0~test.1-1_arm64.deb
# 已有 ~/.venvs/wuji 且安装了相应 SDK，可跳过这一步
wuji-setup-runtime
```

deb 包含完整应用源码、命令、示例、网页、服务定义和文档；**不捆绑 ROS 系统和厂商 SDK 二进制**。`wuji-setup-runtime` 在当前用户的 `~/.venvs/wuji` 建立可读取系统 ROS 包的环境，通过 pip 安装已验证版本 `wuji-sdk==2026.8.31`、NumPy、pynput，需要联网。不要用 sudo 运行此命令。已有环境也可设置 `WUJI_PYTHON=/绝对路径/python`。ROS 功能需要 Ubuntu 22.04 的 ROS 2 Humble / Python 3.10；SDK 直控不依赖 ROS。

如果 apt 没有 ROS 软件源，先按 ROS 官方安装说明安装 Humble，再安装 ROS 功能依赖：

```bash
sudo apt install ros-humble-rclpy ros-humble-sensor-msgs ros-humble-std-msgs \
  ros-humble-std-srvs ros-humble-rmw-fastrtps-cpp ros-humble-ros2bag \
  ros-humble-rosbag2-py ros-humble-rosbag2-storage-mcap
```

高级二代入口的配置文件为 `/etc/wuji-hand2/config.env`，支持当前用户 `~/.config/wuji-hand2/config.env` 覆盖。首次四步 SDK 遥操不要求填写 SN。需要固定二代配对或录制姿态时，用扫描结果填写所用设备的 SN；不要直接照抄另一套设备：

```bash
wuji-hand2-check --scan
sudo nano /etc/wuji-hand2/config.env
# 取消注释并填写 WUJI_LEFT_SN / WUJI_RIGHT_SN
# WUJI_LEFT_GLOVE_SN / WUJI_RIGHT_GLOVE_SN
```

现有部署升级时，原 `/opt/kernelmind/wuji-hand2/config/pairing.env` 会继续读取，已有 `~/.local/share/wuji-hand2/*.json` 不在包内，不会被覆盖。配置覆盖顺序：旧配对文件 → `/etc/wuji-hand2/config.env` → 用户配置。原 `/usr/local/bin` 中指向本应用的包装脚本会备份到 `/var/backups/wuji-hand2/pre-2.2.0/` 并转到新版命令。已经运行的旧进程需要自己退出后重新启动。

## 1. 先检查手和手套连接

以下是原二代部署的地址示例，不是新设备必须使用的地址：左手 `6.6.7.110`，右手 `6.6.7.111`，左手套 `6.6.7.103`，右手套 `6.6.7.102`。按自己的实际地址检查。2.3.0 可先运行 `wuji-glove-network --apply` 自动补充手套网段；二代手本身的网络仍需可达。一代手走 USB，不需要手端 IP。以下 `wuji-hand2-check` 完整反馈检查面向二代，`--scan` 可以列出一代 USB 手。

```bash
ping -c 3 6.6.7.110
ping -c 3 6.6.7.111
ping -c 3 6.6.7.103
ping -c 3 6.6.7.102
wuji-hand2-check --scan
wuji-hand2-check --kind hands --side both
wuji-hand2-check --kind gloves --side both
# 一次检查全部，可保存报告
wuji-hand2-check --kind all --side both --output connection-check.json
# 只连接了右侧时
wuji-hand2-check --kind all --side right
```

ping 仅验证 IP 可达。SDK 检查还验证设备类型、序列号、左右侧、手的 20 个在线关节与实际位置反馈、手套 21 点骨架，以及官方 `RetargetSession` 能否输出 20 个有效关节角。报告 `errors: []` 且每项 `ok: true` 才表示所选设备通过。此检查**不使能电机、不发目标、不做标定**；手套 retarget 在电脑离线计算。

手使用 SDK 端口 `7447`，手套通常为 `50001`。发现不到设备时，检查有线网卡路由、电源、IP 冲突及防火墙；配置 SN + ADDRESS 后检查程序可尝试直接地址连接，但遥操仍依赖 SDK 正常发现。手套检查时可活动手指，确认持续产生骨架数据。先退出占用设备的其他控制程序/Studio。

本套右手套旧固件 0.10.1 曾实测：保存 IP `.101`，重启后的实际 IP 为 `.102`。不要仅因参数不同就重复改成 `.102`，这曾导致实际 `.103` 与左手套冲突。此为该设备的已观察行为，不是所有固件的通用规则。旧改 IP 示例留在源码中供参考，**不作为当前 6.6.7 网络的启动步骤**。

## 2. 用官方 SDK 基线验证手套能否控制手

这一步直接使用官方 `RetargetSession.for_hand(HandModel.WujiHand2)`：手套 21 点骨架 → SDK 20 关节角 → `JointCommand` → 以太网手。不启动 ROS 驱动。

```bash
# 无脚踏：启动后收到有效骨架与手反馈就上使能并跟随
wuji-hand2-sdk --side right --no-footkey
# 双手
wuji-hand2-sdk --side both --no-footkey
# 有脚踏：按住映射为 F7 的脚踏或键盘 F7 跟随，松开保持
wuji-hand2-sdk --side both
```

`wuji-hand2-sdk` 是本项目对官方 SDK 接口的基线封装：强制默认 SDK 用户/内置手模型，关闭应用层手指缩放、对掌与捏合增强；仍保留配对校验、F7 门控和二代手 EMA 平滑。双手模式缺少任一所选设备则报错，不静默降为单手。运行时做握拳、张手、拇指食指捏合来对照。Ctrl+C 停止并下使能已由本进程启用的手。

基于官方示例的一/二代通用入口位于 `/opt/kernelmind/wuji-hand2/retargeting/1.teleop_real.py`，命令 `wuji-teleop`，旧命令 `wujihand2-teleop-real` 保留为同一入口。2.3.0 新增 `--hand-model`、`--side` 和电机参数选项；没有 F7 门控，不接受 `--no-footkey`。它使用默认 SDK 用户和内置模型，检测到同侧多个候选会拒绝启动，不读取二代封装的 SN 配置。详见前面的四步启动与参数说明。

## 3. 使用优化后的 SDK 遥操

先 Ctrl+C 退出上一步，再运行：

```bash
# 无脚踏
wuji-hand2-tuned --side both --no-footkey
# 有脚踏：按住 F7 跟随，松开保持
wuji-hand2-tuned --side both
# 单独右手
wuji-hand2-tuned --side right --no-footkey
```

优化仍使用官方 retarget，增加小指关键点缩放、小指屈曲增益/张开偏置、拇指与邻近指尖对掌、捏合屈曲增强。默认五指缩放 `1 1 1 1 1.1`，小指增益 `0.9`、张开偏置 `0.05 rad`、对掌距离 `0.10 m`、对掌比例 `0.55`、捏合增强 `0.12 rad`。二代手的 EMA 为 `0.35`，MIT 参数 `kp=5.0` / `kd=0.15`，电流限值 `1.5 A`，与现有部署保持一致。

```bash
wuji-hand2-tuned --side right --no-footkey --default-user \
  --pinky-scale 1.1 --pinky-flex-gain 0.9 --pinky-open-bias 0.05 \
  --opposition-close 0.55 --pinch-flex-boost 0.12
# 使用已标定的 SDK 用户（名字必须在 SDK 用户库存在）
wuji-hand2-tuned --side both --user-name YOUR_CALIBRATED_USER
```

不指定用户时优化模式选择 SDK 中首个非默认用户，找不到则使用默认用户；对比基线时建议显式 `--default-user`，避免用户模型差异混入调参效果。所有参数见 `wuji-hand2-tuned --help`。手套骨架超过 1 秒没有新数据时退出控制；SDK 直控退出会下使能，不会无限使用旧骨架。

## 4. ROS 接口控制：脚踏版与无脚踏版

先退出 SDK 直控。两个终端分别运行驱动和手套发送端。

终端 A（两种版本都一样）：

```bash
wuji-hand2-driver --side both
```

终端 B（二选一）：

```bash
# 无脚踏：立即持续跟随
wuji-hand2-glove --side both --no-footkey
# 有脚踏：按住 F7 遥操，松开保持
wuji-hand2-glove --side both
```

单手时两条命令都改为 `--side right` 或 `--side left`。驱动默认待机，上使能由有效模式和关节目标触发。**不用给驱动额外加 `--no-footkey`**；脚踏/无脚踏开关由手套发送端处理。驱动的同名选项仅表示初始模式 1，不负责读键盘。ROS 发送端还可用 `--profile official` 对比未经应用层增强的官方 retarget。

F7 监听依赖 pynput 与本机图形会话（通常 X11 / DISPLAY）。脚踏必须映射为 F7；SSH 终端里打字不等于远端桌面的全局 F7。没有桌面键盘权限时使用 `--no-footkey`，或在设备本机图形终端运行。松开 F7 是**保持姿态**，不是下使能；停止时先 Ctrl+C 退出发送端，再 Ctrl+C 退出驱动下使能。

ROS 指令与反馈详见 [ROS 接口](docs/ROS.md)。`/tj/control/footkey2` 是 **Int32**，不是 Bool：0 待机保持，1 手套遥操，2 显式配置的 home，3 用户姿态，4 回放。手套指令 `/tj/hand_left2/joint_commands` 和 `/tj/hand_right2/joint_commands`；实际位置 `/tj/hand_left2/joint_states` 和 `/tj/hand_right2/joint_states`。

## 5. 不用手套，手动摆放录三种手型

退出遥操和播放器，保留匹配的驱动运行。

```bash
wuji-hand2-driver --side both   # 驱动终端，已经运行则不重复启动
wuji-both-record               # 另一个终端
# 重录已有文件（首次写入前自动备份）
wuji-both-record --overwrite
```

等显示确认下使能后，手动摆放左右手：

- **B**：先读取两只手实际位置，再上使能保持。记录候选值来自使能前。
- **X**：保存这组位置并下使能，继续摆放下一组。共三组。
- **D**：丢弃未保存候选，下使能重新摆。
- **Q / Ctrl+C**：退出示教并下使能；已经保存的组保留。

双手文件 `~/.local/share/wuji-hand2/both_hand_poses.json`；左/右单手分别为 `left_hand_poses.json`、`right_hand_poses.json`。单手使用匹配的驱动加 `wuji-left-record` / `wuji-right-record`。双手录制的每组包含两只手各 20 个关节。手动摆放前必须看到下使能成功提示。

“等候示教服务，最多 10s”是等驱动服务就绪的超时，**不是只给你 10 秒摆放或录制**。找不到服务时，确认驱动版本已更新且 `--side` 一致：双手驱动对应 `wuji-both-record`，右手驱动对应 `wuji-right-record`。不要在双手驱动上运行单手 recorder。

## 6. 手型切换：终端和网页

终端：

```bash
wuji-both-poses
# 可选两秒过渡
wuji-both-poses --transition 2
```

右手 **1 / 2 / 3**，左手 **A / S / D**，左右独立任意组合。终端播放器启动后进入双方状态 1；默认一秒平滑切换。Q/Ctrl+C 停止发送并待机保持，下使能要关闭驱动。单手入口 `wuji-right-poses` / `wuji-left-poses`。这些按键只在对应终端获得焦点时生效；SSH 使用 `ssh -t`。

网页：

```bash
# 保留双手驱动，退出终端播放器和遥操
wuji-pose-web
# 或后台运行（没有开机自启）
systemctl --user daemon-reload
systemctl --user start wuji-pose-web
```

打开 `http://主机IP:8765/`，点击“启用网页控制”，再用六个按钮或键盘选手型。当前主机可用 `http://10.121.40.182:8765/`；也可本机 `http://localhost:8765/`。

网页启用时先保持双手当前位置，点击状态才切换。Esc/“暂停切换”返回保持；页面断联 3 秒、设备反馈中断或出现其他 ROS 控制程序会暂停。关闭网页不等于下使能。一个页面获得控制权后，其他页面不能抢占。服务启动但未点击启用时不会发控制指令。

后台关闭：`systemctl --user stop wuji-pose-web`；日志：`journalctl --user -u wuji-pose-web -n 50`。前台和服务不能同时占用 8765。手型文件不足三组、SN 不符或驱动未就绪时页面会显示原因。

## 7. 录制和回放 ROS 数据

```bash
# 先启动驱动；可以同时使用 ROS 手套遥操或姿态切换
wuji-hand2-bag record ~/hand-bags/session01 --side both
# Ctrl+C 完成录制；查看内容
ros2 bag info ~/hand-bags/session01
# 退出遥操/姿态播放器，保留驱动；执行后实体手开始回放
wuji-hand2-bag play ~/hand-bags/session01 --side both --rate 0.5
```

录制 MCAP：双手实际状态、遥操/用户/回放目标、模式及触觉话题。回放只取实际关节状态并映射到 `/tj/hand_*_replay`，由工具发送模式 4；不会重播录下的模式或向原指令话题重复发数据。退出返回模式 0 保持。仅录右手时 record/play 都用 `--side right`，并运行右手驱动。

若要与现有整机遥操共同录制，将本文的状态/指令/触觉话题加入同一 rosbag recorder。不要同时运行两个模式控制器。现有 Apex 的命名空间、模式状态机、播放键还需匹配，详见 [ROS 接口与整机边界](docs/ROS.md)。本包不改写现有 Apex 安装和启动配置；独立手部 MCAP 验证不等同整机同步回放验收。

## 8. 其他小功能和故障定位

- `wujihand2-fingertip`：官方二代手指尖触觉示例；源码提供解码细节。驱动发布每指及汇总触觉；无触觉时可 `--no-tactile`。
- `--tactile-calibrate`：驱动启动时显式执行触觉清零，必须指尖完全空载；平时不要加。
- Home：默认所有便捷遥操入口 `--no-home`，不自动执行未验证的回零。高级保存/服务示例保留在 `retargeting/3.save_home.py`、`home_pose_service.py`；驱动需 `--home-pose /path/to/home_pose.json` 才允许模式 2。双手三状态功能请用 pose recorder，不要拿旧单一 home 文件替代。
- 官方手套订阅、SDK 用户管理、IK 标定、触觉标定示例在 `/opt/kernelmind/wuji-hand2/wuji_glove/`。它们的用途、运行方法见各脚本说明；标定/配置脚本会写设备或 SDK 用户数据，不属于连接检查。
- 网页打不开：检查网页进程/日志、8765 端口及主机 IP；驱动没启动只会让控制按钮不可用，不影响页面访问。
- F7 不起作用：确认踏板输出 F7，运行于可访问显示服务器的会话，或选无脚踏版。
- 手套在线但不动：先查骨架/retarget 报告，再查当前 ROS 模式和 `joint_commands`，确认对应驱动与 SN；检查是否有其他遥操进程。
- 反馈/模式跨进程不可见：统一 ROS_DOMAIN_ID、RMW_IMPLEMENTATION、ROS_LOCALHOST_ONLY；命令会读取 `/etc/apex/apex.env` 和用户配置。
- 不要以 root 运行普通控制命令：root 的 SDK 用户模型和姿态目录与 nvidia 用户不同。

## 构建与测试

```bash
./build_deb.sh 2.3.0~test.1 arm64
./build_deb.sh 2.3.0~test.1 amd64
bash tests/run_tests.sh
```

构建只需 Bash/Python 与 `dpkg-deb`，不需要实体设备；Python 载荷可跨架构打包，SDK 二进制由目标架构的 pip 安装。测试和验证范围见 [验证记录](docs/VALIDATION.md)。许可证见 [LICENSE](LICENSE)，原 SDK 示例与依赖的许可分别保留。
