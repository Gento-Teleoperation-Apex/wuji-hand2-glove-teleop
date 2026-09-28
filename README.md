# Wuji Hand 2 双手手套遥操 · 2.2.0

从设备检查到 SDK 直控、优化遥操、ROS 控制，再到手动示教、手型切换和网页。本版整理自 Ubuntu 22.04 / Jetson AGX Orin 的实际部署。面向二代以太网灵巧手；左右单手、双手均可选。

安装不会连接设备、上使能、启动驱动或配置开机自启。所有会运动的命令都由操作员手动执行。一个时刻仅使用一个控制入口：SDK 直控、ROS 遥操、姿态播放或 bag 回放。

## 0. 安装与首次配置

从本仓库 [Releases](https://github.com/Gento-Teleoperation-Apex/wuji-hand2-glove-teleop/releases) 下载 `wuji-hand2-glove-teleop_2.2.0-1_arm64.deb`（Jetson）。PC x86_64 使用 amd64 包。

```bash
sudo apt install ./wuji-hand2-glove-teleop_2.2.0-1_arm64.deb
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

配置文件 `/etc/wuji-hand2/config.env`，也支持当前用户 `~/.config/wuji-hand2/config.env` 覆盖。用扫描结果填写四个序列号；不要直接照抄另一套设备的 SN：

```bash
wuji-hand2-check --scan
sudo nano /etc/wuji-hand2/config.env
# 取消注释并填写 WUJI_LEFT_SN / WUJI_RIGHT_SN
# WUJI_LEFT_GLOVE_SN / WUJI_RIGHT_GLOVE_SN
```

现有部署升级时，原 `/opt/kernelmind/wuji-hand2/config/pairing.env` 会继续读取，已有 `~/.local/share/wuji-hand2/*.json` 不在包内，不会被覆盖。配置覆盖顺序：旧配对文件 → `/etc/wuji-hand2/config.env` → 用户配置。原 `/usr/local/bin` 中指向本应用的包装脚本会备份到 `/var/backups/wuji-hand2/pre-2.2.0/` 并转到新版命令。已经运行的旧进程需要自己退出后重新启动。

## 1. 先检查手和手套连接

当前这套设备的通信地址：左手 `6.6.7.110`，右手 `6.6.7.111`，左手套 `6.6.7.103`，右手套 `6.6.7.102`。控制电脑有线接口须能路由到 `6.6.7.0/24`。软件不自动修改网卡或设备 IP。

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

原始官方示例原文也保存在 `/opt/kernelmind/wuji-hand2/retargeting/1.teleop_real.py`，命令 `wujihand2-teleop-real`。**原始示例没有 `--side`、footkey 门控或本项目的 SN 配对保护，会自动驱动发现的匹配设备**；日常验证优先使用上面的基线入口。原始示例与新增封装有明确区别，不能给原始示例追加本文封装的选项。

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
./build_deb.sh 2.2.0 arm64
./build_deb.sh 2.2.0 amd64
bash tests/run_tests.sh
```

构建只需 Bash/Python 与 `dpkg-deb`，不需要实体设备；Python 载荷可跨架构打包，SDK 二进制由目标架构的 pip 安装。测试和验证范围见 [验证记录](docs/VALIDATION.md)。许可证见 [LICENSE](LICENSE)，原 SDK 示例与依赖的许可分别保留。
