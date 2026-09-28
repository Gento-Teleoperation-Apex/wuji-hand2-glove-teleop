# ROS 接口和录制回放

所有二代手的主接口固定在 `/tj`。当前版本不支持仅改 `APEX_ROS_NAMESPACE` 就移动手部话题；该变量只用于寻找 Apex 控制状态/播放键。需要其他手部命名空间时，须同时修改发送端、驱动、pose 和录制路由，不要只改其中一处。

## 模式

话题 `/tj/control/footkey2`，类型 `std_msgs/msg/Int32`：

- 0：待机，不接收新的运动目标；已启用的手持续保持最后目标。
- 1：遥操，接收 `/tj/hand_left2/joint_commands` 和 `/tj/hand_right2/joint_commands`。兼容旧 flat `/tj/hand_left_cmd`、`/tj/hand_right_cmd`；不要同时重复发布两条路由。
- 2：home，仅驱动显式传入有效 `--home-pose` 文件时允许。不是急停，也不是下使能。
- 3：用户姿态，接收 `/tj/hand_left_user`、`/tj/hand_right_user`。
- 4：回放，接收 `/tj/hand_left_replay`、`/tj/hand_right_replay`。

命令类型均为 `sensor_msgs/msg/JointState`，`position` 为严格 20 个有限弧度值，按固件五指×四关节顺序；规范 `name` 为 j0…j19。驱动消费 position 的顺序，不根据 name 自动重排；发布端应规范排序。模式匹配且收到有效命令后才上使能。

自定义控制器应在拿到新鲜状态后构造目标，定时发目标和对应模式，退出时发模式 0。需要下使能时关闭驱动；示教工具通过专用服务下使能。不要用全零数组“试一下是否能动”。

## 反馈与触觉

- `/tj/hand_left2/joint_states`、`/tj/hand_right2/joint_states`：JointState，j0…j19，position/velocity/effort；有效完整状态约 50 Hz。
- `/tj/hand_left2/tactile`、`/tj/hand_right2/tactile`：Float32MultiArray，每只手按 thumb/index/middle/ring/pinky 顺序，各 6 项 `[fx,fy,fz,temperature,contacts,max_force]`。
- `/tj/hand_{left,right}2/tactile/{thumb,index,middle,ring,pinky}`：Float32MultiArray，该指各点 `[fx,fy,fz]×N`；力单位 N。硬件无对应触觉传感器时不发布，并给出日志。

在外部终端使用 ros2 CLI 前加载同一环境：

```bash
source /opt/kernelmind/wuji-hand2/config/env.sh
ros2 topic echo /tj/hand_right2/joint_states --once
ros2 topic hz /tj/hand_right2/joint_states
ros2 topic echo /tj/control/footkey2 --once
ros2 topic info /tj/hand_right2/joint_commands --verbose
```

## 手动示教服务

双手驱动：`/tj/hands2/teach_hold`（std_srvs/SetBool）、`/tj/hands2/teach_finish`（std_srvs/Trigger）。单手驱动为 `/tj/hand_left2/...` 或 `/tj/hand_right2/...`。

teach_hold(false) 进入示教并下使能；true 同时读取所选手当前位置再上使能保持，响应 message 返回 JSON 快照；teach_finish 下使能并结束会话。出现其他模式发布端时拒绝示教。推荐通过 recorder 的 B/X/D/Q 流程调用，避免手动服务调用和录制状态不一致。

## 独立手部 MCAP

`wuji-hand2-bag record` 默认录制双手状态、三类目标、模式和触觉。目标目录必须不存在。`play` 检查 bag 中所选双方状态不为空，发布模式 4，仅选关节状态话题映射到 replay 路由；不播放原模式、触觉、遥操命令。默认按原时间 1 倍速，可 `--rate 0.5`。重复回放前保持驱动运行，停止其他遥操/pose 发送端。

本工具重放实际关节位置轨迹；不用于回放手套原始骨架后重新 retarget。bag 含两只手的同期状态和时间戳，但不含相机、机械臂，除非你另外将它们加入统一 recorder。

## 与现有整机遥操共同录制

将 `integration/record_topics.txt` 中的手部话题加入现有同一个 rosbag recorder，沿用其时钟、相机与机械臂话题；录制本身只订阅，可与正常 ROS 手部遥操共存。不要直接覆盖现有 Apex 配置。现有 SDK 直控不发布 ROS 关节状态，若要与整机 bag 合并请使用 ROS 控制路径。

驱动提供高级 `--apex-mode`，按已有接口假定 Apex 状态 0 idle、1 teleop、2 planner、3 playback；planner 不映射为手部 home。驱动从 `WUJI_APEX_STATE_TOPIC` 读 Int32，回放还要求 `WUJI_PLAYBACK_KEY_TOPIC` 的 Bool 为 true 且 1 秒内更新；驱动自己发布手部模式。在这种模式下，手套端设置 `WUJI_EXTERNAL_MODE=1`，跟随手部模式而不竞争发布。此处 F7 由上层状态机管理，不能再同时运行独立手部 bag/pose 模式发布器。

这些是适配接口，**不能保证任意版本 Apex 的状态编号、话题名或播放键语义一致**。必须对照实际整机 graph；本次打包不改动、不启动 Apex，也没有进行实体整机同步回放验收。独立遥操和姿态请使用默认模式，不加 `--apex-mode`。
