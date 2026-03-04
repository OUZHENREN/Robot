
# 艾利特 CS625 机械臂 ROS 2 项目 (Elite CS625 ROS 2 Project)

本项目是用于控制和可视化艾利特（Elite）CS625协作机器人的ROS 2工作空间。该项目基于官方驱动，并进行了多项稳定性和易用性优化，以解决在实际部署中遇到的连接和鲁棒性问题。

---

## 1. 系统环境与配置 (System Environment)

以下是本项目成功运行并验证的关键环境配置。

| 项目 (Item)             | 版本 / 配置 (Version / Configuration)              | 备注 (Notes)                                   |
| ----------------------- | ---------------------------------------------------- | ---------------------------------------------- |
| **操作系统 (OS)**       | Ubuntu 22.04 LTS                                     | 在 VMware 虚拟机中运行                         |
| **ROS 2 版本**          | Jazzy Jalisco (Desktop Install)                      |                                                |
| **机器人型号 (Robot)**    | Elite CS625                                          |                                                |
| **虚拟机软件 (VM)**     | VMware Virtual Platform                              |                                                |
| **Git 客户端**          | `git version 2.34.1` 或更高                          | 用于版本控制                                   |

---

## 2. 网络设置 (Network Configuration)

网络配置是本项目正常运行的**最关键环节**。

- **虚拟机网络模式**: **桥接模式 (Bridged Mode)**
  - 必须设置为桥接模式，让虚拟机获得与物理机和机器人处于同一网段的独立IP地址。

- **IP 地址分配**:
  - **机器人控制器IP**: `192.168.1.200` (必须为静态IP)
  - **电脑/虚拟机IP**: `192.168.1.102` (或`192.168.1.xxx`网段内的任何其他地址)
  - **子网掩码**: `255.255.255.0` (即 `/24`)

- **物理连接**:
  - 使用网线将装有虚拟机的电脑与机器人控制器直接连接，或连接到同一台路由器/交换机下。

- **连接验证**:
  - 在Ubuntu终端中，使用 `ping 192.168.1.200` 命令，必须能收到回复 (0% packet loss)，证明网络物理连接和IP设置正确。

---

## 3. 安装与部署 (Installation & Deployment)

请按照以下步骤从零开始部署本项目。

1.  **克隆本仓库**
    ```bash
    cd ~
    git clone https://github.com/addission/elite-cs625-ros2-project.git elite_ros_ws
    ```

2.  **安装依赖并编译**
    ```bash
    # 进入工作空间目录
    cd ~/elite_ros_ws

    # 自动安装所有依赖包
    sudo apt update
    rosdep install --from-paths src -y --ignore-src

    # 编译工作空间
    colcon build
    ```

3.  **配置桌面启动器**
    为了方便使用，可以将项目内提供的快捷方式复制到桌面。
    ```bash
    cp ~/elite_ros_ws/desktop_files/CS625机器人启动器.desktop ~/Desktop/
    chmod +x ~/Desktop/CS625机器人启动器.desktop
    ```

---

## 4. 使用方法 (Usage)

完成安装部署后，直接双击桌面上的 **"CS625机器人启动器"** 图标即可启动整个系统。

该启动器 (`cs625_launcher.sh`) 经过特殊优化，会自动完成以下操作：
1.  **自动清理**: 强制杀死所有旧的ROS节点，解决“失败后无法重连”的问题。
2.  **延时等待**: 在启动前等待5秒，确保机器人控制器服务已完全就绪，提高首次连接成功率。
3.  **加载环境并启动**: 自动加载ROS环境并执行 `ros2 launch` 命令。

好的，没问题。将这个详尽的架构图整理成适合 `README.md` 的 Markdown 格式，既要保持信息完整，又要美观易读。

下面是为您精心排版的 Markdown 代码，您可以直接复制并粘贴到您的 `README.md` 文件中。

---


# CS625 机器人控制项目架构

本文档详细描述了 CS625 机器人控制项目的全链路软件架构，从用户启动入口到与机器人硬件的底层通信。

## 核心架构图

整个系统通过一个桌面启动器统一管理，该启动器执行一个主脚本，负责加载环境、启动所有必要的 ROS 2 节点，并最终运行机器人驱动。
```txt
/home/yff/Desktop/CS625机器人启动器.desktop  (用户入口: 双击启动)
└── Exec ─> /home/yff/elite_ros_ws/cs625_launcher.sh [✓ 核心启动脚本]
              │
    .---------'---------------------------------------------------------------------------------.
    │                                                                                         │
    ├─[步骤1] 清理旧进程 (pkill ros2, killall ...)                                              │
    │                                                                                         │
    ├─[步骤2] 加载环境 (source /opt/ros/jazzy/setup.bash, source ~/elite_ros_ws/install/setup.bash) │
    │                                                                                         │
    ├─[步骤3] 启动自定义Python节点 (后台运行 `ros2 run tcp_bridge <node_name> &`)              │
    │   │                                                                                     │
    │   '------> (节点定义) --> /home/yff/elite_ros_ws/src/tcp_bridge/ [✓ ROS Python包]
    │           ├─ package.xml  [✓ 已确认] (声明依赖: rclpy, geometry_msgs, sensor_msgs, python3-scipy)
    │           │
    │           └─ setup.py     [✓ 已确认] (定义`console_scripts`入口, 将节点名链接到Python文件)
    │               ├─ 'tcp_server = tcp_bridge.tcp_server:main'
    │               │   └─ ros2 run ... ──> .../tcp_bridge/tcp_server.py [✓ 关键: 数据入口]
    │               │
    │               ├─ 'robot_commander = tcp_bridge.robot_commander:main'
    │               │   └─ ros2 run ... ──> .../tcp_bridge/robot_commander.py [✓ 关键: 硬件接口]
    │               │
    │               ├─ 'pose_to_tf_broadcaster = tcp_bridge.pose_to_tf_broadcaster:main'
    │               │   └─ ros2 run ... → .../tcp_bridge/pose_to_tf_broadcaster.py [✓ 辅助: TF发布]
    │               │
    │               └─ 'pose_sender_node = tcp_bridge.pose_sender_node:main'
    │                   └─ ros2 run ... ──> .../tcp_bridge/pose_sender_node.py [✓ 工具: 独立测试]
    │
    └─[步骤4] 启动机器人主驱动与RViz (前台运行)
        │
        └─ ros2 launch eli_cs_robot_driver elite_control.launch.py ...
            │
            ├─[加载] 机器人模型 (URDF)
            │   ├─ 模板: .../eli_cs_robot_description/urdf/cs.urdf.xacro
            │   └─ 参数: .../config/cs625/{joint_limits.yaml, ...}
            │
            ├─[加载] ROS 2 控制器配置
            │   └─ 配置: .../eli_cs_robot_driver/config/elite_cs_controllers.yaml
            │
            ├─[加载] RViz 可视化配置
            │   └─ 配置: .../eli_cs_robot_description/rviz/view_robot.rviz
            │
            └─[执行] 核心驱动与工具
                ├─ eli_ros2_control_node (硬件接口节点)
                │   └─ 插件: libeli_cs_hardware_interface_plugin.so
                ├─ robot_state_publisher (TF发布器)
                ├─ controller_manager/spawner (控制器加载器)
                └─ rviz2 (可视化工具)
```

## 数据流向

项目核心的控制数据流清晰地分为三步：接收外部指令、在 ROS 2 内部进行中继、最终发送给机器人硬件。

```text
(外部视觉/控制) ---TCP(port 9999)--> [tcp_server.py] --ROS Topic (/target_pose)--> [robot_commander.py] --TCP(port 60000)--> (机器人硬件)
                                                                │
                                                                '--ROS Topic (/target_pose)--> [pose_to_tf_broadcaster.py] --TF--> (RViz可视化)
```

## 组件说明

### 活跃组件

-   `tcp_server.py`: **数据入口节点**。监听 `9999` 端口，接收外部系统（如视觉算法）发送的 TCP 数据，并将其解析为 `PoseStamped` 消息发布到 `/target_pose` 话题。
-   `robot_commander.py`: **硬件接口节点**。订阅 `/target_pose` 话题，获取目标位姿；同时订阅 `/joint_states` 话题，获取机器人当前状态。它负责将目标位姿转换为机器人能识别的指令格式，并通过 `60000` 端口的 TCP 连接发送给机器人控制器。
-   `pose_to_tf_broadcaster.py`: **可视化辅助节点**。订阅 `/target_pose` 话题，并将该位姿发布为 TF 坐标变换，方便在 RViz 中观察目标点的位置。
-   `pose_sender_node.py`: **独立测试工具**。用于在没有外部视觉系统时，手动发布一个固定的目标位姿到 `/target_pose` 话题，以测试后续节点的行为。

### 废弃组件

-   `vision_bridge/src/tcp_pose_receiver.cpp`: [❌ **确认废弃**]
    -   **原因**: 该 C++ 节点的功能已被 Python 节点 `tcp_server.py` 完全取代。
    -   **证据**: 当前的主启动脚本 `cs625_launcher.sh` 不再调用此节点，并且在脚本开头包含了 `pkill tcp_pose_receiver` 命令，明确表示这是一个需要被清理的旧进程。


---

### 使用说明

1.  **复制**: 将上面 ````markdown` 和 ```` 之间的所有内容完整复制。
2.  **粘贴**: 打开您 GitHub 仓库中的 `README.md` 文件（或在本地编辑器中打开），将复制的内容粘贴进去。
3.  **保存和提交**: 保存文件，然后 `git commit` 和 `git push`。

这样，您的 GitHub 项目主页就会展示一个非常专业、清晰的架构图。
