# Behavior_Ros2_Humble
Based on the behavior tree, the repository has written a general implementation of some common instructions in Ros2 Humble in the behavior tree. Including ros2 run, ros2 topic, etc., the purpose is to integrate it into the behavior tree to achieve multi-function orchestration in the behavior tree.

| 功能 | 描述               | 文件   | 
|----------|--------------------|--------|
| autodock_client   | Apriltag码定位    | autodock_client.hpp   | 
| interrupt_event   | 中断    | interrupt_event.hpp   | 
| nav2_client   | 导航    | nav2_client.hpp   | 
| publish_topic   | 发布某一话题    | publish_topic.hpp   | 
| send_goal   | 发布目标点    | send_goal.cpp   | 
| snapshot_client   | 相机拍照    | snapshot_client.hpp   | 
| start_run_command   | 执行ros2 run    | start_run_command.hpp   | 
| start_usb_cam   | 执行usb相机拍照    | start_usb_cam.hpp   | 
| teleop_event   | cmd_vel控制速度    | teleop_event.hpp   | 
| wait_node   | 睡眠阻塞    | wait_node.hpp   | 

## Build Behavior_ros2:

```bash
$ cd ~/Behavior_ros2/
$ source /opt/ros/humble/setup.bash
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
## Nake self node:
If you want to customize some other behavior tree nodes, please write an hpp or cpp file, and then register it in bt_ros2.cpp, I have shown the example.

Step 1. Make .hpp file.

```cpp
#pragma once
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>
class PublishTopic : public BT::SyncActionNode
{
public:
    PublishTopic(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("lift_control");
        command_ = "ros2 topic pub /lift_control std_msgs/msg/Int32 \"{data: 1}\" --rate 10 --times 20";
    }
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("command") };
    }
    virtual BT::NodeStatus tick() override
    {
        std::string custom_command;
        if (getInput<std::string>("command", custom_command)) {
            command_ = custom_command;
        }
        RCLCPP_INFO(node_->get_logger(), "Executing command: %s", command_.c_str());
        int result = std::system(command_.c_str());
        if (result == 0) {
            RCLCPP_INFO(node_->get_logger(), "publish topic started successfully.");
            std::this_thread::sleep_for(std::chrono::seconds(3));
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to start publish topic: %s", command_.c_str());
            return BT::NodeStatus::FAILURE;
        }
    }
private:
    rclcpp::Node::SharedPtr node_;
    std::string command_;
};
```

Step 2. Registers the behavior tree node.
```cpp
#include "publish_topic.hpp"
factory.registerNodeType<PublishTopic>("PublishTopic");
```

Step 3. Write execution flow.
```xml
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence>
<!--  设置目标点在当前点向-y方向1米处  -->
            <SetBlackboard output_key="Goal_a" value="0.0;-5.0;0.0;1.0"/>
            <SetBlackboard output_key="Goal_b" value="0.0;0.0;0.0;1.0"/>
            <Sequence>
<!--  移动机器人到目标位置  -->
                <SubTree ID="MoveRobot" target="Goal_a"/>
                    <AutodockClient action="start"/>
                    <AutodockClient action="cancel"/>
            </Sequence>
            <Sequence>
                <PublishTopic command="ros2 topic pub /lift_control std_msgs/msg/Int32 '{data: 1}' --rate 10 --times 20"/>
                <WaitNode/>
                <StartUsbCam/>
                <WaitNode/>
                <PublishTopic command="ros2 topic pub /lift_control std_msgs/msg/Int32 '{data: 0}' --rate 10 --times 20"/>
            </Sequence>
            <Sequence>
<!--  移动机器人到目标位置  -->
                <SubTree ID="MoveRobot" target="Goal_b"/>
                <AutodockClient action="start"/>
                <AutodockClient action="cancel"/>
            </Sequence>
<!--  执行对接操作  -->
        </Sequence>
    </BehaviorTree>
    <BehaviorTree ID="MoveRobot">
        <Sequence name="SetGoal">
<!--  导航客户端发送目标点  -->
            <Nav2Client goal="{target}"/>
        </Sequence>
    </BehaviorTree>
</root>
```
