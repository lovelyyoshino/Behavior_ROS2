# Behavior_Ros2
Based on the behavior tree, the repository has written a general implementation of some common instructions in Ros2 humble in the behavior tree. Including ros2 run, ros2 topic, etc., the purpose is to integrate it into the behavior tree to achieve multi-function orchestration in the behavior tree.

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
