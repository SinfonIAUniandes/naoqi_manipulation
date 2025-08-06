# NAOqi Manipulation for ROS 2

This ROS 2 package provides a bridge to the manipulation, posture, and motion safety functionalities of SoftBank Robotics' Pepper and NAO robots. It exposes functionalities from NAOqi's `ALMotion` and `ALRobotPosture` modules as ROS 2 services.

## Features

*   **Posture Control**: Command the robot to go to predefined postures like "Stand", "Sit", "SitRelax", and "Rest".
*   **Stiffness Control**: Set stiffness for individual joints or groups, and toggle "Smart Stiffness" mode.
*   **Hand Control**: Open and close the robot's hands.
*   **Breathing Animation**: Enable or disable the idle breathing movements for different body parts.
*   **Motion Safety**:
    *   Configure tangential and orthogonal security distances for obstacle avoidance.
    *   Enable or disable collision protection for the arms.
    *   A service to enable a set of default safety parameters.
*   **Arm Movement**: Enable or disable arm motion during navigation tasks.

## Dependencies

*   `rclpy`
*   `std_srvs`
*   `naoqi_utilities_msgs`: Contains the custom service definitions used by this node.

## How to Run the Node

To start the node, you need to provide the IP address and port of the robot.

```bash
ros2 run naoqi_manipulation naoqi_manipulation_node --ros-args -p ip:=<robot_ip> -p port:=<robot_port>
```

For example:
```bash
ros2 run naoqi_manipulation naoqi_manipulation_node --ros-args -p ip:=192.168.1.101 -p port:=9559
```

## ROS 2 API

All services are exposed under the node's namespace (`/naoqi_manipulation_node/` by default).

### Services

#### Posture and Stiffness
*   **`~/go_to_posture`** ([naoqi_utilities_msgs/srv/GoToPosture](naoqi_utilities_msgs/srv/GoToPosture.srv))  
    Commands the robot to adopt a predefined posture (e.g., "Stand", "Sit").
*   **`~/set_stiffnesses`** ([naoqi_utilities_msgs/srv/SetStiffnesses](naoqi_utilities_msgs/srv/SetStiffnesses.srv))  
    Sets the stiffness for a list of joints.
*   **`~/toggle_smart_stiffness`** ([std_srvs/srv/SetBool](https://docs.ros2.org/foxy/api/std_srvs/srv/SetBool.html))  
    Enables or disables the smart stiffness feature.

#### Manipulation and Animation
*   **`~/set_open_close_hand`** ([naoqi_utilities_msgs/srv/SetOpenCloseHand](naoqi_utilities_msgs/srv/SetOpenCloseHand.srv))  
    Opens or closes a specified hand ("LHand" or "RHand").
*   **`~/toggle_breathing`** ([naoqi_utilities_msgs/srv/SetBreathing](naoqi_utilities_msgs/srv/SetBreathing.srv))  
    Enables or disables breathing animation for a joint group (e.g., "Arms", "Body").
*   **`~/set_move_arms_enabled`** ([naoqi_utilities_msgs/srv/SetMoveArmsEnabled](naoqi_utilities_msgs/srv/SetMoveArmsEnabled.srv))  
    Allows or prevents arm movement during base navigation.

#### Motion Safety
*   **`~/set_tangential_security_distance`** ([naoqi_utilities_msgs/srv/SetSecurityDistance](naoqi_utilities_msgs/srv/SetSecurityDistance.srv))  
    Sets the tangential security distance for obstacle avoidance.
*   **`~/set_orthogonal_security_distance`** ([naoqi_utilities_msgs/srv/SetSecurityDistance](naoqi_utilities_msgs/srv/SetSecurityDistance.srv))  
    Sets the orthogonal security distance for obstacle avoidance.
*   **`~/toggle_arms_collision_protection`** ([std_srvs/srv/SetBool](https://docs.ros2.org/foxy/api/std_srvs/srv/SetBool.html))  
    Enables or disables collision protection for the arms.
*   **`~/enable_default_security`** ([std_srvs/srv/Trigger](https://docs.ros2.org/foxy/api/std_srvs/srv/Trigger.html))  
    Resets security distances and enables arm collision protection.

## Usage Example

To make the robot go to the "Sit" posture:
```bash
ros2 service call /naoqi_manipulation_node/go_to_posture naoqi_utilities_msgs/srv/GoToPosture "{posture_name: 'Sit'}"
```

To open the robot's left hand:
```bash
ros2 service call /naoqi_manipulation_node/set_open_close_hand naoqi_utilities_msgs/srv/SetOpenCloseHand "{hand: 'LHand', state: true}"
```

To turn off stiffness for the head:
```bash
ros2 service call /naoqi_manipulation_node/set_stiffnesses naoqi_utilities_msgs/srv/SetStiffnesses "{joint_names: ['Head'], stiffnesses: [0.0]}"
```