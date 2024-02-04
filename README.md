# chatgpt_ros_cpp

ROS 2 service for ChatGPT API written in C++

[![ci_humble](https://github.com/MrBearing/chatgpt_ros_cpp/actions/workflows/ci_humble.yaml/badge.svg)](https://github.com/MrBearing/chatgpt_ros_cpp/actions/workflows/ci_humble.yaml)

### Locate package in workspace

```bash
mkdir -p ~/ws_ros2/src
cd ~/ws_ros2/src
git clone git@github.com:MrBearing/chatgpt_ros_cpp.git
```

### Run script to install dependencies

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/ws_ros2/src/MG400_ROS2
./setup.bash
exec -l $SHELL
```

## Launch node and rqt_service

This command launch chatgpt_ros_cpp_node with rqt_service

``` :bash
source ~/ws_ros2/install/setup.bash
ros2 launch chatgpt_ros_cpp_bringup chat.launch.py api_key:="YOUR_OPEN_AI_API_KEY"

```


## Reference

- [OpenAI api reference](https://platform.openai.com/docs/api-reference/chat/create)