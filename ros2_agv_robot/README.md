## **一.项目介绍**
ros2_agv_robot is a ros2 chassis package for embossed two-arm robots.The functions implemented are:

1. Request robot information
2. Subscribe to robot information
3. Subscribe to the bot status
4. Subscription mode information
5. Subscribe to position speed
6. Subscribe to battery information
7. Subscribe to scene information
8. Subscribe to task progress
9. Subscribe to device status
10. Subscribe to run status
11. Task execution
12. Wait for the task execution result
13. Task execution request
14. Callbacks in task execution process
15. Perform predefined tasks
16. Stepping process callback
17. Stepping execution request
18. Wait for stepping execution results

There are also demo test cases for various functions



## **2. File structure**
```bash
├── ros2_agv_robot           # 底盘功能包
│   ├── CMakeLists.txt       # 编译规则文件
│   ├── lib                  # ros2接口安装包
│   ├── package.xml          # 定义功能包属性文件
│   └── src                  # 测试代码目录
## **3. Compilation method**
1. Install the chassis ros2 interface installation package
cd ~/ros2_agv_robot/lib
sudo ./ros-foxy-woosh-robot-agent_0.0.1-0focal_arm64.run
2. Compile and test demo
```bash
cd ros2_ws/     # 进入工作空间目录
colcon build --packages-select ros2_agv_robot
```

## **4. Run command**
- 1. Start the chassis connection node

```bash
ros2 run woosh_robot_agent agent --ros-args -r __ns:=/woosh_robot -p ip:="169.254.128.2"
or
ros2 run woosh_robot_agent agent --ros-args -r __ns:=/woosh_robot
```

- 2. Use cases for starting function package function:

```bash
ros2 run ros2_agv_robot monitor  				          	        #Execute the monitor node to obtain robot information
ros2 run ros2_agv_robot executeask --ros-args -p mark_no:=7613B5D # When a certain storage bit is reached (7613B5D), you need to create a map in advance, the robot initializes it, and execute after creating the storage bit.
ros2 run ros2_agv_robot exec_pre_task --ros-args -p id:=1735291765 # Execute the task (1735291765), create the map in advance, initialize the robot, and execute after creating the task
ros2 run ros2_agv_robot stepctrl # step control
```