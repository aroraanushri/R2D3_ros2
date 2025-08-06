## **1. Project introduction**
ros2_rm_robotIt is a ros2 double-arm package for the embossed double-arm lifting robot.The functions implemented are：
Current test: The controller version is `1.6.5`

1. moveit2 control robot arm

There are also demo test cases for various functions

## **2. File structure**
```bash
├── dual_rm_moveit_config        # Double-arm lifting Moveit configuration function pack
│   ├── dual_rm_65b_moveit_config
│   └── dual_rm_75b_moveit_config
├── dual_rm_control                        # Double-arm controller function pack
│   ├── CMakeLists.txt
│   ├── doc                                # Store information documents
│   ├── include
│   ├── launch
│   ├── package.xml
│   ├── README_CN.md
│   ├── README.md
│   └── src
├── dual_rm_driver                         # Double-arm drive function pack
│   └── rm_driver  
│       ├── CMakeLists.txt
│       ├── config
│       ├── doc
│       ├── include
│       ├── launch
│       ├── lib                            # Store library files
│       ├── package.xml
│       └── src
├── dual_rm_description           # Double-arm robot description function pack
│   ├── CMakeLists.txt
│   ├── config
│   ├── launch
│   ├── meshes                             # Store model meshes files
│   ├── package.xml
│   ├── README_CN.md
│   ├── README.md
│   └── urdf                               # Store urdf files
├── dual_rm_install                        # Install dependency package
│   ├── CMakeLists.txt
│   ├── doc
│   ├── package.xml
│   └── scripts                            # Store script files
└── dual_rm_ros2_interfaces
    └── rm_ros_interfaces
        ├── CMakeLists.txt
        ├── msg                            # Store msg files
        ├── package.xml
        ├── README_CN.md
        └── README.md
```

## **3. Compilation method**
```bash
cd rm_dual_arm_robot_ros2/     # Enter the workspace directory
colcon build --packages-select rm_ros_interfaces dual_rm_65b_description dual_rm_75b_description rm_driver dual_rm_control dual_rm_65b_moveit_config dual_rm_75b_moveit_config  
```

## **4. Run commands**

```

Moveit2 control

- Start the overall ros2 function package launch

​```bash
cd rmc_aida_l_ros2/ 
source install/setup.bash                              # Reload the workspace environment variables
ros2 launch dual_rm_65b_moveit_config demo.launch.py   # Start Moveit Control
```

Drag the end in rviz2 and click to plan and execute, the robotic arm can plan normally.

Moveit  C++ API Control

```sh
# Joint control(After starting the moveit controls the real robot arm, start the following program)
ros2 launch dual_rm_moveit_demo rm_65_moveit2_fk.launch.py
# Cartesian space control
ros2 launch dual_rm_moveit_demo rm_65_moveit2_ik.launch.py
```

Gazebo simulation (65b)

cd rm_dual_arm_robot_ros2/ 
source install/setup.bash                                         # Reload the workspace environment variables
ros2 launch dual_rm_gazebo dual_rm_65b_gazebo.launch.py           # Start the gazebo simulation environment and load the robot model
ros2 launch dual_rm_65b_moveit_config demo.launch.py              # Start the Moveit control simulation robot arm


Gazebo simulation (75b)

cd rm_dual_arm_robot_ros2/ 
source install/setup.bash                                         # Reload the workspace environment variables
ros2 launch dual_rm_gazebo dual_rm_75b_gazebo.launch.py           # Start the gazebo simulation environment and load the robot model
ros2 launch dual_rm_75b_moveit_config demo.launch.py              # Start the Moveit control simulation robot arm

