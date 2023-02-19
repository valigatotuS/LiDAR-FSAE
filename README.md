# LiDAR-FSAE
LiDAR R&amp;D &amp; implementation for a formula student race car

## Requirements

- Robot: [Turtlebot3 Burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/)
- Robot software: ROS2 Foxy Fitzroy
- Computer OS: Ubuntu 20.04

## Setup

Follow setup guide from [ROBOTIS](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/), use Foxy version.

Clone this repository

Open Linux terminal
    
    export TURTLEBOT3_MODEL=burger
    ./install/setup.bash                                 # update dependencies
    ros2 pkg create --build-type ament_python avoid_cone # build package
    colcon build --packages-select avoid_cone --symlink-install # build specific package
    ros2 run avoid_cone avoid_cone                       # run builded package
    ros2 launch turtlebot3_gazebo empty_world.launch.py  # simulation 
  
Have fun

    ros2 launch turtlebot3_gazebo turtlebot3_autorace.launch.py # launch simulation in race world
    

## Sources

- [Robotis](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- Milan Velle, tb3 scripts
- Jan Lemeire, tb3
- [AMZ Driverless FSSIM](https://github.com/AMZ-Driverless/fssim)
  
  
  
