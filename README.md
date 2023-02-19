# LiDAR-FSAE
LiDAR R&amp;D &amp; implementation for a formula student race car


## Setup

Follow setup guide from [ROBOTIS](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/), use Foxy version.

Clone this repository

Open Linux terminal
    ./install/setup.bash                                 # update dependencies
    ros2 pkg create --build-type ament_python avoid_cone # build package
    colcon build --packages-select avoid_cone --symlink-install # build specific package
    ros2 run avoid_cone avoid_cone                       # run builded package
    ros2 launch turtlebot3_gazebo empty_world.launch.py  # simulation 
  
Have fun

## Sources 

- [Robotis](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- Milan Velle, tb3 scripts
- Jan Lemeire, tb3
  
  
  
