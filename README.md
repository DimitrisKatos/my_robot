<<<<<<< HEAD
# Differential drive controller robot with two sensors.
In this tutorial we will build a differential drive controller robot using xacro properties. We will add two sensors, a camera, lidar and finally we control the robot model by adding differential drive plugin. 

Totally, you will learn the following:
- how to add lidar to robot and adding a new mesh file.
- how to add a camera to robot.
- how to add differential drive control to the robot.
- Create a simple algorithm for obsackle avoidance.

You can find a presentation that will guide you through this tutorial. In the presentation, you can find information on how to build this robot.
- [Create a differential drive controller robot](https://docs.google.com/presentation/d/1JCBwxNok0eC-tr-dRsIzfJUy-XPxEjk6/edit?usp=drive_web&ouid=106628092038381749227&rtpof=true) Greek language.
- Create a differential drive controller robot. English language (comming soon).

You can find a project in the following link:
- [Project]() Greek language.
- [Project]() English language( Coming soon).

## Create, build and setup a new package.
Create a new ROS 2 package for the differential drive control robot
```
cd ~/ros2_ws/src
ros2 pkg create --build-type ament-python my_robot
```

Create some folders to the package for better organization.
```
cd ~/ros2_ws/src
mkdir launch urdf meshes worlds
```

After creating the package and the folders, you need to modify the setup.py file of the package.
Add the following modules.
```py
from glob import glob
import os
```
In the data_files list of the script add the following.
```py
        (os.path.join('share',package_name,'launch'),
         glob(os.path.join('launch','*.launch.py'))),
        (os.path.join('share',package_name,'urdf'),
         glob(os.path.join('urdf','*.xacro'))),
         (os.path.join('share',package_name,'urdf'),
         glob(os.path.join('urdf','*.gazebo'))),
        (os.path.join('share',package_name,'worlds'),
         glob(os.path.join('worlds','*.world'))),
        (os.path.join('share',package_name,'meshes'),
         glob(os.path.join('meshes','*.dae'))),
```

The next step is building the package using the colcon tool.
```
cd ~/ros2_ws/
colcon build --packages-select my_robot
```

=======
## Two wheel robot with camera, lidar and differential drive controller.
>>>>>>> 8fde00cbcd158ef666b3237f1276b8624de68c82
