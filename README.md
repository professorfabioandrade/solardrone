<h1 align="center">
    <img alt="Solar Drone Project" ttle="Solar Drone Project" src="https://dp9eps5gd5xd0.cloudfront.net/image-handler/ts/20230104063822/ri/750/src/images/Article_Images/ImageForArticle_568_16728323018456988.jpg" />
    <p>Solar Drone Project</p>
</h1>

<p align="center">
  <a href="#about-the-project">About the Project</a>&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
  <a href="#packages-description">Packages description</a>&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
  <a href="#notion-documentation">Notion Documentation</a><br />
  <a href="#dependencies">Dependencies</a>&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
  <a href="#how-to-run">How to Run</a>&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
  <a href="#troubleshooting">Troubleshooting</a>&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
  <a href="#features">Features</a>&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
  <a href="#References">References</a>
</p>

<br />

## About the Project

This github contains the codes that run the missions for the autonomous solar panel plant inspection with drones.

All the codes were written in ROS2 framework and are available in [this branch](https://github.com/professorfabioandrade/solardrone/tree/ros2).

## Packages description

### airsim_interfaces

This package contains some messages and services used by airsim_ros_packages.

### airsim_ros_pkgs

The Airsim Wrapper, this package makes the bridge between the Airsim and our ROS system, allowing us to receive real-time information from the simulation. Regarding Airsim, we are only interested on the images from the camera and the gimbal control.

### drone_controller

This package contains the node that will interface the commands from our missions and the MAVROS, it's another safety layer that avoids some mistakes to be sent to the Ardupilot, like a take-off command when the drone is already in the sky.

This node is also the responsible to set the drone to the guided mode and arm it when it starts.

### mission

This package contains all the nodes that will run the missions.

Right now only two missions are implemented, a dummy mission that will basically follow the waypoint, without any kind of control, and a PID mission, that will constantly update it's position using a PID algorithm to track the line.

### solar_panels_img_processing

This package contains the nodes responsible for the computer vision algorithms and the georefering.

The output of the CV node is a list of 2 points, marking the beginning and the end of the line in pixels, the georefering gets this information and converts it to drone coordinates.

Right now only the Canny algorithm is implemented.

## Notion Documentation

[Set-up Remote Access with the USN PC](https://www.notion.so/Set-up-Remote-Access-with-the-USN-PC-86849b546e214a9fb138d3bd1ba4bfa8)

[Export Objects from UE4.25 to UE5.2](https://www.notion.so/Export-Objects-from-UE4-25-to-UE5-2-777eeee71d0041eb92479b4c5f87e3c1?pvs=25)

[Install Airsim 5.2](https://www.notion.so/Install-Airsim-5-2-47eeca6c2ea6447c956d6aec769526cf?pvs=25)

[Add Airsim plugin to UE5](https://www.notion.so/Add-Airsim-plugin-to-UE5-ce52d1c1fe1e4f6595cd06a81f18fadd)

[PADS PC](https://www.notion.so/PADS-PC-5e096fcddc864f41addf4043902a1ed4)

## Dependencies

### Windows

Besides the UE5.2 and AirSim, the XLaunch should also be installed.

You can download the XLaunch installer in [this link](http://www.straightrunning.com/XmingNotes/).

### WSL

On WSL we need a few python libraries. [OpenCV](https://pypi.org/project/opencv-python/) to handle the computer vision operations and [CASADI](https://web.casadi.org/) that is applied on the MPC algorithm.

To install these libraries, just run the following command:

```
$ pip install casadi==3.6.5 opencv-python
```

Also, make sure [Ardupilot](https://ardupilot.org/) and [ROS2](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Windows.html#install-ros-2-in-wsl) are installed.

## How to Run

The first step after installing all the dependencies in the previous section is to clone the repository and build the packages.

### Set-up the environment

First, download/clone this repository, then copy the packages inside the src folder into the source of your ROS workspace.

You can clone on github with the commands:

```
$ git clone https://github.com/professorfabioandrade/solardrone.git
$ git switch ros2
```

If a ROS workspace created was not created yet, just run the following commands.

```
$ cd ~
$ mkdir -p ~/ros2_ws/src
```

Then copy the files to the workspace.

```
$ cp -r /path/to/solardrone/folder ~/ros2_ws/src
```

After that, use rosdep to install all the ROS dependencies.

```
$ cd ~/ros2_ws/
$ rosdep install --from-paths src --ignore-src -r -y
```

Now we can build.

### Build

```
$ cd ~/ros2_ws/
$ colcon build
```

### Run the system

To run the system, first we need to open the model on UE5.2 and click on the green button, nothing will happen, this is normal, it will only load the drone when the Ardupilot is launched. Then, open the XLaunch and just click **Next** on the dialog windows until the last one.

On WSL, we will need 5 terminals open, to run each of the functions, the commands and its meaning are described below.

**Terminal 1: Ardupilot**
This terminal will handle the Ardupilot. To start the ardupilot, just run:

```
$ sim_vehicle.py -v ArduCopter -f airsim-copter --console --map
```

Once the Ardupilot is up and running, you will notice that the drone will finally load on UE5.2 model.

**Terminal 2: MAVROS + Airsim Wrapper**
We are now ready for launch the interfaces that will communicate with Ardupilot and Airsim and convert its information to something we can reach via ROS topics and services.

To do it, we need to launch MAVROS, the package that makes the bridge between the Ardupilot and our ROS codes.

```
$ cd Colosseum/ros2
$ source install/setup.bash
$ ros2 launch mission initial_setup.launch.py
```

**Terminal 3: Launch Drone Controller Node**
This node will interface the commands from our missions and the MAVROS, it's another safety layer that avoids some mistakes to be sent to the Ardupilot, like a take-off command when the drone is already in the sky.

This node is also the responsible to set the drone to the guided mode and arm it when it starts.

```
$ cd Colosseum/ros2
$ source install/setup.bash
$ ros2 launch drone_controller drone_controller.launch.py
```

**Terminal 4: Computer Vision Algorithm and Georefering**
This terminal will launch the Computer vision algorithm and the georefering. The output of the CV node is a list of 2 points, marking the beginning and the end of the line in pixels, the georefering gets this information and converts it to drone coordinates.

Right now only the Canny algorithm is implemented.

```
$ cd Colosseum/ros2
$ source install/setup.bash
$ ros2 launch solar_panels_img_processing canny_with_georef.launch.py
```

**Terminal 5: Mission**
Finally, the terminal that will run the mission. Right now only two missions are implemented, a dummy mission that will basically follow the waypoint, without any kind of control, and a PID mission, that will constantly update it's position using a PID algorithm to track the line.

To run the dummy mission:

```
$ cd Colosseum/ros2
$ source install/setup.bash
$ ros2 launch mission dummy_mission.launch.py
```

To run the PID mission:

```
$ cd Colosseum/ros2
$ source install/setup.bash
$ ros2 launch mission pid_mission.launch.py
```

## Troubleshooting

### Display Error

It happens when the XLaunch can't find the IP address of the WSL terminal. When this happens, it's easier to set the environmental variable DISPLAY as default with the command:

```
$ export DISPLAY=0:0
```

### OpenCV error

Similar to the previous error, and the solution is similar.

```
$ export DISPLAY=:0
```

### Qt problem

Similar to the previous error, and the solution is similar.

```
$ export DISPLAY=:0
```

### Error on setting mode to guided

By default, Ardupilot doesn't allow us to set as guided without perform the arming check. If this is the case, go to the **Terminal 1** where the Ardupilot is running and set the parameter ARMING_CHECK as false.

```
$ param set ARMING_CHECK 0
```

## Features

- [x] Start a node that set mode to GUIDED, arm the motors and prepare to take-off
- [x] Use Canny and HoughLines for line detection
- [x] Georefering
- [x] Dummy Mission (just a code that follows the waypoints, without a control)
- [x] PID Mission
- [ ] MPC Mission
- [ ] DSEF Line Detection

## References
