# B3RB ROS LINE FOLLOWER

## <span style="background-color: #FFFF00">INTRODUCTION</span>
A line follower project for [NXP MR-B3RB](https://nxp.gitbook.io/mr-b3rb) (Mobile Robotics Buggy 3 Rev B) for participants of [AIM 2024](https://nxpaimindia.com/).
- This project provides a framework for an autonomous driving application. (See description below)

### <span style="background-color: #CBC3E3">HARDWARE</span>
This software can run on the B3RB and Gazebo Simulator.
1. [NXP MR-B3RB](https://nxp.gitbook.io/mr-b3rb): The actual hardware rover made by NXP.
2. [Gazebo Simulator](https://gazebosim.org/home): Development and testing environment used for B3RB.
    - It's used to simulate the B3RB with it's various sensors and capabilities.
    - It's used to simulate the track with it's various challenges and obstacles.

### <span style="background-color: #CBC3E3">SOFTWARE</span>
This project is based on the autopilot project - [CogniPilot](https://cognipilot.org/) (AIRY Release for B3RB).
- Refer the [CogniPilot AIRY Dev Guide](https://airy.cognipilot.org/) for information about it's various components.
- [Cranium](https://airy.cognipilot.org/cranium/about/): A ROS workspace that performs higher level computation for CogniPilot.
  - On the hardware B3RB, it runs on [NavQPlus](https://nxp.gitbook.io/navqplus/) board (Mission Computer).
  - On the Gazebo Simulator, it runs on the Ubuntu Linux machine.
- This project includes a ROS2 Python package that integrates into the Cranium workspace.
  - This project (b3rb_ros_line_follower) should be moved to ~/cognipilot/cranium/src.
  - This is the only folder that participants would modify and submit for the regional finale.

## <span style="background-color: #FFFF00">DESCRIPTION</span>
This project contains three python scripts which provide a framework for a line follower application.
- <span style="background-color: #FFC0CB; font-weight:bold">b3rb_ros_edge_vectors</span>: It creates vectors on the edges of the road in front of the rover.
  - The image captured from the front camera is used for detecting edges of the road.
  - Cases based on number of vectors created:
    - 0: When neither left or right edge of the road is detected.
    - 1: When only 1 out of left or right edge of the road is detected.
    - 2: When both left and right edge of the road are detected.
      - Both the vectors's mid-point can't lie in either the left or right half.
      - One vector must lie in the left half and the other must lie in the right half.
  - The vectors are published to the topic "/edge_vectors".
    - Message type: "~/cognipilot/cranium/src/synapse_msgs/msg/EdgeVectors.msg".
  - We assume the part of road that is very close to the rover is relevant for decision making.
    - Hence, only the bottom 40% of the image is analyzed for edges of the road.
      - This threshold could be modified by changing the value of lower_image_height.
    - Hence, the y-coordinates of the vectors ∈ [40% of image height, image height].
  - Please feel free to modify this file if you feel that would improve the vector creation.
- <span style="background-color: #FFC0CB; font-weight:bold"> b3rb_ros_line_follower</span>: Contains framework for running the rover using edge vectors.
  - Write your code in the "edge_vectors_callback" function for line follower application.
    - This callback is called whenever a new set of vectors are published on "/edge_vectors".
  - Utilize "rover_move_manual_mode" for moving the rover. Refer its docstring for explanation.
  - Write your code in the "lidar_callback" function for obstacle avoidance and ramp detection.
    - This callback is called whenever a new set of data is published on "/scan".
  - Please note that this file contains a generic implementation of line follower functionality.
    - You are allowed to modify or implement a different method to improve performance.
- <span style="background-color: #FFC0CB; font-weight:bold"> b3rb_ros_object_recog</span>: Contains framework for recognizing objects on the track.
  - Write your code in the "camera_image_callback" function.

## <span style="background-color: #FFFF00">EXECUTION STEPS</span>

Follow - [https://airy.cognipilot.org/getting_started/install/](https://airy.cognipilot.org/getting_started/install/).

Move "b3rb_ros_line_follower" to "~/cognipilot/cranium/src/".

Open a terminal and follow the following steps to setup **Cranium**.
```
cd ~/cognipilot/cranium/src/
rm -rf dream_world
rm -rf synapse_msgs

git clone git@github.com:NXPHoverGames/dream_world.git
git clone git@github.com:NXPHoverGames/synapse_msgs.git

cd ~/cognipilot/cranium/src/dream_world
git checkout aim_track

cd ~/cognipilot/cranium/src/synapse_msgs
git checkout b3rb_ros_line_follower
```

Open a terminal and follow the following steps for building **Cranium** and running **Gazebo Simulation**.
```
cd ~/cognipilot/cranium/
colcon build
ros2 launch b3rb_gz_bringup sil.launch.py world:=<track_name>
```

Open a terminal and follow the following steps for running **b3rb_ros_edge_vectors**.
```
source ~/cognipilot/cranium/install/setup.bash
ros2 run b3rb_ros_line_follower vectors
```

Open a terminal and follow the following steps for running **b3rb_ros_object_recog**.
```
source ~/cognipilot/cranium/install/setup.bash
ros2 run b3rb_ros_line_follower detect
```

Open a terminal and follow the following steps for running **b3rb_ros_line_follower**.
```
source ~/cognipilot/cranium/install/setup.bash
ros2 run b3rb_ros_line_follower runner
```

## <span style="background-color: #FFFF00">LIMITATIONS</span>
- The track width should be 3.5 to 4.0 times that of the rover.
- At the start, the rover should be inside the road and in the middle.
- Intersections and forks should be detected using machine learning.
  - There should be road signs telling the rover which path to take.
- The rover should stop by detecting the STOP sign at the finish line.
  - The participants need to implement the ML model for detecting stop signs.
  - Alternatively, the rover could be stopped by terminating runner manually.
- Obstacles, bumps, ramps on the track need to be handled by the participants.
- TODO: This project needs to be tested on the hardware track when available.
