# ros2-gesture-robot
ROS2 Humble: hand gesture (finger count) control of TurtleBot3 in Gazebo using a webcam and OpenCV
# Gesture-Controlled TurtleBot3 (ROS2 Humble)

A webcam publishes images to ROS2. A second node counts raised fingers
with OpenCV and drives a TurtleBot3 in Gazebo through `/cmd_vel`.

## Architecture

```
camera_node -> /camera/image_raw -> gesture_node -> /cmd_vel -> turtlebot3_diff_drive (Gazebo)
```

## Gestures

| Fingers | Action |
|---|---|
| 0 (fist) | Stop |
| 1 | Forward |
| 2 | Backward |
| 3 | Turn left |
| 4 | Turn right |

Show the hand palm to the camera, fingers spread, inside the green rectangle
on a plain background.

## Requirements

- Ubuntu 22.04, ROS2 Humble
- `ros-humble-turtlebot3-gazebo`, `ros-humble-gazebo-ros-pkgs`
- `ros-humble-cv-bridge`, `python3-opencv`
- `export TURTLEBOT3_MODEL=burger`

## Run

Open 3 terminals, run `source /opt/ros/humble/setup.bash` in each:

```bash
# 1. Simulation
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch turtlebot3_gazebo empty_world.launch.py

# 2. Camera
python3 camera_node.py

# 3. Gesture recognition
python3 gesture_node_cv.py
```

Backup with turtlesim:

```bash
ros2 run turtlesim turtlesim_node
python3 gesture_node_cv.py --ros-args -p cmd_topic:=/turtle1/cmd_vel -p speed_scale:=10.0
```

## How it works

1. Skin segmentation in YCrCb colour space.
2. Largest contour is taken as the hand.
3. Convexity defects (gaps between fingers) are counted; fingers = gaps + 1.
4. Majority vote over the last 7 frames removes noise.
5. The finger count is mapped to a `geometry_msgs/Twist` command.

## Report

See the Word report in this repository: `ROS2_Gesture_Robot_Report.docx`.

## Author

Name: Manibai Erkenur, Group: Antares
