# Puzzlebot challenge
This repository includes the code used to give a partial solution to the Puzzlebot challenge.

# Requirements
* ROS Melodic
* ROS Noetic

## ROS Melodic scripts
Scripts under `melodic` must be placed inside the directory `catkin_ws/src/puzzlebot/scripts` in the Jetson Nano provided by Manchester Robotics.

### Inside the Puzzletbot
The following commands must be running in the following order:
```
$ roslaunch ros_deep_learning video_source.ros1.launch
$ rosrun puzzlebot line_follower.py
$ rosrun puzzlebot state_machine.py
```

## ROS Noetic scripts
A ROS Noetic installation must be running to run the script inside `noetic`, which includes the neural net and ROS publisher for the model predictions.

### Before running `img_node.py`
To run `img_node.py`, `line_follower.py` must be running first in the Puzzlebot:
```
$ rorsrun puzzlebot line_follower.py`
```

### After running `img_node.py`
Once `img_node.py` is running, the state machine can be ran:
```
$ rosrun puzzlebot state_machine.py
```

## Neural net requirements
To be able to run `img_node.py`, the following dependencies must be installed inside a virtual environment:
```
python -m pip install -r yolo_requirements.txt
```
