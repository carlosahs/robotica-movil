# Puzzlebot challenge
This repository includes the code used to give a partial solution to the Puzzlebot challenge.

# Requirements
* ROS Melodic
* ROS Noetic

## ROS Melodic scripts
Scripts under `melodic` must be placed inside the directory `catkin_ws/src/puzzlebot/scripts` in the Jetson Nano provided by Manchester Robotics.

## ROS Noetic scripts
A ROS Noetic installation must be running to run the script inside `noetic`, which includes the neural net and ROS publisher for the model predictions.

## Neural net requirements
To be able to run `img_node.py`, the following dependencies must be installed inside a virtual environment:
```
python -m pip install -r yolo_requirements.txt
```
