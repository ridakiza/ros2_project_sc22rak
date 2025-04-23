
## **Setup Instructions**

### **Step 1: Enter the Singularity Shell**
Before running any commands, ensure you are inside the **Singularity shell**.

```python
cd ~robotics/ros2_ws

```
```python
source ~/.bashrc

```
for the Gazebo interface 
```python
ros2 launch turtlebot3_gazebo turtlebot3_task_world_2025.launch.py

```

for the map and to set initial position 

```python
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/3edyear/robotics/ros2_ws/src/ros2_project_sc22rak/map/map.yaml

```

then for the script 
```python
ros2 run ros2_project_sc22rak ros2_project_sc22rak

```
