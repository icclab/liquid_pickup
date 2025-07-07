# Collect liquid samples and deploy sensors at ports
**Main concepts:**
1. [Behavior trees](https://www.behaviortree.dev/) for managing multiple robot navigation and arm manipulation actions through Finite State Machines.
2. [ROS 2](https://docs.ros.org/en/humble/index.html) for the navigation and manipulation nodes.
3. [Nav2](https://docs.nav2.org/) is used for navigation.
4. [MoveIt 2](https://moveit.picknik.ai/main/index.html) is used for manipulation.

**Working example of Behavior Tree:** [https://www.youtube.com/watch?v=NtHkbNBGDB4](https://www.youtube.com/watch?v=NtHkbNBGDB4)

![BT_2](https://github.com/user-attachments/assets/704c91b5-5606-4992-aeb4-ca5d3f310832)

**How to run:**

***Installation***
```
git clone -b 4.5.x https://github.com/BehaviorTree/BehaviorTree.CPP.git && \
git clone https://github.com/icclab/add-obstacle-costmap-plugin-layer.git && \
git clone https://github.com/icclab/ros2_laser_scan_merger.git && \
git clone https://github.com/icclab/moveit_visual_tools.git -b humble-trajectory-viz
colcon build --symlink-install
```
1. **For simulation only**: After sourcing the `~/colcon_ws` *(usual command `. install/setup.bash`)* in one terminal run `ros2 launch icclab_summit_xl summit_xl_simulation.launch.py`.
2. In another similar sourced terminal, run `ros2 launch icclab_summit_xl summit_xl_move_it.launch.py use_sim_time:=true | grep -v moveit_robot_model.robot_model`. `use_sim_time:=true` is default for **simulation**, but when working with the **real arm** put `use_sim_time:=false`.
3. In another terminal, run `ros2 launch icclab_summit_xl summit_xl_nav2.launch.py
4. For running the Behavior Tree first clone, build and source this GitHub repo in your ROS 2 workspace, for example `~/rap/gaurav_ws$`. Then run `ros2 launch liquid_pickup liquid_pickup_sim.launch.py coordinates:='"[[-1.2, -1.20], [1.8, 0], [1.0, 0.0]]"' | grep -v moveit_robot_model.robot_model`. For simulation and real robot, we should switch between `use_sim_time:=true` and `use_sim_time:=false` in the launch file.
5. For providing a custom Behavior Tree (BT) XML to the BT node, place your XML file in [config]((https://github.com/icclab/liquid_pickup/blob/main/config/liquid_pickup.xml)) directory of this repo. Then this BT Tree could be provided to the BT node parameter via launch.
6. The arm joint goal positions provided in the BT XML can be configured via the [arm_positions.yaml](https://github.com/icclab/liquid_pickup/blob/main/config/arm_positions.yaml).
7. For visualizing the BT install [Groot2](https://www.behaviortree.dev/groot/), and run `./groot2` in the installed directory, for example `~/Groot2/bin`.
