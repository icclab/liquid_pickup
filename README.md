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
git clone -b 4.5.x https://github.com/BehaviorTree/BehaviorTree.CPP.git
colcon build
```
1. **For simulation only**: After sourcing the `~/colcon_ws` *(usual command `. install/setup.bash`)* in one terminal run `ros2 launch icclab_summit_xl summit_xl_simulation.launch.py`.
2. In another similar sourced terminal, run `ros2 launch icclab_summit_xl summit_xl_move_it.launch.py use_sim_time:=true | grep -v moveit_robot_model.robot_model`. `use_sim_time:=true` is default for **simulation**, but when working with the **real arm** put `use_sim_time:=false`.
3. For running the Behavior Tree first clone, build and source this GitHub repo in your ROS 2 workspace, for example `~/rap/gaurav_ws$`. Then run `ros2 launch liquid_pickup liquid_pickup_launch.py`. For simulation and real robot, we should switch between `use_sim_time:=true` and `use_sim_time:=false` in the launch file, for example [here](https://github.com/Gaurav-Kapoor-07/liquid_pickup/blob/main/launch/liquid_pickup_launch.py#L14).
4. For providing a custom Behavior Tree (BT) XML to the BT node, place your XML file in [config]((https://github.com/Gaurav-Kapoor-07/liquid_pickup/tree/main/config)) directory of this repo. Then this BT Tree could be provided to the BT node parameter via launch, for example [here](https://github.com/Gaurav-Kapoor-07/liquid_pickup/blob/main/launch/liquid_pickup_launch.py#L14).
5. For visualizing the BT install [Groot2](https://www.behaviortree.dev/groot/), and run `./groot2` in the installed directory, for example `~/Groot2/bin`.
