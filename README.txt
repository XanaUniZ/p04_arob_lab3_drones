The experiments are launched from the same launch file, with the following command line:
roslaunch lab3_drones basic.launch

From there one can activate or deactivate Gazebo visualization as well as making all the changes required for the experiments. 

Inside the "drone_race" you can change the desired circuit by changing the "targets_file_path":
<param name="targets_file_path"           value="/home/arob/catkin_ws/src/p04_arob_lab3_drones/data/gates.txt"/>
or
<param name="targets_file_path"           value="/home/arob/catkin_ws/src/p04_arob_lab3_drones/data/gates_hard.txt"/>

Also in that node you can choose the kind of yaw control used by changing the "yaw_control" argument. You can choose between the following: 
<param name="yaw_control"           value="no_control"/>
<param name="yaw_control"           value="yaw_2D_vel"/>
<param name="yaw_control"           value="RPY_control"/>
<param name="yaw_control"           value="next_gate"/>

