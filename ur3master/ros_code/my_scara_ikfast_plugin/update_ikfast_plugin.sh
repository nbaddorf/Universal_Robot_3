search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=ur3.srdf
robot_name_in_srdf=ur3
moveit_config_pkg=scara
robot_name=ur3
planning_group_name=Scara_Arm
ikfast_plugin_pkg=my_scara_ikfast_plugin
base_link_name=base_link
eef_link_name=end_effector
ikfast_output_path=/home/ros/catkin_ws/src/my_scara_ikfast_plugin/src/ur3_Scara_Arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
