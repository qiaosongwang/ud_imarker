ud_imarker
==========
This is the marker server node for RVIZ. It subscribes user mouse clicks from ud_cursor and pointcloud2 data, performs user-assisted pointcloud processing, then publishes marker/processed point cloud back to RVIZ to highlight the region of interest, or transfer user-edited data to other ROS nodes.

To use this marker tool, please clone directory to catkin_ws/src and do a catkin_make, then type rosrun ud_imarker UD_rviz_interaction to start the marker server. The default fame id is /base_link and please make sure you inserted Marker in RVIZ. Note that the main program is in UD_rviz_interaction.cpp, UD_iMarker.cpp is the skeleton code for future integration of interactive markers.

When using the ud_imarker tool, press i for menu/mode selection, m for changing viewpoints, u for adding/moving points.

Contact Info:
Qiaosong Wang
qiaosong@udel.edu
