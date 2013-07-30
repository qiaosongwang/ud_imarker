ud_imarker
==========
This is the marker server node for RVIZ. It subscribes user mouse clicks from ud_cursor and pointcloud2 data, performs user-assisted pointcloud processing, then publishes marker/processed point cloud back to RVIZ.

To use this marker tool, please clone directory to catkin_ws/src and do a catkin_make, then type rosrun ud_imarker UD_rviz_interaction to start the marker server. The default fame id is /base_link and please make sure you inserted Marker in RVIZ.
