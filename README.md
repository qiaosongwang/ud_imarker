ud_imarker
==========
This is the marker server node for RVIZ. It subscribes user mouse clicks from ud_cursor and pointcloud2 data, performs user-assisted pointcloud processing, then publishes marker/processed point cloud back to RVIZ to highlight the region of interest, or transfer user-edited data to other ROS nodes.

To install: You also need to download ud_cursor and ud_measurement_panel in order to compile the code. Git clone the following repositories in your catkin workspace src directory:

git clone https://github.com/qiaosongwang/ud_cursor.git 
& git clone https://github.com/qiaosongwang/ud_measurement_panel.git 
& git clone https://github.com/qiaosongwang/ud_imarker.git

Then do a catkin_make to compile the code and install the rviz plugin.

To use: Press the + symbol in rviz to insert UD_cursor. When using the ud_imarker tool, press i for menu/mode selection, m for changing viewpoints, u for adding/moving points. 

Contact Info:
Qiaosong Wang
qiaosong@udel.edu
