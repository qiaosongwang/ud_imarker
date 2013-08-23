//RVIZ measurement tool
//Qiaosong Wang
//University of Delaware
//qiaosong@udel.edu

//----------------------------------------------------------------------------

#include "ud_imarker.hh"
#include "ud_measurement_panel/MeasurementCommand.h"
#include "ud_cursor/UDCursor.h"

//----------------------------------------------------------------------------

// this is where all of the magic numbers are set

ud_measurement_panel::MeasurementCommand measurement_msg;

// rosrun pcl_ros pcd_to_pointcloud golfcart_pillar1_hokuyo.pcd 1
// rosrun pcl_ros pcd_to_pointcloud path1_vehicle_only_KINFU.pcd 1

bool have_scene_cloud = false;   // this becomes true when either a message arrives or we
                                 // load a .pcd locally

bool have_object_cloud = false;

//----------------------------------------------------------------------------

// a vehicle object

bool do_load_kinfu_vehicle = false;

// various scenes

bool do_load_armory_golf = false;
bool do_load_armory_valves1 = false;
bool do_load_tepra_PS = false;
bool do_load_tepra_DS = false;

//bool do_load_scene_xyzi = false;
//bool do_load_scene_xyz = false;

//char *scene_xyzi_filename = NULL;
//char *scene_xyz_filename = NULL;

//----------------------------------------------------------------------------

//bool do_clip = false;

// this should change with menu selection

bool do_crop = false;

//----------------------------------------------------------------------------

sensor_msgs::PointCloud2 inlier_cloud_msg;
sensor_msgs::PointCloud2 outlier_cloud_msg;

sensor_msgs::PointCloud2 input_cloud_msg;
sensor_msgs::PointCloud2 object_cloud_msg;

//----------------------------------------------------------------------------

//pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloudptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr raw_object_cloudptr;
pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloudptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr cursor_cloudptr;

// aka the scene
pcl::PointCloud<pcl::PointXYZI>::Ptr raw_xyzi_input_cloudptr;
pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi_input_cloudptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr raw_input_cloudptr;
pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloudptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloudptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr input_cropped_cloudptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr rough_inlier_cloudptr;
pcl::PointCloud<pcl::PointXYZ>::Ptr rough_outlier_cloudptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloudptr;
pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloudptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr projected_inlier_cloudptr;
pcl::PointCloud<pcl::PointXYZ>::Ptr level_inlier_cloudptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloudptr;

//----------------------------------------------------------------------------

ros::Publisher input_cloud_pub;
ros::Publisher object_cloud_pub;

ros::Publisher inlier_cloud_pub;
ros::Publisher outlier_cloud_pub;

//----------------------------------------------------------------------------

pcl::ModelCoefficients rough_plane_coefficients;
pcl::ModelCoefficients rough_line_coefficients;

pcl::ModelCoefficients::Ptr plane_coefficients_ptr;
pcl::ModelCoefficients line_coefficients;

pcl::ModelCoefficients::Ptr circle_coefficients_ptr;

pcl::ModelCoefficients::Ptr ground_plane_coefficients_ptr;

pcl::ModelCoefficients cylinder_coefficients;

//----------------------------------------------------------------------------

boost::shared_ptr<InteractiveMarkerServer> server;

MenuHandler menu_handler;

MenuHandler::EntryHandle h_move_entry;
MenuHandler::EntryHandle h_delete_entry;
MenuHandler::EntryHandle h_measure_entry;
MenuHandler::EntryHandle h_estimate_entry;
MenuHandler::EntryHandle h_crop_entry;
MenuHandler::EntryHandle h_display_entry;

MenuHandler::EntryHandle h_mode_last;

//----------------------------------------------------------------------------

//global points
Eigen::Vector4f minPoint;
Eigen::Vector4f maxPoint;
 
//double rough_max_cylinder_radius = 0.1; // 05;
//double max_cylinder_radius = 0.05; // 0.03

//double cylinder_inlier_distance_threshold = 0.025;

//double rough_max_plane_distance = 0.1;
//double max_plane_distance = 0.025;

//double ransac_inlier_distance_threshold = 0.05;

//Defining ROS parameters

//ros::Publisher rough_cylinder_marker_pub;
ros::Publisher cylinder_marker_pub;

ros::Publisher plane_pub;

ros::Publisher marker_pub;
ros::Publisher marker_array_pub;

ros::Publisher ptcloud_pub;

ros::Publisher inliers_cloud_pub;
ros::Publisher outliers_cloud_pub;

ros::NodeHandle *nhp;
ros::Subscriber cursor_sub;
ros::Subscriber click_sub;
ros::Subscriber ptcloud_sub;
ros::Subscriber measurement_sub;

float cpdist=0; //Current frame
float globalscale = 1;  //What are the units on this? -Brad
geometry_msgs::Point fp; //future point
geometry_msgs::Point cp; //current point
geometry_msgs::Point pp; //previous point

//Number of points, need to be dynamically assiagned once the panel is done
//int num_polypoints=100;
//int polycount = 0;

//Declare 3D point storage structure
//vector<vector<float> > polypoints(3, vector<float>(100));

vector <pcl::PointXYZ> ud_cursor_pts;
double ud_cursor_pt_radius = 0.025;
double ud_cursor_pt_line_width = 0.01;
int ud_cursor_pt_selection_index = NONE_SELECTED;

bool ud_cursor_pt_display_indices = false;
bool ud_cursor_pt_display_connections = false;
//bool ud_cursor_pt_do_move = false;

//int ud_cursor_edit_mode;

//Declare received pointcloud from RVIZ
pcl::PointCloud< pcl::PointXYZRGB> rviz_pt;
pcl::PointCloud< pcl::PointXYZRGB> rviz_pt_filtered;

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// WARNING: hard-coded path to file

void load_armory_golfcart_hokuyo_pcd()
{
  ground_plane_coefficients_ptr->values[0] = -0.00572;
  ground_plane_coefficients_ptr->values[1] =  0.03041;
  ground_plane_coefficients_ptr->values[2] =  0.99952;
  ground_plane_coefficients_ptr->values[3] =  0.23462;
  
  load_xyzi_point_cloud("/home/cer/Downloads/golfcart_pillar1_hokuyo.pcd", *raw_xyzi_input_cloudptr, 1.0);
  transform_to_level(*raw_xyzi_input_cloudptr, *xyzi_input_cloudptr, *ground_plane_coefficients_ptr);
  
  //  copy_xyzi_to_point_cloud(*xyzi_input_cloudptr, *raw_input_cloudptr);
  copy_xyzi_to_point_cloud(*xyzi_input_cloudptr, *input_cloudptr);
  
  pcl::toROSMsg(*xyzi_input_cloudptr, input_cloud_msg);
  input_cloud_msg.header.frame_id = "base_link";
}

//----------------------------------------------------------------------------

// WARNING: hard-coded path to file

void load_armory_valves1_hokuyo_pcd()
{
  /*
  ground_plane_coefficients_ptr->values[0] = -0.00572;
  ground_plane_coefficients_ptr->values[1] =  0.03041;
  ground_plane_coefficients_ptr->values[2] =  0.99952;
  ground_plane_coefficients_ptr->values[3] =  0.23462;
  */

  //  load_xyzi_point_cloud("/home/cer/Downloads/armory_upstairs_valves1_hokuyo.pcd", *raw_xyzi_input_cloudptr, 1.0);
  load_xyzi_point_cloud("/home/cer/Downloads/armory_upstairs_valves1_hokuyo.pcd", *xyzi_input_cloudptr, 1.0);
  //  transform_to_level(*raw_xyzi_input_cloudptr, *xyzi_input_cloudptr, *ground_plane_coefficients_ptr);
  
  //  copy_xyzi_to_point_cloud(*xyzi_input_cloudptr, *raw_input_cloudptr);
  copy_xyzi_to_point_cloud(*xyzi_input_cloudptr, *input_cloudptr);
  
  pcl::toROSMsg(*xyzi_input_cloudptr, input_cloud_msg);
  input_cloud_msg.header.frame_id = "base_link";
}

//----------------------------------------------------------------------------

// WARNING: hard-coded path to file

void load_tepra_DS_asus_pcd()
{
  ground_plane_coefficients_ptr->values[0] = -0.03658;
  ground_plane_coefficients_ptr->values[1] =  0.71868;
  ground_plane_coefficients_ptr->values[2] = -0.69438; 
  ground_plane_coefficients_ptr->values[3] =  1.48465;

  load_point_cloud("/home/cer/Documents/papers/tepra2013/capture/driver_steering/driver_steering_ASUS.pcd", *raw_input_cloudptr, 1.0);
  transform_to_level(*raw_input_cloudptr, *input_cloudptr, *ground_plane_coefficients_ptr);
  
  pcl::toROSMsg(*input_cloudptr, input_cloud_msg);
  input_cloud_msg.header.frame_id = "base_link";
}

//----------------------------------------------------------------------------

// WARNING: hard-coded path to file

void load_tepra_PS_asus_pcd()
{
  ground_plane_coefficients_ptr->values[0] = -0.03001;
  ground_plane_coefficients_ptr->values[1] =  0.84770;
  ground_plane_coefficients_ptr->values[2] = -0.52963;
  ground_plane_coefficients_ptr->values[3] =  1.56254; 

  load_point_cloud("/home/cer/Documents/papers/tepra2013/capture/passenger_steering/passenger_steering_ASUS.pcd", *raw_input_cloudptr, 1.0);
  transform_to_level(*raw_input_cloudptr, *input_cloudptr, *ground_plane_coefficients_ptr);
  
  pcl::toROSMsg(*input_cloudptr, input_cloud_msg);
  input_cloud_msg.header.frame_id = "base_link";
}

//----------------------------------------------------------------------------

void load_scene_pcd()
{
  if (do_load_armory_golf)
    load_armory_golfcart_hokuyo_pcd();
  else if (do_load_armory_valves1)
    load_armory_valves1_hokuyo_pcd();
  else if (do_load_tepra_PS)
    load_tepra_PS_asus_pcd();
  else if (do_load_tepra_DS)
    load_tepra_DS_asus_pcd();

  have_scene_cloud = true;
}

//----------------------------------------------------------------------------

void advertise_scene_pointcloud()
{
  // special because XYZI

  if (do_load_armory_golf || do_load_armory_valves1)
    input_cloud_pub = nhp->advertise<pcl::PointCloud<pcl::PointXYZI> > ("scene_cloud", 1);

  // default XYZ

  else 
    input_cloud_pub = nhp->advertise<pcl::PointCloud<pcl::PointXYZ> > ("scene_cloud", 1);

}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// WARNING: hard-coded path to file

void load_vehicle_pcd()
{
  vector <double> rough_path1_kinfu_vehicle_state;

  rough_path1_kinfu_vehicle_state.resize(VEHICLE_STATE_DIMENSIONS);

  // pre-calculated vehicle state from find_vehicle() on full kinfu map

  rough_path1_kinfu_vehicle_state[VEHICLE_LENGTH] = 2.323;
  rough_path1_kinfu_vehicle_state[VEHICLE_WIDTH] = 1.343;
  rough_path1_kinfu_vehicle_state[VEHICLE_X] = 1.660;
  rough_path1_kinfu_vehicle_state[VEHICLE_Y] = 1.863;
  rough_path1_kinfu_vehicle_state[VEHICLE_THETA] = DEG2RAD(90-87.478);
 
  if (do_load_kinfu_vehicle) {
    load_point_cloud("/home/cer/Documents/papers/tepra2013/capture/kinfu/path1_vehicle_only_KINFU.pcd", *raw_object_cloudptr, 1.0);

    // this takes the vehicle point cloud to the origin -- i.e., center of vehicle is at (0, 0), etc.

    rectify_vehicle_point_cloud(*raw_object_cloudptr, *object_cloudptr, rough_path1_kinfu_vehicle_state);
    pcl::toROSMsg(*object_cloudptr, object_cloud_msg);
    object_cloud_msg.header.frame_id = "base_link";
  }
}

//----------------------------------------------------------------------------

// the OBJECT is the complicated thing of interest in the scene.
// it could be the vehicle, it could be a ladder, it could be a drill.
// for now there is only one.

void load_object_pcd()
{
  load_vehicle_pcd();

  have_object_cloud = true;
}

//----------------------------------------------------------------------------

void advertise_object_pointcloud()
{
  object_cloud_pub = nhp->advertise<pcl::PointCloud<pcl::PointXYZ> > ("object_cloud", 1);
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

void initialize_pcl()
{
  ud_cursor_pts.clear();
  
  cursor_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);

  raw_xyzi_input_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
  xyzi_input_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZI>);

  raw_input_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
  input_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);

  temp_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);

  raw_object_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
  object_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);

  input_cropped_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);

  inlier_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
  outlier_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);

  projected_inlier_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
  level_inlier_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);

  rough_inlier_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
  rough_outlier_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);

  hull_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);

  plane_coefficients_ptr.reset(new pcl::ModelCoefficients);
  circle_coefficients_ptr.reset(new pcl::ModelCoefficients);

  ground_plane_coefficients_ptr.reset(new pcl::ModelCoefficients);
  ground_plane_coefficients_ptr->values.resize(4);

  load_scene_pcd();

  load_object_pcd();

  // distance slice raw_input_cloudptr and remove ground plane -> input_cloudptr

  //  Eigen::Matrix4f tform;
  //  icp_align_point_clouds(cloud1_ptr, object_cloudptr, cloud1_registered,  tform);
}

//----------------------------------------------------------------------------

// for roof pillar

void send_line_cylinder_marker(pcl::ModelCoefficients & line_coeffs,
			  double radius,
			  double min_xp, double max_xp,
			  float r, float g, float b, float a)
{
  // Initialize marker parameters

  visualization_msgs::Marker cyl;
  cyl.header.frame_id="/base_link";
  cyl.header.stamp =ros::Time::now();
  cyl.ns="UD_rviz_interaction";
  cyl.action=visualization_msgs::Marker::ADD;

  printf("%.3lf %.3lf %.3lf = %.3lf [%.3lf min, %.3lf max]\n", 
	 line_coeffs.values[3],
	 line_coeffs.values[4],
	 line_coeffs.values[5],
	 line_coeffs.values[3]*line_coeffs.values[3]+line_coeffs.values[4]*line_coeffs.values[4]+line_coeffs.values[5]*line_coeffs.values[5],
	 min_xp,
	 max_xp);

  cyl.pose.position.x = line_coeffs.values[0] + 0.5 * (max_xp + min_xp) * line_coeffs.values[3];
  cyl.pose.position.y = line_coeffs.values[1] + 0.5 * (max_xp + min_xp) * line_coeffs.values[4];
  cyl.pose.position.z = line_coeffs.values[2] + 0.5 * (max_xp + min_xp) * line_coeffs.values[5];

  //  tf::Quaternion q = shortest_rotation_quaternion(-line_coeffs.values[3], -line_coeffs.values[4], -line_coeffs.values[5], 0, 0, 1);
  tf::Quaternion q = shortest_rotation_quaternion(0, 0, 1,
						  line_coeffs.values[3], line_coeffs.values[4], line_coeffs.values[5]);
 
  cyl.pose.orientation.x =q.x();
  cyl.pose.orientation.y =q.y();
  cyl.pose.orientation.z =q.z();
  cyl.pose.orientation.w =q.w();
  

  //ID

  cyl.id =33;
  
  //Type
  cyl.type = visualization_msgs::Marker::CYLINDER;
  
  //Scale

  cyl.scale.x = 2.0*radius;
  cyl.scale.y = 2.0*radius;
  cyl.scale.z = max_xp - min_xp;
  
  //Color

  cyl.color.r = r;
  cyl.color.g = g;
  cyl.color.b = b;
  cyl.color.a = a;


  cylinder_marker_pub.publish(cyl);
}

//----------------------------------------------------------------------------

// for steering wheel

void send_circle_cylinder_marker(double x, double y, double z,
				 pcl::ModelCoefficients & plane_coeffs,
				 double radius, double height,
				 float r, float g, float b, float a)
{
  // Initialize marker parameters

  visualization_msgs::Marker cyl;
  cyl.header.frame_id="/base_link";
  cyl.header.stamp =ros::Time::now();
  cyl.ns="UD_rviz_interaction";
  cyl.action=visualization_msgs::Marker::ADD;

  /*
  printf("%.3lf %.3lf %.3lf = %.3lf [%.3lf min, %.3lf max]\n", 
	 line_coeffs.values[3],
	 line_coeffs.values[4],
	 line_coeffs.values[5],
	 line_coeffs.values[3]*line_coeffs.values[3]+line_coeffs.values[4]*line_coeffs.values[4]+line_coeffs.values[5]*line_coeffs.values[5],
	 min_xp,
	 max_xp);
  */

  cyl.pose.position.x = x;
  cyl.pose.position.y = y;
  cyl.pose.position.z = z;

  tf::Quaternion q = shortest_rotation_quaternion(0, 0, 1,
						  plane_coeffs.values[0], plane_coeffs.values[1], plane_coeffs.values[2]);
 
  cyl.pose.orientation.x =q.x();
  cyl.pose.orientation.y =q.y();
  cyl.pose.orientation.z =q.z();
  cyl.pose.orientation.w =q.w();

  //ID

  cyl.id =33;
  
  //Type
  cyl.type = visualization_msgs::Marker::CYLINDER;
  
  //Scale

  cyl.scale.x = 2.0*radius;
  cyl.scale.y = 2.0*radius;
  cyl.scale.z = height;
  
  //Color

  cyl.color.r = r;
  cyl.color.g = g;
  cyl.color.b = b;
  cyl.color.a = a;


  cylinder_marker_pub.publish(cyl);
}

//----------------------------------------------------------------------------

void send_ud_cursor_point_markers()
{
  // Initialize marker parameters

  visualization_msgs::Marker psphere;
  psphere.header.frame_id="/base_link";
  psphere.header.stamp =ros::Time::now();
  psphere.ns="UD_rviz_interaction";
  psphere.action=visualization_msgs::Marker::ADD;
  psphere.pose.orientation.x =0.0;
  psphere.pose.orientation.y =0.0;
  psphere.pose.orientation.z =0.0;
  psphere.pose.orientation.w =1.0;
  
  //ID
  psphere.id =3;
  
  //Type
  psphere.type = visualization_msgs::Marker::SPHERE_LIST;
  
  //Scale
  psphere.scale.x = 2.0*ud_cursor_pt_radius;
  psphere.scale.y = 2.0*ud_cursor_pt_radius;
  psphere.scale.z = 2.0*ud_cursor_pt_radius;
  
  //Color

  /*
  psphere.color.r = 0.0;
  psphere.color.g = 1.0;
  psphere.color.b = 0.0;
  psphere.color.a = 0.6;
  */

  geometry_msgs::Point p;
  std_msgs::ColorRGBA color;

  for (int i = 0; i < ud_cursor_pts.size(); i++) {
    
    p.x = ud_cursor_pts[i].x;
    p.y = ud_cursor_pts[i].y;
    p.z = ud_cursor_pts[i].z;

    psphere.points.push_back(p);

    if (i == ud_cursor_pt_selection_index) {
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.5;
      color.a = 0.6;
    }
    else {
      color.r = 0.0;
      color.g = 0.0;
      color.b = 1.0;
      color.a = 0.6;
    }

    psphere.colors.push_back(color);

  }
  
  // Publish markers

  marker_pub.publish(psphere);

  if (ud_cursor_pt_display_connections) {

    psphere.id = 2;
    
    psphere.type = visualization_msgs::Marker::LINE_STRIP;

    psphere.scale.x = ud_cursor_pt_line_width;

    for (int i = 0; i < ud_cursor_pts.size(); i++) {
      psphere.colors[i].r = 0.0;
      psphere.colors[i].g = 0.0;
      psphere.colors[i].b = 1.0;
      psphere.colors[i].a = 0.6; 
    }

    marker_pub.publish(psphere);

  }
  
  if (ud_cursor_pt_display_indices) {

    visualization_msgs::MarkerArray ptextarray;
    
    psphere.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    psphere.points.clear();
    psphere.colors.clear();
    psphere.lifetime = ros::Duration(1.0);

    for (int i = 0; i < ud_cursor_pts.size(); i++) {

      psphere.id = 4+ i;

      psphere.pose.position.x = ud_cursor_pts[i].x;
      psphere.pose.position.y = ud_cursor_pts[i].y;
      psphere.pose.position.z = ud_cursor_pts[i].z; //  + 2.0 * ud_cursor_pt_radius;

      psphere.color.r = 0.0;
      psphere.color.g = 0.0;
      psphere.color.b = 1.0;
      psphere.color.a = 1.0;

      std::stringstream point_id;
      point_id << i;
      psphere.text = point_id.str();

      ptextarray.markers.push_back(psphere);
    }

    marker_array_pub.publish(ptextarray);

  }



}

//----------------------------------------------------------------------------

// menu code taken from here:
// http://ftp.isr.ist.utl.pt/pub/roswiki/rviz(2f)Tutorials(2f)Interactive(20)Markers(3a20)Getting(20)Started.html#menu

/*

// here's an example of making menu items appear and disappear

void enableCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  menu_handler.getCheckState( handle, state );

  if ( state == MenuHandler::CHECKED )
  {
    menu_handler.setCheckState( handle, MenuHandler::UNCHECKED );
    ROS_INFO("Hiding first menu entry");
    menu_handler.setVisible( h_first_entry, false );
  }
  else
  {
    menu_handler.setCheckState( handle, MenuHandler::CHECKED );
    ROS_INFO("Showing first menu entry");
    menu_handler.setVisible( h_first_entry, true );
  }
  menu_handler.reApply( *server );
  ros::Duration(2.0).sleep();
  ROS_INFO("update");
  server->applyChanges();
}
*/

//----------------------------------------------------------------------------

Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale;
  marker.scale.y = msg.scale;
  marker.scale.z = msg.scale;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.5;


  return marker;
}

//----------------------------------------------------------------------------

InteractiveMarker makeEmptyMarker( bool dummyBox=true )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/base_link";
  int_marker.pose.position.x = 0.0;
  int_marker.pose.position.y = 0.0;
  int_marker.pose.position.z = 0.0;

  int_marker.scale = 4.0*ud_cursor_pt_radius;

  return int_marker;
}

//----------------------------------------------------------------------------

void makeMenuMarker( std::string name )
{
  InteractiveMarker int_marker = makeEmptyMarker();
  int_marker.name = name;

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::MENU; // BUTTON;
  control.always_visible = true;

  control.markers.push_back( makeBox( int_marker ) );
  int_marker.controls.push_back(control);

  server->insert( int_marker );
}

//----------------------------------------------------------------------------

// get rid of the ud_cursor point that is currently selected (last added by default
// unless an existing one is clicked on)

void DeleteSelected()
{
  printf("IM menus deprecated\n"); fflush(stdout);

  /*
  if (ud_cursor_pt_selection_index >= 0 && ud_cursor_pt_selection_index < ud_cursor_pts.size()) {
    
    ud_cursor_pts.erase(ud_cursor_pts.begin() + ud_cursor_pt_selection_index);

    // make new selected point one after the one we just deleted.  if that one was the last, make 
    // new last the selected point

    if (ud_cursor_pt_selection_index >= ud_cursor_pts.size())
      ud_cursor_pt_selection_index--;
    
    if (ud_cursor_pts.size() == 0) {

      ud_cursor_pt_selection_index = -1;

      ud_cursor_pt_do_move = false;
      menu_handler.setCheckState( h_move_entry, MenuHandler::UNCHECKED );
      menu_handler.reApply( *server );
      server->applyChanges();
    }

    send_ud_cursor_point_markers();

  }

  //  ROS_INFO("The delete last sub-menu has been found.");

  */
}

void DeleteSelectedCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  DeleteSelected();
}

//----------------------------------------------------------------------------

// get rid of every ud_cursor point we have stored

void DeleteAll()
{
  printf("IM menus deprecated\n"); fflush(stdout);

  /*
  ud_cursor_pts.clear();

  ud_cursor_pt_selection_index = -1;

  ud_cursor_pt_do_move = false;
  menu_handler.setCheckState( h_move_entry, MenuHandler::UNCHECKED );
  menu_handler.reApply( *server );
  server->applyChanges();

  //  printf("delete all: %i points\n", ud_cursor_pts.size());

  send_ud_cursor_point_markers();

  //  ROS_INFO("The delete all sub-menu has been found.");
  */
}

void DeleteAllCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  DeleteAll();
}

//----------------------------------------------------------------------------

// treat all ud_cursor points as polyline and calculated its total length

void MeasureLength() 
{
  double dx, dy, dz;
  double total_length = 0.0;
  
  if (ud_cursor_pts.size() > 1) {

    for (int i = 0; i < ud_cursor_pts.size() - 1; i++) {

      dx = ud_cursor_pts[i].x - ud_cursor_pts[i + 1].x;
      dy = ud_cursor_pts[i].y - ud_cursor_pts[i + 1].y;
      dz = ud_cursor_pts[i].z - ud_cursor_pts[i + 1].z;

      total_length += sqrt(dx*dx + dy*dy + dz*dz); 
    }

    printf("Total length = %.3lf\n", total_length);

    //    send_ud_cursor_point_markers();

  }

}

void MeasureLengthCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  MeasureLength();
}

//----------------------------------------------------------------------------

#ifdef PCA_NONSENSE

  // Placeholder for the 3x3 covariance matrix at each surface patch
  Eigen::Matrix3f covariance_matrix;
  // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
  Eigen::Vector4f xyz_centroid;

  // Estimate the XYZ centroid
  compute3DCentroid (*inlier_cloudptr, xyz_centroid);

  // Compute the 3x3 covariance matrix
  computeCovarianceMatrix (*inlier_cloudptr, xyz_centroid, covariance_matrix);

  EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
  pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);

  // smallest eigen_value is square of smallest semi-axis (aka smallest radius)

  cout << "centroid xyz " << endl << xyz_centroid << endl;
  cout << "cov mat " << endl << covariance_matrix << endl;

  printf("%lf %lf %lf %lf\n", eigen_vector[0],eigen_vector[1],eigen_vector[2],sqrt(eigen_value));
 
  float nx, ny, nz, curvature;

  pcl::solvePlaneParameters(covariance_matrix,
			    nx, ny, nz, curvature);

  printf("%lf %lf %lf %lf\n", nx, ny, nz, curvature);


  // get radius estimate from PCA on inliers?  still slightly biased by outliers, but must be better
  // than using constant measurement_msg.LineMaxDistance, right?  --provided true radius is <= measurement_msg.LineMaxDistance

  /*
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud (inlier_cloudptr);

  Eigen::Vector4f mean = pca.getMean();
  Eigen::Vector3f eigenvalues = pca.getEigenValues();
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
  */

  Eigen::Vector3f pca_eigenvalues;
  Eigen::Matrix3f pca_eigenvectors;

 // Compute mean
  Eigen::Vector4f pca_mean = Eigen::Vector4f::Zero ();
  compute3DCentroid (*inlier_cloudptr, pca_mean);
  // Compute demeanished cloud
  Eigen::MatrixXf cloud_demean;
  demeanPointCloud (*inlier_cloudptr, pca_mean, cloud_demean);

  //  cout << "demean " << endl << cloud_demean << endl;

  // Compute the product cloud_demean * cloud_demean^T
  Eigen::Matrix3f alpha = static_cast<Eigen::Matrix3f> (cloud_demean.topRows<3> () * cloud_demean.topRows<3> ().transpose ());

  cout << "alpha " << endl << alpha << endl;

  // Compute eigen vectors and values
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> evd (covariance_matrix);
  //  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> evd (alpha);
  // Organize eigenvectors and eigenvalues in ascendent order
  for (int i = 0; i < 3; ++i)
  {
    pca_eigenvalues[i] = evd.eigenvalues () [2-i];
    pca_eigenvectors.col (i) = evd.eigenvectors ().col (2-i);
  }

  cout << "pca mean " << endl << pca_mean << endl;
  cout << "pca eigenvalues " << endl << pca_eigenvalues << endl;
  cout << "pca eigenvectors " << endl << pca_eigenvectors << endl;


  //  printf("%.3lf %.3lf %.3lf\n", eigenvalues(0), eigenvalues(1), eigenvalues(2));


#endif

//----------------------------------------------------------------------------

// parametrize (if n = 2) or fit (n >= 3) LINE to all ud_cursor points

void EstimateLine()
{
  int i;

  cursor_cloudptr->points.clear();

  for (i = 0; i < ud_cursor_pts.size(); i++)
    cursor_cloudptr->points.push_back(ud_cursor_pts[i]);

  // get a rough line as our starting point

  robust_line_fit(*cursor_cloudptr,
		  *inlier_cloudptr,
		  *outlier_cloudptr,
		  rough_line_coefficients,
		  measurement_msg.CursorMaxDistance);

  // use its endpoints and a max radius to crop input point cloud to cylindrical volume of interest (VOI)

  double min_xp, max_xp;
  //  compute_line_limits(cursor_cloudptr, line_coefficients, min_xp, max_xp);
  compute_line_limits(inlier_cloudptr, rough_line_coefficients, min_xp, max_xp);

  cylinder_slice(*input_cloudptr,
		 *rough_inlier_cloudptr,
		 *rough_outlier_cloudptr,
		 rough_line_coefficients,
		 measurement_msg.LineRoughMaxDistance,
		 min_xp, max_xp);

  // fit again to get finer estimate

  robust_line_fit(*rough_inlier_cloudptr,
		  *inlier_cloudptr,
		  *outlier_cloudptr,
		  line_coefficients,
		  measurement_msg.LineMaxDistance);

  compute_line_limits(inlier_cloudptr, line_coefficients, min_xp, max_xp);

  //    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  double mean_radius = mean_pointcloud_distance_to_3D_line(inlier_cloudptr, line_coefficients);

  printf("estimated radius = %lf\n", mean_radius);

  /*
  robust_cylinder_fit(*inlier_cloudptr,
		      *rough_inlier_cloudptr,
		      *rough_outlier_cloudptr,
		      cylinder_coefficients,
		      0.01, measurement_msg.LineMaxDistance); // measurement_msg.CircleMaxDistance);
  */


  send_line_cylinder_marker(line_coefficients,
		       mean_radius, // measurement_msg.LineMaxDistance,
		       min_xp, max_xp,
		       1.0, 0.0, 0.0, 0.5);

  inlier_cloudptr->header.frame_id = "base_link";
  inlier_cloud_pub.publish(*inlier_cloudptr);

  outlier_cloudptr->header.frame_id = "base_link";
  outlier_cloud_pub.publish(*outlier_cloudptr);

  //  rough_cylinder_marker_pub.publish();

  /*
  inlier_cloudptr->header.frame_id = "base_link";
  inlier_cloud_pub.publish(*inlier_cloudptr);

  outlier_cloudptr->header.frame_id = "base_link";
  outlier_cloud_pub.publish(*outlier_cloudptr);
  */

  // now more exactly get the parameters of a cylindrical cluster within the VOI

  /*
  robust_cylinder_fit(*rough_inlier_cloudptr,
		      *inlier_cloudptr,
		      *outlier_cloudptr,
		      cylinder_coefficients,
		      measurement_msg.CircleMaxDistance);

  inlier_cloudptr->header.frame_id = "base_link";
  inlier_cloud_pub.publish(*inlier_cloudptr);

  outlier_cloudptr->header.frame_id = "base_link";
  outlier_cloud_pub.publish(*outlier_cloudptr);
  */


}

// parametrize (if n = 2) or fit (n >= 3) LINE to all ud_cursor points


void EstimateLineCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  EstimateLine();
}

//----------------------------------------------------------------------------

// parametrize (if n = 3) or fit (n >= 4) PLANE to all ud_cursor points

void EstimatePlane(double prism_distance_threshold, bool do_publish)
{
  int i;

  //  bool fit_twice = true; // false;
  //  double hull_scale_factor = 2.0;

  cursor_cloudptr->points.clear();

  for (i = 0; i < ud_cursor_pts.size(); i++)
    cursor_cloudptr->points.push_back(ud_cursor_pts[i]);
  
  // this is just on the cursor points

  robust_plane_fit(*cursor_cloudptr,
		   *inlier_cloudptr,
		   *outlier_cloudptr,
		   *plane_coefficients_ptr,
		   measurement_msg.CursorMaxDistance);

  // now convex hull + proximity to plane forms polygonal prism to roughly clip the data 

  pcl::ConvexHull<pcl::PointXYZ> hull;
  hull.setInputCloud (inlier_cloudptr);
  hull.reconstruct(*hull_cloudptr);

  if (hull.getDimension () != 2) {
    printf("hull points are not 2-D...this is a problem\n");
    return;
  }

  pcl::PointXYZ P_mean;
  P_mean.x = P_mean.y = P_mean.z = 0.0;

  for (i = 0; i < hull_cloudptr->points.size(); i++) {
    P_mean.x += hull_cloudptr->points[i].x;
    P_mean.y += hull_cloudptr->points[i].y;
    P_mean.z += hull_cloudptr->points[i].z;
  }

  P_mean.x /= (double) hull_cloudptr->points.size();
  P_mean.y /= (double) hull_cloudptr->points.size();
  P_mean.z /= (double) hull_cloudptr->points.size();
  
  for (i = 0; i < hull_cloudptr->points.size(); i++) {
    hull_cloudptr->points[i].x = P_mean.x + measurement_msg.PlaneHullScaleFactor * (hull_cloudptr->points[i].x - P_mean.x);
    hull_cloudptr->points[i].y = P_mean.y + measurement_msg.PlaneHullScaleFactor * (hull_cloudptr->points[i].y - P_mean.y);
    hull_cloudptr->points[i].z = P_mean.z + measurement_msg.PlaneHullScaleFactor * (hull_cloudptr->points[i].z - P_mean.z);
  }

  pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
  prism.setInputCloud (input_cloudptr);
  prism.setInputPlanarHull (hull_cloudptr);
  prism.setHeightLimits (-prism_distance_threshold, prism_distance_threshold);
  //  prism.setHeightLimits (-measurement_msg.PlanePrismDistance, measurement_msg.PlanePrismDistance);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // do the clip on entire point cloud

  prism.segment (*inliers);
  
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  
  rough_inlier_cloudptr->clear();
  rough_outlier_cloudptr->clear();
  
  extract.setInputCloud(input_cloudptr->makeShared());
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*rough_inlier_cloudptr);
  
  extract.setNegative (true);
  extract.filter (*rough_outlier_cloudptr);
  
  //  printf("rough polygonal prism: %i in, %i out\n", rough_inlier_cloudptr->points.size(), rough_outlier_cloudptr->points.size());

  // proximity to original plane was loose, and there weren't that many cursor points, so fit again 

  if (measurement_msg.PlaneFitTwice)
    robust_plane_fit(*rough_inlier_cloudptr,
		     *inlier_cloudptr,
		     *outlier_cloudptr,
		     *plane_coefficients_ptr,
		     measurement_msg.PlaneMaxDistance);
  
  //  printf("fine plane: %i in, %i out\n", inlier_cloudptr->points.size(), outlier_cloudptr->points.size()); fflush(stdout);

  // publish inliers and outliers for...

  // ...just points inside convex hull

  if (measurement_msg.PlaneHullCrop) {

    if (do_publish) {

      if (measurement_msg.PlaneFitTwice) {
      
	pcl::toROSMsg(*inlier_cloudptr, inlier_cloud_msg);
	inlier_cloud_msg.header.frame_id = "base_link";
	inlier_cloud_pub.publish(inlier_cloud_msg);
	
	pcl::toROSMsg(*outlier_cloudptr, outlier_cloud_msg);
	outlier_cloud_msg.header.frame_id = "base_link";
	outlier_cloud_pub.publish(outlier_cloud_msg);

      }
      else {

	pcl::toROSMsg(*rough_inlier_cloudptr, inlier_cloud_msg);
	inlier_cloud_msg.header.frame_id = "base_link";
	inlier_cloud_pub.publish(inlier_cloud_msg);
	
	pcl::toROSMsg(*rough_outlier_cloudptr, outlier_cloud_msg);
	outlier_cloud_msg.header.frame_id = "base_link";
	outlier_cloud_pub.publish(outlier_cloud_msg);

      }

      inlier_cloudptr->header.frame_id = "base_link";
      inlier_cloud_pub.publish(*inlier_cloudptr);
      
      outlier_cloudptr->header.frame_id = "base_link";
      outlier_cloud_pub.publish(*outlier_cloudptr);
    }
  }

  // ...entire data set (this is like ExtractPolygonalPrism() without the polygonal boundaries)
  
  else {

    //    printf("a\n"); fflush(stdout);

    if (measurement_msg.PlaneFitTwice)
      plane_slice(*input_cloudptr,
		  *rough_inlier_cloudptr,
		  *rough_outlier_cloudptr,
		  *plane_coefficients_ptr,
		  measurement_msg.PlaneMaxDistance);

    if (do_publish) {

      rough_inlier_cloudptr->width = rough_inlier_cloudptr->height = 0;
      //    printf("b w %i h %i size %i\n", rough_inlier_cloudptr->width,rough_inlier_cloudptr->height,rough_inlier_cloudptr->points.size() ); fflush(stdout);
      
      pcl::toROSMsg(*rough_inlier_cloudptr, inlier_cloud_msg);
      inlier_cloud_msg.header.frame_id = "base_link";
      inlier_cloud_pub.publish(inlier_cloud_msg);
      
      /*
	rough_inlier_cloudptr->header.frame_id = "base_link";
	inlier_cloud_pub.publish(*rough_inlier_cloudptr);
      */
      
      
      //    printf("c\n"); fflush(stdout);
      
      rough_outlier_cloudptr->width = rough_outlier_cloudptr->height = 0;
      
      pcl::toROSMsg(*rough_outlier_cloudptr, outlier_cloud_msg);
      outlier_cloud_msg.header.frame_id = "base_link";
      outlier_cloud_pub.publish(outlier_cloud_msg);

    /*
    rough_outlier_cloudptr->header.frame_id = "base_link";
    outlier_cloud_pub.publish(*rough_outlier_cloudptr);
    */

      shape_msgs::Plane plane_msg;
      plane_msg.coef[0] = plane_coefficients_ptr->values[0];
      plane_msg.coef[1] = plane_coefficients_ptr->values[1];
      plane_msg.coef[2] = plane_coefficients_ptr->values[2];
      plane_msg.coef[3] = plane_coefficients_ptr->values[3];

      plane_pub.publish(plane_msg);

    //    printf("d\n"); fflush(stdout);
    }

  }
}

// parametrize (if n = 3) or fit (n >= 4) PLANE to all ud_cursor points

void EstimatePlaneCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  EstimatePlane(measurement_msg.PlanePrismDistance, true);
}

//----------------------------------------------------------------------------

// parametrize (if n = 3) or fit (n >= 4) 3-D CIRCLE to all ud_cursor points

void EstimateCircle()
{
  EstimatePlane(0.025, false);

  // Create the filtering object

  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (rough_inlier_cloudptr);
  proj.setModelCoefficients (plane_coefficients_ptr);
  proj.filter (*projected_inlier_cloudptr);

  // bring points to Z = 0 plane to do 2-D fit

  transform_to_level(*projected_inlier_cloudptr, *level_inlier_cloudptr, *plane_coefficients_ptr);

  //  double min_radius = 0.15; // 0.1;
  //  double max_radius = 0.3; // 0.2;

  robust_circle2d_fit(*level_inlier_cloudptr,
		      *rough_inlier_cloudptr,
		      *inlier_cloudptr,
		      *outlier_cloudptr,
		      *circle_coefficients_ptr,
		      0.02,
		      measurement_msg.CircleMinRadius, measurement_msg.CircleMaxRadius,
		      true);
  
  /*
  robust_circle2d_fit(*level_inlier_cloudptr,
		      *rough_inlier_cloudptr,
		      *rough_outlier_cloudptr,
		      *circle_coefficients_ptr,
		      0.02,
		      0.1, 0.2);

  // now bring them back up to 3-D again before publishing them

  reverse_transform_to_level(*rough_inlier_cloudptr, *inlier_cloudptr, *plane_coefficients_ptr);
  reverse_transform_to_level(*rough_outlier_cloudptr, *outlier_cloudptr, *plane_coefficients_ptr);
  */

  //  rough_inlier_cloudptr->width = rough_inlier_cloudptr->height = 0;
  //  pcl::toROSMsg(*rough_inlier_cloudptr, inlier_cloud_msg);

  pcl::toROSMsg(*inlier_cloudptr, inlier_cloud_msg);
  inlier_cloud_msg.header.frame_id = "base_link";
  inlier_cloud_pub.publish(inlier_cloud_msg);

  pcl::toROSMsg(*outlier_cloudptr, outlier_cloud_msg);
  outlier_cloud_msg.header.frame_id = "base_link";
  outlier_cloud_pub.publish(outlier_cloud_msg);
     
  level_inlier_cloudptr->points.resize(1);
  level_inlier_cloudptr->points[0].x = circle_coefficients_ptr->values[0];
  level_inlier_cloudptr->points[0].y = circle_coefficients_ptr->values[1];
  level_inlier_cloudptr->points[0].z = 0.0;
  reverse_transform_to_level(*level_inlier_cloudptr, *temp_cloudptr, *plane_coefficients_ptr);

  send_circle_cylinder_marker(temp_cloudptr->points[0].x, temp_cloudptr->points[0].y, temp_cloudptr->points[0].z,
			      *plane_coefficients_ptr,
			      circle_coefficients_ptr->values[2] + 0.02, 0.05,
			      0.0, 1.0, 0.0, 0.5);

}

void EstimateCircleCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  EstimateCircle();
}

//----------------------------------------------------------------------------

void EstimateRigidTransform()
{
  pcl::PointCloud<pcl::PointXYZ> cloud1;
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  Eigen::Matrix4f RT;

  for (int i = 0; i < ud_cursor_pts.size(); i++) {
    if (!(i % 2))
      cloud1.points.push_back(ud_cursor_pts[i]);
    else
      cloud2.points.push_back(ud_cursor_pts[i]);
  }

  estimate_rigid_transform(cloud1, cloud2, RT);
}

//----------------------------------------------------------------------------

void initialize_crop()
{
  fp.x=1;
  fp.y=1;
  fp.z=1;
  cp.x=pp.x=0;
  cp.y=pp.y=0;
  cp.z=pp.z=0;

  minPoint[0]=-999; 
  minPoint[1]=-999; 
  minPoint[2]=-999; 

  maxPoint[0]=999; 
  maxPoint[1]=999; 
  maxPoint[2]=999;  
}

//----------------------------------------------------------------------------

// Crop the pointcloud and republish to ud_output

void Crop()
{


  if (ud_cursor_pts.size() != 2)
    printf("need 2 points to parametrize a cropbox\n");
  else
    {
    minPoint[0]=ud_cursor_pts[0].x; 
    minPoint[1]=ud_cursor_pts[0].y; 
    minPoint[2]=ud_cursor_pts[0].z; 

    maxPoint[0]=ud_cursor_pts[1].x+1; 
    maxPoint[1]=ud_cursor_pts[1].y+1; 
    maxPoint[2]=ud_cursor_pts[1].z+1;  
    
		std::cout << "Max x: " << maxPoint[0] << std::endl;
		std::cout << "Max y: " << maxPoint[1] << std::endl;
		std::cout << "Max z: " << maxPoint[2] << std::endl;
		std::cout << "Min x: " << minPoint[0] << std::endl;
		std::cout << "Min y: " << minPoint[1] << std::endl;
		std::cout << "Min z: " << minPoint[2] << std::endl;
    }
}

void CropCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  Crop();
}

//----------------------------------------------------------------------------

void UndoCrop()
{
    minPoint[0]=-999; 
    minPoint[1]=-999; 
    minPoint[2]=-999; 

    maxPoint[0]=999; 
    maxPoint[1]=999; 
    maxPoint[2]=999;  

}

void UndoCropCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  UndoCrop();
}

//----------------------------------------------------------------------------

// set flag to interpret next click as movement of selected point rather than addition of new point

void MoveSelected()
{
  printf("IM menus deprecated\n"); fflush(stdout);

  /*
  //  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  //  menu_handler.getCheckState( handle, state );

  menu_handler.getCheckState(h_move_entry, state);

  if ( state == MenuHandler::CHECKED ) {
    //    menu_handler.setCheckState( handle, MenuHandler::UNCHECKED );
    menu_handler.setCheckState( h_move_entry, MenuHandler::UNCHECKED );
    ud_cursor_pt_do_move = false;
  }
  else if (ud_cursor_pts.size() > 0) {
    //    menu_handler.setCheckState( handle, MenuHandler::CHECKED );
    menu_handler.setCheckState( h_move_entry, MenuHandler::CHECKED );
    ud_cursor_pt_do_move = true;
  }

  menu_handler.reApply( *server );
  //  ros::Duration(2.0).sleep();
  //  ROS_INFO("update");
  server->applyChanges();

  send_ud_cursor_point_markers();
  */
}

void MoveSelectedCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  MoveSelected();
}

//----------------------------------------------------------------------------

// set flag to draw ud_cursor point indices as text next to spheres

void DisplayIndicesCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  menu_handler.getCheckState( handle, state );

  if ( state == MenuHandler::CHECKED ) {
    menu_handler.setCheckState( handle, MenuHandler::UNCHECKED );
    ud_cursor_pt_display_indices = false;
  }
  else {
    menu_handler.setCheckState( handle, MenuHandler::CHECKED );
    ud_cursor_pt_display_indices = true;
  }

  menu_handler.reApply( *server );
  //  ros::Duration(2.0).sleep();
  //  ROS_INFO("update");
  server->applyChanges();

  send_ud_cursor_point_markers();

}

//----------------------------------------------------------------------------

// set flag to draw line segments connecting ud_cursor point spheres

void DisplayConnectionsCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  menu_handler.getCheckState( handle, state );

  if ( state == MenuHandler::CHECKED ) {
    menu_handler.setCheckState( handle, MenuHandler::UNCHECKED );
    ud_cursor_pt_display_connections = false;
  }
  else {
    menu_handler.setCheckState( handle, MenuHandler::CHECKED );
    ud_cursor_pt_display_connections = true;
  }

  menu_handler.reApply( *server );
  //  ros::Duration(2.0).sleep();
  //  ROS_INFO("update");
  server->applyChanges();

  send_ud_cursor_point_markers();

}

//----------------------------------------------------------------------------

void initMenu()
{
  MenuHandler::EntryHandle entry;

  // MOVE

  h_move_entry = menu_handler.insert( "Move selected point", &MoveSelectedCb );

  /*
  if (ud_cursor_pt_do_move)
    menu_handler.setCheckState( h_move_entry, MenuHandler::CHECKED );
  else
    menu_handler.setCheckState( h_move_entry, MenuHandler::UNCHECKED );
  */

  // DELETION

  h_delete_entry = menu_handler.insert( "Delete..." );

  entry = menu_handler.insert( h_delete_entry, "Selected point", &DeleteSelectedCb);
  entry = menu_handler.insert( h_delete_entry, "All points", &DeleteAllCb );
  
  //  menu_handler.setCheckState( menu_handler.insert( "Show Delete Entry", &enableCb ), MenuHandler::CHECKED );

  // MEASUREMENT

  h_measure_entry = menu_handler.insert( "Measure..." );

  entry = menu_handler.insert( h_measure_entry, "Length", &MeasureLengthCb);
  // entry = menu_handler.insert( h_measure_entry, "Area", &MeasureAreaCb );

  // ESTIMATION

  h_estimate_entry = menu_handler.insert( "Estimate..." );

  entry = menu_handler.insert( h_estimate_entry, "Line", &EstimateLineCb);
  entry = menu_handler.insert( h_estimate_entry, "Plane", &EstimatePlaneCb );
  entry = menu_handler.insert( h_estimate_entry, "Circle", &EstimateCircleCb );

  // Crop

  h_crop_entry = menu_handler.insert( "Crop..." );
  entry = menu_handler.insert( h_crop_entry, "Crop pointcloud", &CropCb);
  entry = menu_handler.insert( h_crop_entry, "Undo", &UndoCropCb);
  // DISPLAY

  h_display_entry = menu_handler.insert( "Display..." );

  entry = menu_handler.insert( h_display_entry, "Indices", &DisplayIndicesCb);
  if (ud_cursor_pt_display_indices)
    menu_handler.setCheckState( entry, MenuHandler::CHECKED );
  else
    menu_handler.setCheckState( entry, MenuHandler::UNCHECKED );

  entry = menu_handler.insert( h_display_entry, "Connections", &DisplayConnectionsCb);
  if (ud_cursor_pt_display_connections)
    menu_handler.setCheckState( entry, MenuHandler::CHECKED );
  else
    menu_handler.setCheckState( entry, MenuHandler::UNCHECKED );


}

//----------------------------------------------------------------------------

void initialize_menu()
{
  server.reset( new InteractiveMarkerServer("menu","",false) );
  initMenu();

  makeMenuMarker( "marker1" );
  menu_handler.apply( *server, "marker1" );
  server->applyChanges();
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

/*
float calc_cp_dist(int i, vector<vector<float> > polypoints)
{
  float cp_dist= sqrt((polypoints[0][i]-polypoints[0][i-1])*(polypoints[0][i]-polypoints[0][i-1])+(polypoints[1][i]-polypoints[1][i-1])*(polypoints[1][i]-polypoints[1][i-1])+(polypoints[2][i]-polypoints[2][i-1])*(polypoints[2][i]-polypoints[2][i-1]));
  return cp_dist;
}
*/

//----------------------------------------------------------------------------

void ptcloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  /*
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	
  cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg (*input, *cloud);
  */

  pcl::fromROSMsg (*input, *input_cloudptr);

  have_scene_cloud = true;

  //do data processing

  if (do_crop) {
    pcl::CropBox<pcl::PointXYZ> cropFilter;
    cropFilter.setInputCloud (input_cloudptr);
    
    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);
    cropFilter.filter (*input_cropped_cloudptr);  
    
    // Publish the data
    ptcloud_pub.publish (*input_cropped_cloudptr);
  }

/*
pcl::fromROSMsg(*input, rviz_pt);
sensor_msgs::PointCloud2 cloud_filtered;

// Perform the actual filtering
pcl::VoxelGrid<pcl::PointXYZRGB> sor ;
sor.setInputCloud (rviz_pt);
sor.setLeafSize (0.01, 0.01, 0.01);
sor.filter (rviz_pt_filtered);

//Convert the pcl cloud back to rosmsg
pcl::toROSMsg(*rviz_pt_filtered, cloud_filtered);
//Set the header of the cloud
cloud_filtered.header.frame_id = input->header.frame_id;
// Publish the data
//You may have to set the header frame id of the cloud_filtered also
pub.publish (cloud_filtered);

*/
}

//----------------------------------------------------------------------------

//This function downsamples the pointcloud and converts it to pcl::PolygonMesh using fast trangulation, needs another panel to define
//saving path for the mesh file, and mesh reconstruction parameters.

void click_pt2mesh_triangulation(sensor_msgs::PointCloud2::Ptr rawpt)
{


//Voxel filter

// Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


  sensor_msgs::PointCloud2 cloud_blob;
  sensor_msgs::PointCloud2::Ptr input (new sensor_msgs::PointCloud2 ());
  sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2 ());


  input= rawpt;

 //For testing

 // pcl::PCDReader reader;
 
 // reader.read ("./armory_stairs1_hokuyo.pcd", *input);


  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (input);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (*output);

 //For testing
  pcl::PCDWriter writer;
  writer.write ("./downsampled.pcd", *output, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

//Downsampling completed!

  cloud_blob=*output;

//Moving Least Squares smooth on downsampled pointcloud

  pcl::io::loadPCDFile ("./downsampled.pcd", *cloud);
  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.process (mls_points);

  // Save output
  pcl::io::savePCDFile ("./smoothed.pcd", mls_points);


//Mesh reconstruction by fast triangulation


//Comment this line to disable smooth
  pcl::io::loadPCDFile ("./smoothed.pcd", cloud_blob);
  pcl::fromROSMsg (cloud_blob, *cloud);


//* the data should be available in cloud
// Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree2);
	n.setKSearch (20);
	n.compute (*normals);
//* normals should not contain the point normals + surface curvatures

// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
//* cloud_with_normals = cloud + normals

// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree3 (new pcl::search::KdTree<pcl::PointNormal>);
	tree3->setInputCloud (cloud_with_normals);

// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (3);

// Set typical values for the parameters
	gp3.setMu (3);
	gp3.setMaximumNearestNeighbors (300);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree3);
	gp3.reconstruct (triangles);

// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

//For testing (save mesh files)
        //pcl::io::savePLYFile("trangulation.ply", mesh);
	pcl::io::saveVTKFile("trangulation.vtk",triangles);


}

//----------------------------------------------------------------------------

//This function downsamples the pointcloud and converts it to mesh using poisson surface reconstruction
void click_pt2mesh_poisson(sensor_msgs::PointCloud2::Ptr rawpt)
{

//Voxel filter

// Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


  sensor_msgs::PointCloud2 cloud_blob;
  sensor_msgs::PointCloud2::Ptr input (new sensor_msgs::PointCloud2 ());
  sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2 ());


  input= rawpt;

 //For testing

 // pcl::PCDReader reader;
 
 // reader.read ("./armory_stairs1_hokuyo.pcd", *input);


  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (input);
  sor.setLeafSize (0.001, 0.001, 0.001);
  sor.filter (*output);

 //For testing
  pcl::PCDWriter writer;
  writer.write ("./downsampled.pcd", *output, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

//Downsampling completed

	cloud_blob=*output;

  pcl::io::loadPCDFile ("./downsampled.pcd", *cloud);

//PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);


//Begin passthrough filter
//Pointcloud loaded
  PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
  PassThrough<PointXYZ> filter;
  filter.setInputCloud(cloud);
  filter.filter(*filtered);
//Passthrough filter complete

//Uncomment if you want to do mls smoothing
      // cout << "begin moving least squares" << endl;
      // MovingLeastSquares<PointXYZ, PointXYZ> mls;
      // mls.setInputCloud(filtered);
      // mls.setSearchRadius(0.01);
      // mls.setPolynomialFit(true);
      // mls.setPolynomialOrder(2);
      // mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
      // mls.setUpsamplingRadius(0.005);
      // mls.setUpsamplingStepSize(0.003);

      // PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ>());
      // mls.process(*cloud_smoothed);
      // cout << "MLS complete" << endl;

//Begin normal estimation
  NormalEstimationOMP<PointXYZ, Normal> ne;
  ne.setNumberOfThreads(8);
  ne.setInputCloud(filtered);
  ne.setRadiusSearch(0.01);
  Eigen::Vector4f centroid;
  compute3DCentroid(*filtered, centroid);
  ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

  PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
  ne.compute(*cloud_normals);

//Normal estimation complete

//Reverse normals' direction

  for(size_t i = 0; i < cloud_normals->size(); ++i){
  cloud_normals->points[i].normal_x *= -1;
  cloud_normals->points[i].normal_y *= -1;
  cloud_normals->points[i].normal_z *= -1;
  }

//Combine points and normals
  PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
  concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

//Begin poisson reconstruction
  Poisson<PointNormal> poisson;
  poisson.setDepth(10);
  poisson.setInputCloud(cloud_smoothed_normals);
  PolygonMesh mesh;
  poisson.reconstruct(mesh);
//Save mesh file
  //io::savePLYFile("poisson.ply", mesh);
  pcl::io::saveVTKFile("poisson.vtk",mesh);

}


//----------------------------------------------------------------------------
//This function downsamples the pointcloud and converts it to mesh using matching cube algorithm
void click_pt2mesh_cube(sensor_msgs::PointCloud2::Ptr rawpt)
{
//Voxel filter

// Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


  sensor_msgs::PointCloud2 cloud_blob;
  sensor_msgs::PointCloud2::Ptr input (new sensor_msgs::PointCloud2 ());
  sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2 ());


  input= rawpt;

 //For testing

 // pcl::PCDReader reader;
 
 // reader.read ("./armory_stairs1_hokuyo.pcd", *input);


  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (input);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (*output);

 //For testing
  pcl::PCDWriter writer;
  writer.write ("./downsampled.pcd", *output, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

//Downsampling completed

	cloud_blob=*output;

  pcl::io::loadPCDFile ("./downsampled.pcd", *cloud);

//Pointcloud loaded

  NormalEstimationOMP<PointXYZ, Normal> ne;
  search::KdTree<PointXYZ>::Ptr tree1 (new search::KdTree<PointXYZ>);
  tree1->setInputCloud (cloud);
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree1);
  ne.setKSearch (20);
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
  ne.compute (*normals);
         
// Concatenate the XYZ and normal fields
  PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
  concatenateFields(*cloud, *normals, *cloud_with_normals);
 
// Create search tree
  search::KdTree<PointNormal>::Ptr tree (new search::KdTree<PointNormal>);
  tree->setInputCloud (cloud_with_normals);
    
//Begin marching cubes reconstruction

  MarchingCubesRBF<PointNormal> mc;
  PolygonMesh::Ptr triangles(new PolygonMesh);
  mc.setInputCloud (cloud_with_normals);
  mc.setSearchMethod (tree);
  mc.reconstruct (*triangles);
//Save mesh file
  //io::savePLYFile("cube.ply", *triangles);
  io::saveVTKFile("cube.vtk", *triangles);
}


//----------------------------------------------------------------------------

//Plane fitting on user clicked 3D point coordinates, works directly on the global vector array: polypoints
//Also need a panel to tell the function when to stop
void click_segplane()
{


// clicked_plane plane estimation:
  PointCloudT::Ptr clicked_points_3d (new PointCloudT);
  Eigen::VectorXf clicked_plane_coeffs;
  clicked_plane_coeffs.resize(4);
  std::vector<int> clicked_points_indices;
  for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
//Pass the value of polypoints to clicked_points_3d
    clicked_points_indices.push_back(i);

  pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
  model_plane.computeModelCoefficients(clicked_points_indices,clicked_plane_coeffs);
  std::cout << "clicked_plane plane: " << clicked_plane_coeffs(0) << " " << clicked_plane_coeffs(1) << " " << clicked_plane_coeffs(2) << " " << clicked_plane_coeffs(3) << std::endl;

}

//----------------------------------------------------------------------------

// is the point clicked already in our list?

bool check_selected(pcl::PointXYZ P, int & index)
{
  double dx, dy, dz;
  double dist_squared, r_squared;
  bool return_val = false;

  r_squared = ud_cursor_pt_radius * ud_cursor_pt_radius;

  for (int i = 0; i < ud_cursor_pts.size(); i++) {

    dx = P.x - ud_cursor_pts[i].x;
    dy = P.y - ud_cursor_pts[i].y;
    dz = P.z - ud_cursor_pts[i].z;

    dist_squared = dx*dx + dy*dy + dz*dz;

    if (dist_squared < r_squared) {
      index = i;
      return_val = true;
      r_squared = dist_squared;
    }      
  }

  return return_val;
}

//----------------------------------------------------------------------------

// this gets called when a new ud_cursor point comes in

void cursorCallback(const ud_cursor::UDCursor& msg)
{
  pcl::PointXYZ P;

  P.x = msg.x;
  P.y = msg.y;
  P.z = msg.z;

  if (msg.DisplayIndices) {

    ud_cursor_pt_display_indices = !ud_cursor_pt_display_indices;

  }

  else if (msg.DisplayConnections) {

    ud_cursor_pt_display_connections = !ud_cursor_pt_display_connections;

  }


  // MOVE mode : currently selected point goes where this message says

  else if (msg.EditMode == UD_CURSOR_EDIT_MODE_MOVE) {

    if (ud_cursor_pt_selection_index != NONE_SELECTED) {

      printf("Moving\n");

      ud_cursor_pts[ud_cursor_pt_selection_index] = P;

    }

  }

  // DELETE selected point

  else if (msg.EditMode == UD_CURSOR_EDIT_MODE_DELETE) {

    if (check_selected(P, ud_cursor_pt_selection_index)) {

      printf("Deleting\n");

      if (ud_cursor_pt_selection_index >= 0 && ud_cursor_pt_selection_index < ud_cursor_pts.size()) {
    
	ud_cursor_pts.erase(ud_cursor_pts.begin() + ud_cursor_pt_selection_index);

	// make new selected point one after the one we just deleted.  if that one was the last, make 
	// new last the selected point

	if (ud_cursor_pt_selection_index >= ud_cursor_pts.size())
	  ud_cursor_pt_selection_index--;
    
	if (ud_cursor_pts.size() == 0) 
	  ud_cursor_pt_selection_index = NONE_SELECTED;

      }
    }
  }

  // DELETE ALL points

  else if (msg.EditMode == UD_CURSOR_EDIT_MODE_DELETE_ALL) {

    printf("Deleting all\n");

    ud_cursor_pts.clear();

    ud_cursor_pt_selection_index = NONE_SELECTED;

  }

  // ADD point (which can also mean select)

  else {

    // ADD

    if (!check_selected(P, ud_cursor_pt_selection_index)) {

      printf("Adding\n");
      
      //    ud_cursor_pts.push_back(P);
      ud_cursor_pt_selection_index++;
      ud_cursor_pts.insert(ud_cursor_pts.begin() + ud_cursor_pt_selection_index, P);
      
      // last point added is automatically selected
      
      //    ud_cursor_pt_selection_index = ud_cursor_pts.size() - 1;

      if (ud_cursor_pt_selection_index == 0) {

	geometry_msgs::Pose pose;
 
	pose.position.x = P.x + 5.0*ud_cursor_pt_radius;
	pose.position.y = P.y + 5.0*ud_cursor_pt_radius;
	pose.position.z = P.z + 5.0*ud_cursor_pt_radius;

	server->setPose( "marker1", pose );
	server->applyChanges();

      }
    }

    // SELECT -- don't do anything because it already happened in check_selected()

    else {
      //    printf("selected %i!\n", ud_cursor_pt_selection_index);
    }
  }

  //  printf("add: %i points\n", ud_cursor_pts.size());

  send_ud_cursor_point_markers();

}

//----------------------------------------------------------------------------

// this gets called when a new ud_cursor point comes in

// note that the edit_mode has been written into header.seq

void clickCallback(const geometry_msgs::PointStamped& msg)
{
  printf("clickCallback() deprecated\n");


  /*

  // hack of the day club...you're welcome
  ud_cursor_edit_mode = msg.header.seq;
  
  if (ud_cursor_edit_mode == UD_CURSOR_EDIT_MODE_ADD) 
    printf("A\n");
  else if (ud_cursor_edit_mode == UD_CURSOR_EDIT_MODE_MOVE) 
    printf("M\n");
  else if (ud_cursor_edit_mode == UD_CURSOR_EDIT_MODE_DELETE) 
    printf("D\n");


  pcl::PointXYZ P;

  P.x = msg.point.x;
  P.y = msg.point.y;
  P.z = msg.point.z;

  //  if (ud_cursor_pt_do_move) {
  if (ud_cursor_edit_mode == UD_CURSOR_EDIT_MODE_MOVE) {

    ud_cursor_pts[ud_cursor_pt_selection_index] = P;

  }
  else {
    if (!check_selected(P, ud_cursor_pt_selection_index)) {
      
      //    ud_cursor_pts.push_back(P);
      ud_cursor_pt_selection_index++;
      ud_cursor_pts.insert(ud_cursor_pts.begin() + ud_cursor_pt_selection_index, P);
      
      // last point added is automatically selected
      
      //    ud_cursor_pt_selection_index = ud_cursor_pts.size() - 1;

      if (ud_cursor_pt_selection_index == 0) {

	geometry_msgs::Pose pose;
 
	pose.position.x = P.x + 5.0*ud_cursor_pt_radius;
	pose.position.y = P.y + 5.0*ud_cursor_pt_radius;
	pose.position.z = P.z + 5.0*ud_cursor_pt_radius;

	server->setPose( "marker1", pose );
	server->applyChanges();

      }
    }
    else {
      //    printf("selected %i!\n", ud_cursor_pt_selection_index);
    }
  }

  //  printf("add: %i points\n", ud_cursor_pts.size());

  send_ud_cursor_point_markers();
  */
}

//----------------------------------------------------------------------------


void panelCallback(const ud_measurement_panel::MeasurementCommand& msg)
{
  measurement_msg = msg;

  // parameters

  // commands -- only one should be true

  /*
  if (msg.MoveSelected == 1) {
    MoveSelected();
    return;
  }
  else if (msg.DeleteAll == 1) {
    DeleteAll();
    return;
  }
  else if (msg.DeleteSelected == 1) {
    DeleteSelected();
    return;
  }
  */
  if (msg.EstimatePlane == 1) {
    EstimatePlane(measurement_msg.PlanePrismDistance, true);
    return;
  }
  else if (msg.EstimateLine == 1) {
    EstimateLine();
    return;
  }
  else if (msg.EstimateCircle == 1) {
    EstimateCircle();
    return;
  }
  else if (msg.EstimateRigidTransform == 1) {
    EstimateRigidTransform();
    return;
  }
  else if (msg.MeasureLength == 1) {
    MeasureLength();
    return;
  }
  else if (msg.Crop == 1) {
    Crop();
    return;
  }
  else if (msg.Undo == 1) {
    UndoCrop();
    return;
  }
  else {
    cout << "Didn't understand the measurement_command ros msg." << endl;
  }
}

//----------------------------------------------------------------------------

void initialize_ros()
{
  // Publishers

  plane_pub = nhp->advertise<shape_msgs::Plane>("reference_plane", 10);

  marker_pub = nhp->advertise<visualization_msgs::Marker>("visualization_marker", 10);
  marker_array_pub = nhp->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

  //  rough_cylinder_marker_pub = nhp->advertise<visualization_msgs::Marker>("rough_cylinder_marker", 10);
  cylinder_marker_pub = nhp->advertise<visualization_msgs::Marker>("cylinder_marker", 10);

  ptcloud_pub = nhp->advertise<pcl::PointCloud<pcl::PointXYZ> > ("ud_output", 1);

  advertise_scene_pointcloud();
  advertise_object_pointcloud();

  inlier_cloud_pub = nhp->advertise<pcl::PointCloud<pcl::PointXYZ> > ("ud_inliers", 1);
  outlier_cloud_pub = nhp->advertise<pcl::PointCloud<pcl::PointXYZ> > ("ud_outliers", 1);

  // Subscribers

  cursor_sub = nhp->subscribe("ud_cursor_point", 10, cursorCallback);

  click_sub = nhp->subscribe("ud_clicked_point", 10, clickCallback);

  ptcloud_sub = nhp->subscribe("cloud_pcd", 10, ptcloudCallback);
  
  //add a subscriber to listen for things that are clicked on in the measurement panel
  measurement_sub = nhp->subscribe("measurement_command", 10, panelCallback);

}

//----------------------------------------------------------------------------

int main( int argc, char** argv )
{
  printf("ud_imarker server starting\n"); fflush(stdout);

  // options to locally load .pcd files instead of subscribing to what pcd_to_pointcloud puts out.
  // this should be done with ros params, etc., but i'm too lazy to deal with that right now -- for now
  // it works to just write "rosrun ud_imarker UD_rviz_interaction -armory_golf -kinfu_vehicle"

  for (int i = 0; i < argc; i++) {
    
    // only one of these should be chosen

    /*
    if (!strcmp(argv[i], "-scene_xyzi")) {
      do_load_scene_xyzi = true;
      scene_xyzi_filename = argv[i+1];
    }
    else if (!strcmp(argv[i], "-scene_xyz")) {
      do_load_scene_xyz = true;
      scene_xyz_filename = argv[i+1];
    }
    */

    if (!strcmp(argv[i], "-armory_golf"))
      do_load_armory_golf = true;
    else if (!strcmp(argv[i], "-armory_valves1"))
      do_load_armory_valves1 = true;
    else if (!strcmp(argv[i], "-tepra_ds"))
      do_load_tepra_DS = true;
    else if (!strcmp(argv[i], "-tepra_ps"))
      do_load_tepra_PS = true;

    // this can be chosen independently 

    else if (!strcmp(argv[i], "-kinfu_vehicle"))
      do_load_kinfu_vehicle = true;
  }

  ros::init(argc, argv, "UD_rviz_interaction");
  nhp = new ros::NodeHandle();

  initialize_pcl();
  initialize_crop();
  initialize_menu();

  initialize_ros();
  
  ros::Rate UD_rviz_interaction_rate(30);

  double publish_interval = 1.0;
  double last_timestamp = ros::Time::now().toSec();

  while (ros::ok())
  {
    UD_rviz_interaction_rate.sleep();

    double now_timestamp = ros::Time::now().toSec();

    if ((now_timestamp - last_timestamp) >= publish_interval) {

      //      printf("publishing!\n"); fflush(stdout);
      last_timestamp = now_timestamp;

      if (have_scene_cloud)
	input_cloud_pub.publish(input_cloud_msg);

      if (have_object_cloud)
	object_cloud_pub.publish(object_cloud_msg);

    }

    ros::spinOnce();
  }
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
