//RVIZ measurement tool
//Qiaosong Wang
//University of Delaware
//qiaosong@udel.edu

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>


#include <boost/foreach.hpp>
#include <boost/bind.hpp>

#include <cmath>
#include <vector>
#include <math.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


using namespace std;
using namespace visualization_msgs;
using namespace interactive_markers;

boost::shared_ptr<InteractiveMarkerServer> server;
float marker_pos = 0;

MenuHandler menu_handler;

MenuHandler::EntryHandle h_first_entry;
MenuHandler::EntryHandle h_mode_last;

//changing modes
int workmode = 0; //0- distance, 1 polyline mode 2 box selection 3- selct 3 points to find a plane (BMF)



//Defining ROS parameters
ros::Publisher marker_pub;
ros::Publisher ptcloud_pub;
ros::NodeHandle *nhp;
ros::Subscriber click_sub;
ros::Subscriber ptcloud_sub;

float cpdist=0; //Current frame
float globalscale = 1;  //What are the units on this? -Brad
geometry_msgs::Point fp; //future point
geometry_msgs::Point cp; //current point
geometry_msgs::Point pp; //previous point

//Number of points, need to be dynamically assiagned once the panel is done
int num_polypoints=100;
int polycount = 0;

//Declare 3D point storage structure
vector<vector<float> > polypoints(3, vector<float>(100));

//Declare received pointcloud from RVIZ
pcl::PointCloud< pcl::PointXYZRGB> rviz_pt;
pcl::PointCloud< pcl::PointXYZRGB> rviz_pt_filtered;

// menu code taken from here:
// http://ftp.isr.ist.utl.pt/pub/roswiki/rviz(2f)Tutorials(2f)Interactive(20)Markers(3a20)Getting(20)Started.html#menu

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

void modeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  menu_handler.setCheckState( h_mode_last, MenuHandler::UNCHECKED );
  h_mode_last = feedback->menu_entry_id;
  menu_handler.setCheckState( h_mode_last, MenuHandler::CHECKED );

  ROS_INFO("Switching to menu entry #%d", h_mode_last);

  menu_handler.reApply( *server );
  server->applyChanges();
}

Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarker makeEmptyMarker( bool dummyBox=true )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/base_link";
  int_marker.pose.position.y = -3.0 * marker_pos++;;
  int_marker.scale = 1;

  return int_marker;
}

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

void deepCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO("The deep sub-menu has been found.");
}

void initMenu()
{
  h_first_entry = menu_handler.insert( "First Entry" );
  MenuHandler::EntryHandle entry = menu_handler.insert( h_first_entry, "deep" );
  entry = menu_handler.insert( entry, "sub" );
  entry = menu_handler.insert( entry, "menu", &deepCb );
  
  menu_handler.setCheckState( menu_handler.insert( "Show First Entry", &enableCb ), MenuHandler::CHECKED );

  MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Switch" );

  for ( int i=0; i<5; i++ )
  {
    std::ostringstream s;
    s << "Mode " << i;
    h_mode_last = menu_handler.insert( sub_menu_handle, s.str(), &modeCb );
    menu_handler.setCheckState( h_mode_last, MenuHandler::UNCHECKED );
  }
  //check the very last entry
  menu_handler.setCheckState( h_mode_last, MenuHandler::CHECKED );
}



float calc_cp_dist(int i, vector<vector<float> > polypoints)
{
  float cp_dist= sqrt((polypoints[0][i]-polypoints[0][i-1])*(polypoints[0][i]-polypoints[0][i-1])+(polypoints[1][i]-polypoints[1][i-1])*(polypoints[1][i]-polypoints[1][i-1])+(polypoints[2][i]-polypoints[2][i-1])*(polypoints[2][i]-polypoints[2][i-1]));
  return cp_dist;
}


void ptcloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{

  //do the pointcloud processing
  sensor_msgs::PointCloud2 output= *input;
  // Publish the data
  ptcloud_pub.publish (output);

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


//This function returns downsampled pointcloud and convert it to pcl::PolygonMesh, needs another panel to define
//saving path for the mesh file, and mesh reconstruction parameters.
sensor_msgs::PointCloud2::Ptr click_pt2mesh(sensor_msgs::PointCloud2::Ptr rawpt)
{


///////////////////Voxel filter///////////////////////////////////

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
 writer.write ("./downsampled.pcd", *output,
       Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

printf("Downsampling completed!\n");

cloud_blob=*output;

///////////////////Moving Least Squares smooth on downsampled pointcloud///////////////////////////////////


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


///////////////////Mesh reconstruction by fast triangulation///////////////////////////////////


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

//For testing
pcl::io::saveVTKFile("mesh.vtk",triangles);

return output;
}


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



void clickCallback(const geometry_msgs::PointStamped& msg)
{
  if(workmode == 3)  //plane finding
  {
  
  //Getting ROS geometry msgs
  //ROS_INFO("New Point at X: %f, Y: %f, Z: %f\n", msg.point.x, msg.point.y, msg.point.z);
  fp.x = msg.point.x;
  fp.y = msg.point.y;
  fp.z = msg.point.z;
  
  //Need to extend to allow 3 clicks which determine a plane
  
  //marker
  
  visualization_msgs::Marker plane1;
  plane1.header.frame_id = "base_link";
  plane1.header.stamp = ros::Time();
  plane1.ns = "UD_rviz_interaction";
  plane1.id = 0;
  plane1.type = visualization_msgs::Marker::CUBE;
  plane1.action = visualization_msgs::Marker::ADD;
  plane1.pose.position.x = fp.x;
  plane1.pose.position.y = fp.y; 
  plane1.pose.position.z = fp.z;
  //need to determine plane orientation
  plane1.pose.orientation.x = 0.0;
  plane1.pose.orientation.y = 0.0;
  plane1.pose.orientation.z = 0.0;
  plane1.pose.orientation.w = 1.0;
  plane1.scale.x = 1;
  plane1.scale.y = 1;
  plane1.scale.z = 0.1;
  plane1.color.a = 1.0;
  plane1.color.r = 0.0;
  plane1.color.g = 1.0;
  plane1.color.b = 0.0;
  //only if using a MESH_RESOURCE marker type:
  //UD_rviz_interaction.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  marker_pub.publish( plane1 );
	  
  }
  if(workmode == 0)  //distance finding
  {

  //Getting ROS geometry msgs
  //ROS_INFO("New Point at X: %f, Y: %f, Z: %f\n", msg.point.x, msg.point.y, msg.point.z);
  fp.x = msg.point.x;
  fp.y = msg.point.y;
  fp.z = msg.point.z;
  

  // Initialize marker parameters
  visualization_msgs::Marker points, line_strip, line_list , psphere, csphere, arrow, showtext,showtextid;
  showtext.header.frame_id = arrow.header.frame_id=psphere.header.frame_id=csphere.header.frame_id = points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = showtextid.header.frame_id = "/base_link";
  showtext.header.stamp = arrow.header.stamp=psphere.header.stamp =csphere.header.stamp= points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  showtext.ns = arrow.ns=psphere.ns=csphere.ns=points.ns = line_strip.ns = line_list.ns = "UD_rviz_interaction";
  showtext.action = arrow.action=psphere.action=csphere.action=points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  showtext.pose.orientation.w = arrow.pose.orientation.w=psphere.pose.orientation.w =csphere.pose.orientation.w= points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
  
  //ID
  points.id = 0;
  line_strip.id = 1;
  line_list.id = 2;
  psphere.id =3;
  csphere.id =4;
  arrow.id=5;
  showtext.id=6;
  showtextid.id=7;
  
  //Type
  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  psphere.type = visualization_msgs::Marker::SPHERE;
  csphere.type = visualization_msgs::Marker::SPHERE;
  arrow.type = visualization_msgs::Marker::ARROW;
  showtext.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  showtextid.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  
  //Scale
  psphere.scale.x = 0.05*globalscale;
  psphere.scale.y = 0.05*globalscale;
  psphere.scale.z = 0.05*globalscale;
  
  csphere.scale.x = 0.05*globalscale;
  csphere.scale.y = 0.05*globalscale;
  csphere.scale.z = 0.05*globalscale;
  
  showtext.scale.x = 0.05*globalscale;
  showtext.scale.y = 0.05*globalscale;
  showtext.scale.z = 0.05*globalscale;
  
  showtextid.scale.x = 0.05*globalscale;
  showtextid.scale.y = 0.05*globalscale;
  showtextid.scale.z = 0.05*globalscale;
  
  line_list.scale.x = 0.005*globalscale;
  line_list.scale.y = 0.005*globalscale;
  line_list.scale.z = 0.005*globalscale;
  
  line_strip.scale.x = 0.005*globalscale;
  
  //Color
  points.color.b = 1.0;
  points.color.a = 1.0;
  
  // Line strip is blue
  line_list.color.b = 1.0;
  line_list.color.a = 1.0;
  
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;
  
  // Sphere is green
  psphere.color.r = 1.0;
  psphere.color.a = 0.6;
  
  csphere.color.g = 1.0;
  csphere.color.a = 1.0;
  
  //Text is green
  showtext.color.r = 1.0;
  showtext.color.g = 1.0;
  showtext.color.a = 1.0;
  
  showtextid.color.r = 0.0;
  showtextid.color.g = 1.0;
  showtextid.color.a = 1.0;
  
  cp.x=fp.x;
  cp.y=fp.y;
  cp.z=fp.z;
  
  psphere.points.push_back(pp);
  psphere.pose.position.x = pp.x;
  psphere.pose.position.y = pp.y;
  psphere.pose.position.z = pp.z;
  
  csphere.points.push_back(cp);
  csphere.pose.position.x = cp.x;
  csphere.pose.position.y = cp.y;
  csphere.pose.position.z = cp.z;
  
  
  // Create the vertices for the points and lines
  
  polypoints[0][polycount]=fp.x;
  polypoints[1][polycount]=fp.y;
  polypoints[2][polycount]=fp.z;
  
  
  for (int i = 0; i<=polycount;i++) {
    
    geometry_msgs::Point p;
    p.x = polypoints[0][i];
    p.y = polypoints[1][i];
    p.z = polypoints[2][i];
    /*
printf("%f\n", polypoints[0][i]);
printf("%f\n", polypoints[1][i]);
printf("%f\n", polypoints[2][i]);
*/
    points.points.push_back(p);
    line_strip.points.push_back(p);
    
    
    std::stringstream point_id;
    point_id<< i << "\n";
    
    printf("%d\n", i);
    
    showtextid.text = point_id.str();
    showtextid.pose.position.x = polypoints[0][i];
    showtextid.pose.position.y = polypoints[1][i];
    showtextid.pose.position.z = polypoints[2][i];
    
    // line_list.points.push_back(p);
    // p.z += 1.0;
    //line_list.points.push_back(p);
    if(i>0)
      cpdist=cpdist+calc_cp_dist(i,polypoints);
  }
  
  pp.x = cp.x;
  pp.y = cp.y;
  pp.z = cp.z;
  
  printf("Distance: %f \n", cpdist);
  
  std::stringstream show_dist;
  show_dist<< cpdist << "\n";
  
  showtext.text = show_dist.str();
  showtext.pose.position.x = polypoints[0][polycount];
  showtext.pose.position.y = polypoints[1][polycount];
  showtext.pose.position.z = polypoints[2][polycount];
  
  // Publish markers
  marker_pub.publish(line_list);
  marker_pub.publish(psphere);
  marker_pub.publish(csphere);
  marker_pub.publish(showtext);
  marker_pub.publish(showtextid);
  marker_pub.publish(line_strip);
  
  
  if (polycount!=num_polypoints)
    polycount++;
  else {
    polycount=0;
    cpdist=0;
  }
  
} //end of workmode 0 code (distance finding)
  
}



int main( int argc, char** argv )
{

  fp.x=1;
  fp.y=1;
  fp.z=1;
  cp.x=pp.x=0;
  cp.y=pp.y=0;
  cp.z=pp.z=0;

  printf("ud_imarker server starting\n"); fflush(stdout);

  ros::init(argc, argv, "UD_rviz_interaction");

  server.reset( new InteractiveMarkerServer("menu","",false) );
  initMenu();

  makeMenuMarker( "marker1" );
  // makeMenuMarker( "marker2" );

  menu_handler.apply( *server, "marker1" );
  // menu_handler.apply( *server, "marker2" );
  server->applyChanges();

   nhp= new ros::NodeHandle();

  //Publishers
  marker_pub = nhp->advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ptcloud_pub = nhp->advertise<sensor_msgs::PointCloud2> ("ud_output", 1);

  //Subscribers
  click_sub = nhp->subscribe("ud_clicked_point", 10,clickCallback);


  ptcloud_sub = nhp->subscribe("cloud_pcd",10,ptcloudCallback);



  ros::Rate UD_rviz_interaction_rate(30);

  while (ros::ok())
  {
    UD_rviz_interaction_rate.sleep();
    ros::spinOnce();
  }
}
