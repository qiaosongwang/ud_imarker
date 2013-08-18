//RVIZ measurement tool
//Qiaosong Wang
//University of Delaware
//qiaosong@udel.edu

//----------------------------------------------------------------------------

#include "ud_imarker.hh"
#include "ud_measurement_panel/MeasurementCommand.h"

// rosrun pcl_ros pcd_to_pointcloud golfcart_pillar1_hokuyo.pcd 1

//----------------------------------------------------------------------------

// this should change with menu selection

bool do_crop = false;

//----------------------------------------------------------------------------

pcl::PointCloud<pcl::PointXYZ>::Ptr cursor_cloudptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloudptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr input_cropped_cloudptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr rough_inlier_cloudptr;
pcl::PointCloud<pcl::PointXYZ>::Ptr rough_outlier_cloudptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloudptr;
pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloudptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloudptr;

ros::Publisher inlier_cloud_pub;
ros::Publisher outlier_cloud_pub;

pcl::ModelCoefficients rough_plane_coefficients;
pcl::ModelCoefficients rough_line_coefficients;

pcl::ModelCoefficients plane_coefficients;
pcl::ModelCoefficients line_coefficients;

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

//changing modes
int workmode = 0; //0- distance, 1 polyline mode 2 box selection 3- select 3 points to find a plane

//global points
Eigen::Vector4f minPoint;
Eigen::Vector4f maxPoint;
 

double rough_max_cylinder_radius = 0.1; // 05;
double max_cylinder_radius = 0.05; // 0.03

double cylinder_inlier_distance_threshold = 0.025;

double rough_max_plane_distance = 0.1;
double max_plane_distance = 0.025;

double ransac_inlier_distance_threshold = 0.05;
double ransac_inlier_distance_threshold_delta = 0.01;

//Defining ROS parameters

//ros::Publisher rough_cylinder_marker_pub;
ros::Publisher cylinder_marker_pub;

ros::Publisher marker_pub;
ros::Publisher marker_array_pub;

ros::Publisher ptcloud_pub;

ros::Publisher inliers_cloud_pub;
ros::Publisher outliers_cloud_pub;

ros::NodeHandle *nhp;
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
int ud_cursor_pt_selection_index = -1;
bool ud_cursor_pt_display_indices = false;
bool ud_cursor_pt_display_connections = false;
bool ud_cursor_pt_do_move = false;

//Declare received pointcloud from RVIZ
pcl::PointCloud< pcl::PointXYZRGB> rviz_pt;
pcl::PointCloud< pcl::PointXYZRGB> rviz_pt_filtered;

//----------------------------------------------------------------------------

void initialize_pcl()
{
  cursor_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);

  input_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);

  input_cropped_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);

  inlier_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
  outlier_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);

  rough_inlier_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
  rough_outlier_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);

  hull_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

//----------------------------------------------------------------------------

void send_cylinder_marker(pcl::ModelCoefficients & coefficients,
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
	 coefficients.values[3],
	 coefficients.values[4],
	 coefficients.values[5],
	 coefficients.values[3]*coefficients.values[3]+coefficients.values[4]*coefficients.values[4]+coefficients.values[5]*coefficients.values[5],
	 min_xp,
	 max_xp);

  cyl.pose.position.x = coefficients.values[0] + 0.5 * (max_xp + min_xp) * coefficients.values[3];
  cyl.pose.position.y = coefficients.values[1] + 0.5 * (max_xp + min_xp) * coefficients.values[4];
  cyl.pose.position.z = coefficients.values[2] + 0.5 * (max_xp + min_xp) * coefficients.values[5];

  //  tf::Quaternion q = shortest_rotation_quaternion(-coefficients.values[3], -coefficients.values[4], -coefficients.values[5], 0, 0, 1);
  tf::Quaternion q = shortest_rotation_quaternion(0, 0, 1,
						  coefficients.values[3], coefficients.values[4], coefficients.values[5]);
 
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
      color.g = 0.5;
      color.b = 0.0;
      color.a = 0.6;
    }
    else {
      color.r = 0.0;
      color.g = 1.0;
      color.b = 0.0;
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
}

void DeleteSelectedCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  DeleteSelected();
}

//----------------------------------------------------------------------------

// get rid of every ud_cursor point we have stored

void DeleteAll()
{
  ud_cursor_pts.clear();

  ud_cursor_pt_selection_index = -1;

  ud_cursor_pt_do_move = false;
  menu_handler.setCheckState( h_move_entry, MenuHandler::UNCHECKED );
  menu_handler.reApply( *server );
  server->applyChanges();

  //  printf("delete all: %i points\n", ud_cursor_pts.size());

  send_ud_cursor_point_markers();

  //  ROS_INFO("The delete all sub-menu has been found.");
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

void IncreaseInlierDistanceThreshCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ransac_inlier_distance_threshold += ransac_inlier_distance_threshold_delta;
  printf("ransac inlier distance thresh = %.3lf\n", ransac_inlier_distance_threshold);
}

//----------------------------------------------------------------------------

void DecreaseInlierDistanceThreshCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if (ransac_inlier_distance_threshold > ransac_inlier_distance_threshold_delta) {
    ransac_inlier_distance_threshold -= ransac_inlier_distance_threshold_delta;
    printf("ransac inlier distance thresh = %.3lf\n", ransac_inlier_distance_threshold);
  }
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
  // than using constant max_cylinder_radius, right?  --provided true radius is <= max_cylinder_radius

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
		  ransac_inlier_distance_threshold);

  // use its endpoints and a max radius to crop input point cloud to cylindrical volume of interest (VOI)

  double min_xp, max_xp;
  //  compute_line_limits(cursor_cloudptr, line_coefficients, min_xp, max_xp);
  compute_line_limits(inlier_cloudptr, rough_line_coefficients, min_xp, max_xp);

  cylinder_slice(*input_cloudptr,
		 *rough_inlier_cloudptr,
		 *rough_outlier_cloudptr,
		 rough_line_coefficients,
		 rough_max_cylinder_radius,
		 min_xp, max_xp);

  // fit again to get finer estimate

  robust_line_fit(*rough_inlier_cloudptr,
		  *inlier_cloudptr,
		  *outlier_cloudptr,
		  line_coefficients,
		  max_cylinder_radius);

  compute_line_limits(inlier_cloudptr, line_coefficients, min_xp, max_xp);

  //    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  double mean_radius = mean_pointcloud_distance_to_3D_line(inlier_cloudptr, line_coefficients);

  printf("mean dist = %lf\n", mean_radius);

  /*
  robust_cylinder_fit(*inlier_cloudptr,
		      *rough_inlier_cloudptr,
		      *rough_outlier_cloudptr,
		      cylinder_coefficients,
		      0.01, max_cylinder_radius); // cylinder_inlier_distance_threshold);
  */


  send_cylinder_marker(line_coefficients,
		       mean_radius, // max_cylinder_radius,
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
		      cylinder_inlier_distance_threshold);

  inlier_cloudptr->header.frame_id = "base_link";
  inlier_cloud_pub.publish(*inlier_cloudptr);

  outlier_cloudptr->header.frame_id = "base_link";
  outlier_cloud_pub.publish(*outlier_cloudptr);
  */


  /*
  if (ud_cursor_pts.size() < 2)
    printf("need at least 2 points to parametrize a line\n");
  if (ud_cursor_pts.size() == 2)
  {
	  //directly find the parametric form of a line
	  
	  double dx, dy, dz;
	  
      dx = ud_cursor_pts[1].x - ud_cursor_pts[0].x;
      dy = ud_cursor_pts[1].y - ud_cursor_pts[0].y;
      dz = ud_cursor_pts[1].z - ud_cursor_pts[0].z;
      
      cout << "The line equation is: " << endl;
      cout << "x = " << ud_cursor_pts[0].x << " + " << dx << "t" << endl;
      cout << "y = " << ud_cursor_pts[0].y << " + " << dy << "t" << endl;
      cout << "z = " << ud_cursor_pts[0].z << " + " << dz << "t" << endl;
	  
  }
  if( ud_cursor_pts.size() > 2 )
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width  = ud_cursor_pts.size();
  cloud.height = 1;
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = ud_cursor_pts[i].x;
    cloud.points[i].y = ud_cursor_pts[i].y;
    cloud.points[i].z = ud_cursor_pts[i].z;
  }


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (ransac_inlier_distance_threshold);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }

  std::cerr << "Line params (x, y, z of point, direction vector) : " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << " " 
                                       << coefficients->values[4] << " " 
                                        << coefficients->values[5]
                                        << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
                                               << cloud.points[inliers->indices[i]].y << " "
                                               << cloud.points[inliers->indices[i]].z << std::endl;
  }
  */
}

// parametrize (if n = 2) or fit (n >= 3) LINE to all ud_cursor points


void EstimateLineCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  EstimateLine();
}

//----------------------------------------------------------------------------

// parametrize (if n = 3) or fit (n >= 4) PLANE to all ud_cursor points

void EstimatePlane()
{
  cursor_cloudptr->points.clear();

  for (int i = 0; i < ud_cursor_pts.size(); i++)
    cursor_cloudptr->points.push_back(ud_cursor_pts[i]);
  
  // this is just on the cursor points

  robust_plane_fit(*cursor_cloudptr,
		   *inlier_cloudptr,
		   *outlier_cloudptr,
		   plane_coefficients,
		   ransac_inlier_distance_threshold);


  pcl::ConvexHull<pcl::PointXYZ> hull;
  hull.setInputCloud (inlier_cloudptr);
  hull.reconstruct(*hull_cloudptr);
  if (hull.getDimension () == 2) {
    printf("hull points are 2-D\n");

    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    prism.setInputCloud (input_cloudptr);
    prism.setInputPlanarHull (hull_cloudptr);
    prism.setHeightLimits (-rough_max_plane_distance, rough_max_plane_distance);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

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

    printf("rough plane: %i in, %i out\n", rough_inlier_cloudptr->points.size(), rough_outlier_cloudptr->points.size());
 
    robust_plane_fit(*rough_inlier_cloudptr,
		     *inlier_cloudptr,
		     *outlier_cloudptr,
		     plane_coefficients,
		     max_plane_distance);

    printf("fine plane: %i in, %i out\n", inlier_cloudptr->points.size(), outlier_cloudptr->points.size());

  }
  else {
    printf("hull points are %i-D, should be 2-D\n", hull.getDimension());
    return;
  }

  /*
  plane_slice(*input_cloudptr,
	      *inlier_cloudptr,
	      *outlier_cloudptr,
	      plane_coefficients,
	      rough_max_plane_distance);
  */

  inlier_cloudptr->header.frame_id = "base_link";
  inlier_cloud_pub.publish(inlier_cloudptr);

  outlier_cloudptr->header.frame_id = "base_link";
  outlier_cloud_pub.publish(*outlier_cloudptr);

  /*

  if (ud_cursor_pts.size() < 3)
    printf("need at least 3 points to parametrize a plane\n");
  
  //RANSAC seems to work fine even on 3 points.  It just returns the a, b, c, and d values for the plane that is determined by them.
  else {
    
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width  = ud_cursor_pts.size();
  cloud.height = 1;
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = ud_cursor_pts[i].x;
    cloud.points[i].y = ud_cursor_pts[i].y;
    cloud.points[i].z = ud_cursor_pts[i].z;
  }
  
  std::cerr << "Point cloud data: " << cloud.points.size () << " points" << std::endl;
  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " 
                        << cloud.points[i].y << " " 
                        << cloud.points[i].z << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (ransac_inlier_distance_threshold);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
                                               << cloud.points[inliers->indices[i]].y << " "
                                               << cloud.points[inliers->indices[i]].z << std::endl;

 

}
  */

}

// parametrize (if n = 3) or fit (n >= 4) PLANE to all ud_cursor points

void EstimatePlaneCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  EstimatePlane();
}

//----------------------------------------------------------------------------

// parametrize (if n = 3) or fit (n >= 4) 3-D CIRCLE to all ud_cursor points

void EstimateCircle()
{
//TODO ADD CIRCLE CODE HERE

}

void EstimateCircleCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  EstimateCircle();
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

  if (ud_cursor_pt_do_move)
    menu_handler.setCheckState( h_move_entry, MenuHandler::CHECKED );
  else
    menu_handler.setCheckState( h_move_entry, MenuHandler::UNCHECKED );

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
  entry = menu_handler.insert( h_estimate_entry, "(+) inlier distance thresh", &IncreaseInlierDistanceThreshCb );
  entry = menu_handler.insert( h_estimate_entry, "(-) inlier distance thresh", &DecreaseInlierDistanceThreshCb );

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

void clickCallback(const geometry_msgs::PointStamped& msg)
{
  pcl::PointXYZ P;

  P.x = msg.point.x;
  P.y = msg.point.y;
  P.z = msg.point.z;

  if (ud_cursor_pt_do_move) {

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

  /*

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
  else if(workmode == 0)  //distance finding
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

    //    if(i>0)
    //      cpdist=cpdist+calc_cp_dist(i,polypoints);
    
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
  
  
  //  if (polycount!=num_polypoints)
  //    polycount++;
  //  else {
  //    polycount=0;
  //    cpdist=0;
  //  }
    
  } //end of workmode 0 code (distance finding)
    */

}

//----------------------------------------------------------------------------


void panelCallback(const ud_measurement_panel::MeasurementCommand& msg)
{
  // parameters

  // commands -- only one should be true

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
  else if (msg.EstimatePlane == 1) {
    EstimatePlane();
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

int main( int argc, char** argv )
{
  initialize_pcl();

  ud_cursor_pts.clear();

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

  printf("ud_imarker server starting\n"); fflush(stdout);

  ros::init(argc, argv, "UD_rviz_interaction");

  server.reset( new InteractiveMarkerServer("menu","",false) );
  initMenu();

  makeMenuMarker( "marker1" );
  menu_handler.apply( *server, "marker1" );
  server->applyChanges();

   nhp= new ros::NodeHandle();

  // Publishers

  marker_pub = nhp->advertise<visualization_msgs::Marker>("visualization_marker", 10);
  marker_array_pub = nhp->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

  //  rough_cylinder_marker_pub = nhp->advertise<visualization_msgs::Marker>("rough_cylinder_marker", 10);
  cylinder_marker_pub = nhp->advertise<visualization_msgs::Marker>("cylinder_marker", 10);

  ptcloud_pub = nhp->advertise<pcl::PointCloud<pcl::PointXYZ> > ("ud_output", 1);

  inlier_cloud_pub = nhp->advertise<pcl::PointCloud<pcl::PointXYZ> > ("ud_inliers", 1);
  outlier_cloud_pub = nhp->advertise<pcl::PointCloud<pcl::PointXYZ> > ("ud_outliers", 1);

  // Subscribers

  click_sub = nhp->subscribe("ud_clicked_point", 10, clickCallback);

  ptcloud_sub = nhp->subscribe("cloud_pcd", 10, ptcloudCallback);
  
  //add a subscriber to listen for things that are clicked on in the measurement panel
  measurement_sub = nhp->subscribe("measurement_command", 10, panelCallback);
  

  ros::Rate UD_rviz_interaction_rate(30);

  while (ros::ok())
  {
    UD_rviz_interaction_rate.sleep();
    ros::spinOnce();
  }
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
