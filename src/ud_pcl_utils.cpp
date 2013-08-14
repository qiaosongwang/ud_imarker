//----------------------------------------------------------------------------

#include "ud_imarker.hh"

#define DEBUG_FIT

//----------------------------------------------------------------------------

// project each point in line_cloudptr onto line defined by line_coefficients
// and establish its 1-D position relative to "origin" given by line_coefficients[0-2].
// then return min and max 1-D positions along line

void compute_line_limits(pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloudptr, 
			 pcl::ModelCoefficients & line_coefficients,
			 double & min_xp, double & max_xp)
{
  Eigen::Vector3f L0(line_coefficients.values[0], line_coefficients.values[1], line_coefficients.values[2]);
  Eigen::Vector3f Ldir(line_coefficients.values[3], line_coefficients.values[4], line_coefficients.values[5]);
  Ldir.normalize();
  
  Eigen::Vector3f Pdir;
  double xp;
  min_xp = 1000.0;
  max_xp = -1000.0;

  for (int i = 0; i < line_cloudptr->points.size(); i++) {
    Eigen::Vector3f P(line_cloudptr->points[i].x, line_cloudptr->points[i].y, line_cloudptr->points[i].z);
    Pdir = P - L0;
    xp = Pdir.dot(Ldir);
    if (xp < min_xp)
      min_xp = xp;
    if (xp > max_xp)
      max_xp = xp;
    //    printf("%i: %.3lf\n", i, xp);
  }
  //  printf("[%.3lf, %.3lf]\n", min_xp, max_xp);
}

//----------------------------------------------------------------------------

// everything within distance *threshold* of LINE is an inlier, everything outside that is an outlier

// basically pcl::SampleConsensusModelLine<PointT>::selectWithinDistance () from sac_model_line.hpp

void cylinder_slice(pcl::PointCloud<pcl::PointXYZ> & cloud,
		    pcl::PointCloud<pcl::PointXYZ> & cloud_inliers,
		    pcl::PointCloud<pcl::PointXYZ> & cloud_outliers,
		    pcl::ModelCoefficients & line_coefficients,
		    double threshold,
		    double min_xp, double max_xp)
{
  cloud_inliers.points.clear();
  cloud_outliers.points.clear();

  double sqr_threshold = threshold * threshold;

  // Obtain the line point and direction
  Eigen::Vector4f line_pt  (line_coefficients.values[0], line_coefficients.values[1], line_coefficients.values[2], 0);
  Eigen::Vector4f line_dir (line_coefficients.values[3], line_coefficients.values[4], line_coefficients.values[5], 0);
  line_dir.normalize ();

  double dist;

  // for projection

  Eigen::Vector3f L0(line_coefficients.values[0], line_coefficients.values[1], line_coefficients.values[2]);
  Eigen::Vector3f Ldir(line_coefficients.values[3], line_coefficients.values[4], line_coefficients.values[5]);
  Ldir.normalize();

  Eigen::Vector3f Pdir;
  double xp;

  for (int i = 0; i < cloud.points.size(); i++) {

    // Calculate the distance from the point to the line
    // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)

    double sqr_distance = (line_pt - cloud.points[i].getVector4fMap ()).cross3 (line_dir).squaredNorm ();

    // close enough radially

    if (sqr_distance < sqr_threshold) {

      Eigen::Vector3f P(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
      Pdir = P - L0;
      xp = Pdir.dot(Ldir);
      if (xp >= min_xp && xp <= max_xp)
	cloud_inliers.points.push_back(cloud.points[i]);
    }
    else 
      cloud_outliers.points.push_back(cloud.points[i]);
    
  }
}

//----------------------------------------------------------------------------

// everything within distance *threshold* of PLANE is an inlier, everything outside that is an outlier

void plane_slice(pcl::PointCloud<pcl::PointXYZ> & cloud,
		      pcl::PointCloud<pcl::PointXYZ> & cloud_inliers,
		      pcl::PointCloud<pcl::PointXYZ> & cloud_outliers,
		      pcl::ModelCoefficients & plane_coefficients,
		      double threshold)
{
  cloud_inliers.points.clear();
  cloud_outliers.points.clear();

  double dist;

  for (int i = 0; i < cloud.points.size(); i++) {

    dist = 
      plane_coefficients.values[0] * cloud.points[i].x +
      plane_coefficients.values[1] * cloud.points[i].y +
      plane_coefficients.values[2] * cloud.points[i].z +
      plane_coefficients.values[3];

    if (fabs(dist) < threshold)
      cloud_inliers.points.push_back(cloud.points[i]);
    else 
      cloud_outliers.points.push_back(cloud.points[i]);
  }

}

//----------------------------------------------------------------------------

// use RANSAC to find PLANAR fit to point cloud

bool robust_plane_fit(pcl::PointCloud<pcl::PointXYZ> & cloud,
		      pcl::PointCloud<pcl::PointXYZ> & cloud_inliers,
		      pcl::PointCloud<pcl::PointXYZ> & cloud_outliers,
		      pcl::ModelCoefficients & coefficients,
		      double threshold)
{
  int i;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // Optional
  seg.setOptimizeCoefficients (true);

  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (threshold);
  
  seg.setMaxIterations(500);
  //  seg.setProbability(0.99);

  //  double val;
  // seg.getSamplesMaxDist(val);

#ifdef DEBUG_FIT
  printf("max iters = %i, prob = %lf\n", seg.getMaxIterations(), seg.getProbability());
#endif

  seg.setInputCloud(cloud.makeShared());
  //  seg.segment (*inliers, *coefficients);
  seg.segment (*inliers, coefficients);
  
  if (inliers->indices.size () == 0) {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return false;
  }
  
#ifdef DEBUG_FIT
  printf("PLANE coefficients: %.3lf %.3lf %.3lf %.3lf\n", 
	 coefficients.values[0],
	 coefficients.values[1],
	 coefficients.values[2],
	 coefficients.values[3]);
    
  printf("Model inliers: %i\n", inliers->indices.size());
#endif

  // unorganized

  if (cloud.height == 1) {

    // Create the filtering object

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    cloud_inliers.clear();
    cloud_outliers.clear();
    
    // extract inliers
    
    extract.setInputCloud(cloud.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(cloud_inliers);
    
    extract.setNegative (true);
    extract.filter (cloud_outliers);
    
#ifdef DEBUG_FIT
    printf("%i inliers, %i outliers\n", cloud_inliers.size(), cloud_outliers.size());
#endif
  }

  // organized

  else {

    cloud_inliers.width = cloud.width;
    cloud_inliers.height = cloud.height;
    cloud_inliers.points.resize(cloud.points.size());
    cloud_inliers.is_dense = false;

    cloud_outliers.width = cloud.width;
    cloud_outliers.height = cloud.height;
    cloud_outliers.points.resize(cloud.points.size());
    cloud_outliers.is_dense = false;

    pcl::PointXYZ badP;
    badP.x = numeric_limits<float>::quiet_NaN();
    badP.y = numeric_limits<float>::quiet_NaN();
    badP.z = numeric_limits<float>::quiet_NaN();

    //    printf("inliers size %i\n", inliers->indices.size());

    int i;

    for (i = 0; i < cloud.points.size(); i++) {
      cloud_inliers.points[i] = badP;
      cloud_outliers.points[i] = cloud.points[i];
    }

    for (i = 0; i < inliers->indices.size(); i++) {
      cloud_inliers.points[inliers->indices[i]] = cloud.points[inliers->indices[i]];
      cloud_outliers.points[inliers->indices[i]] = badP;
    }
  }

  return true;
}

//----------------------------------------------------------------------------

// use RANSAC to find LINEAR fit to point cloud

bool robust_line_fit(pcl::PointCloud<pcl::PointXYZ> & cloud,
		      pcl::PointCloud<pcl::PointXYZ> & cloud_inliers,
		      pcl::PointCloud<pcl::PointXYZ> & cloud_outliers,
		      pcl::ModelCoefficients & coefficients,
		      double threshold)
{
  int i;
  //  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // Optional
  seg.setOptimizeCoefficients (true);

  // Mandatory
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (threshold);
  
  seg.setMaxIterations(500);
  //  seg.setProbability(0.99);

  //  double val;
  // seg.getSamplesMaxDist(val);

#ifdef DEBUG_FIT
  printf("max iters = %i, prob = %lf\n", seg.getMaxIterations(), seg.getProbability());
#endif

  seg.setInputCloud(cloud.makeShared());
  //  seg.segment (*inliers, *coefficients);
  seg.segment (*inliers, coefficients);
  
  if (inliers->indices.size () == 0) {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return false;
  }
  
#ifdef DEBUG_FIT
  printf("LINE coefficients: %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf\n", 
	 coefficients.values[0],
	 coefficients.values[1],
	 coefficients.values[2],
	 coefficients.values[3],
	 coefficients.values[4],
	 coefficients.values[5]);
    
  printf("Model inliers: %i\n", inliers->indices.size());
#endif

  // unorganized

  if (cloud.height == 1) {

    // Create the filtering object

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    cloud_inliers.clear();
    cloud_outliers.clear();
    
    // extract inliers
    
    extract.setInputCloud(cloud.makeShared());
    //  extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(cloud_inliers);
    
    extract.setNegative (true);
    extract.filter (cloud_outliers);
    
#ifdef DEBUG_FIT
    printf("%i inliers, %i outliers\n", cloud_inliers.size(), cloud_outliers.size());
#endif
  }

  // organized

  else {

    cloud_inliers.width = cloud.width;
    cloud_inliers.height = cloud.height;
    cloud_inliers.points.resize(cloud.points.size());
    cloud_inliers.is_dense = false;

    cloud_outliers.width = cloud.width;
    cloud_outliers.height = cloud.height;
    cloud_outliers.points.resize(cloud.points.size());
    cloud_outliers.is_dense = false;

    pcl::PointXYZ badP;
    badP.x = numeric_limits<float>::quiet_NaN();
    badP.y = numeric_limits<float>::quiet_NaN();
    badP.z = numeric_limits<float>::quiet_NaN();

    //    printf("inliers size %i\n", inliers->indices.size());

    int i;

    for (i = 0; i < cloud.points.size(); i++) {
      cloud_inliers.points[i] = badP;
      cloud_outliers.points[i] = cloud.points[i];
    }

    for (i = 0; i < inliers->indices.size(); i++) {
      cloud_inliers.points[inliers->indices[i]] = cloud.points[inliers->indices[i]];
      cloud_outliers.points[inliers->indices[i]] = badP;
    }

  }

  return true;
}

//----------------------------------------------------------------------------

// use RANSAC to find CYLINDRICAL fit to point cloud

bool robust_cylinder_fit(pcl::PointCloud<pcl::PointXYZ> & cloud,
			 pcl::PointCloud<pcl::PointXYZ> & cloud_inliers,
			 pcl::PointCloud<pcl::PointXYZ> & cloud_outliers,
			 pcl::ModelCoefficients & coefficients,
			 double threshold)
{
  int i;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // Optional
  seg.setOptimizeCoefficients (true);

  // Mandatory
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (threshold);
  
  seg.setMaxIterations(500);
  //  seg.setProbability(0.99);

  //  double val;
  // seg.getSamplesMaxDist(val);

#ifdef DEBUG_FIT
  printf("max iters = %i, prob = %lf\n", seg.getMaxIterations(), seg.getProbability());
#endif

  seg.setInputCloud(cloud.makeShared());
  //  seg.segment (*inliers, *coefficients);
  seg.segment (*inliers, coefficients);
  
  if (inliers->indices.size () == 0) {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return false;
  }
  
#ifdef DEBUG_FIT
  printf("CYLINDER coefficients: %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf\n", 
	 coefficients.values[0],
	 coefficients.values[1],
	 coefficients.values[2],
	 coefficients.values[3],
	 coefficients.values[4],
	 coefficients.values[5],
	 coefficients.values[6]);
    
  printf("Model inliers: %i\n", inliers->indices.size());
#endif

  // unorganized

  if (cloud.height == 1) {

    // Create the filtering object

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    cloud_inliers.clear();
    cloud_outliers.clear();
    
    // extract inliers
    
    extract.setInputCloud(cloud.makeShared());
    //  extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(cloud_inliers);
    
    extract.setNegative (true);
    extract.filter (cloud_outliers);
    
#ifdef DEBUG_FIT
    printf("%i inliers, %i outliers\n", cloud_inliers.size(), cloud_outliers.size());
#endif
  }

  // organized

  else {

    cloud_inliers.width = cloud.width;
    cloud_inliers.height = cloud.height;
    cloud_inliers.points.resize(cloud.points.size());
    cloud_inliers.is_dense = false;

    cloud_outliers.width = cloud.width;
    cloud_outliers.height = cloud.height;
    cloud_outliers.points.resize(cloud.points.size());
    cloud_outliers.is_dense = false;

    pcl::PointXYZ badP;
    badP.x = numeric_limits<float>::quiet_NaN();
    badP.y = numeric_limits<float>::quiet_NaN();
    badP.z = numeric_limits<float>::quiet_NaN();

    //    printf("inliers size %i\n", inliers->indices.size());

    int i;

    for (i = 0; i < cloud.points.size(); i++) {
      cloud_inliers.points[i] = badP;
      cloud_outliers.points[i] = cloud.points[i];
    }

    for (i = 0; i < inliers->indices.size(); i++) {
      cloud_inliers.points[inliers->indices[i]] = cloud.points[inliers->indices[i]];
      cloud_outliers.points[inliers->indices[i]] = badP;
    }

  }

  return true;
}

//----------------------------------------------------------------------------

// use RANSAC to find CIRCLE  fit to point cloud

bool robust_circle_fit(pcl::PointCloud<pcl::PointXYZ> & cloud,
			 pcl::PointCloud<pcl::PointXYZ> & cloud_inliers,
			 pcl::PointCloud<pcl::PointXYZ> & cloud_outliers,
			 pcl::ModelCoefficients & coefficients,
			 double radius_min,
                         double radius_max)
{  

// Declaration
    pcl::NormalEstimation<PointXYZ, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointXYZ, pcl::Normal> seg;
    pcl::ExtractIndices<PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
    new pcl::PointCloud<pcl::Normal>);

 //   pcl::ModelCoefficients::Ptr circle_coeff(
 //   new pcl::ModelCoefficients);

    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices),
    circle_inlier(new pcl::PointIndices);

// Estimate normal
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud.makeShared());
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

// Set segmentation object model to be a circle
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE2D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.1);

// For testing
//  radius_min =0.02;
//  radius_max =0.08;
    seg.setRadiusLimits(radius_min, radius_max);
    seg.setInputCloud(cloud.makeShared());
    seg.setInputNormals(cloud_normals);

// Get circle coefficients and inliers
    seg.segment(*circle_inlier, coefficients);


    extract.setInputCloud(cloud.makeShared());
    extract.setIndices(circle_inlier);
    extract.setNegative(false);
    extract.filter(cloud_inliers);

    if (cloud_inliers.points.empty()) 
    {

#ifdef DEBUG_FIT
    printf( "Can't find circle");
#endif
    return false;
		}
#ifdef DEBUG_FIT
    printf( "Circle found!");
#endif



//Uncomment for simplified code
/*
    pcl::PointIndices circle_inlier;
    pcl::ModelCoefficients circle_coeff;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<PointXYZ> extract;


    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CIRCLE2D);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0.05, 0.1);
    seg.setInputCloud (cloud.makeShared());
    // Obtain the circle inliers and coefficients
    seg.segment (circle_inlier, circle_coeff);
    
#ifdef DEBUG_FIT
    std::cerr << "Circle inliers " << circle_inlier.indices.size() << std::endl;
    std::cerr << "Circle coefficients: " << circle_coeff << std::endl;
#endif

    extract.setInputCloud(cloud.makeShared());
    extract.setIndices(boost::make_shared<const pcl::PointIndices> (circle_inlier));

    pcl::PointCloud<pcl::PointXYZ> outputcloud;
    extract.filter (outputcloud);
*/

    return true;
}

//----------------------------------------------------------------------------
// Color based segmentation
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Used for highlighting a pointcloud
void change_color (pcl::PointCloud<pcl::PointXYZRGB> & cloud, 
                      int r, 
                      int g, 
                      int b)
{
    for (size_t i = 0; i < cloud.size(); ++i){
        (cloud)[i].r=r;
        (cloud)[i].g=g;
        (cloud)[i].b=b;
    }
}

//----------------------------------------------------------------------------

// The purpose is to let the user select one point or a group of points to mark 
// foreground/background, then use color based methods to get segmented pointcloud 
// with similar color, then do circle/plane/cylinder fitting. 


void segment_color(pcl::PointCloud<pcl::PointXYZRGB> & cloud,
                         pcl::PointIndices::Ptr segmented,
                         int reference_point_index,
                         int threshold)
{

    pcl::PointXYZRGB ud_cursor_current = (cloud)[reference_point_index];
    int r = ud_cursor_current.r;
    int g = ud_cursor_current.g;
    int b = ud_cursor_current.b;
    int i = 0;
    for (i = 0; i <cloud.size()-1; i++)
    {
        if
        (
             abs(cloud.at(i).r-r)<threshold &&
             abs(cloud.at(i).g-g)<threshold &&
             abs(cloud.at(i).b-b)<threshold
        )
        segmented->indices.push_back(i);
    }
}


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
