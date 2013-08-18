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

// shortest rotation between two 3-D vectors (e.g., normals)

tf::Quaternion shortest_rotation_quaternion(double a1, double b1, double c1,
					    double a2, double b2, double c2)
{
  Eigen::Vector3d v1(a1, b1, c1);
  Eigen::Vector3d v2(a2, b2, c2);
  Eigen::Vector3d cr = v1.cross(v2);

  tf::Quaternion q(cr(0), cr(1), cr(2), 1 + v1.dot(v2));
  q.normalize();

  return q;
}

//----------------------------------------------------------------------------

// rotate AND translate to correct height given plane equation

// http://www.gamedev.net/topic/429507-finding-the-quaternion-betwee-two-vectors/#entry3856228

void transform_to_level(pcl::PointCloud<pcl::PointXYZ> & cloud_in, pcl::PointCloud<pcl::PointXYZ> & cloud_out, double a, double b, double c, double d)
{
  tf::Transform xform(shortest_rotation_quaternion(a, b, c, 0, 0, 1),
		      tf::Vector3(0, 0, d));

  pcl_ros::transformPointCloud(cloud_in, cloud_out, xform);
}

//----------------------------------------------------------------------------

void transform_to_level(pcl::PointCloud<pcl::PointXYZ> & cloud_in, pcl::PointCloud<pcl::PointXYZ> & cloud_out, pcl::ModelCoefficients & plane_coefficients)
{
  transform_to_level(cloud_in, cloud_out, 
		     plane_coefficients.values[0], plane_coefficients.values[1], plane_coefficients.values[2], plane_coefficients.values[3]);
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

//SIFT keypoint detection

void sift_detection ( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints)
{

    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>* sift3D = new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    sift3D->setScales(0.1, 3, 2);
    sift3D->setMinimumContrast(0.0);
    sift3D->setSearchMethod(tree);
    sift3D->setInputCloud(input);
    sift3D->compute(*keypoints);
}

//----------------------------------------------------------------------------

//FPFH feature detection

void fpfh_detection ( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
                          pcl::PointCloud<pcl::FPFHSignature33>::Ptr features)
{
    pcl::Feature<pcl::PointXYZRGB, pcl::FPFHSignature33>::Ptr feature_extractor_ (new pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>);
    feature_extractor_->setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    feature_extractor_->setRadiusSearch (50);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts(new pcl::PointCloud<pcl::PointXYZRGB>);
    kpts->points.resize(keypoints->points.size());

    pcl::copyPointCloud(*keypoints, *kpts);

    pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> > (feature_extractor_);
    feature_extractor_->setInputCloud(kpts);
   //Normals estimation and Descriptors extraction
    pcl::PointCloud<pcl::Normal>::Ptr normals (new  pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
    normal_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    normal_estimation.setRadiusSearch (50);
    normal_estimation.setInputCloud (kpts);
    normal_estimation.compute (*normals);
    feature_from_normals->setInputNormals(normals);
    feature_extractor_->compute (*features);

}

//----------------------------------------------------------------------------

//Find feature correspondence

void find_correspondences (
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr source,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr target,
        vector<int>& correspondences)
{

    correspondences.resize (source->size());

 
    pcl::KdTreeFLANN<pcl::FPFHSignature33> descriptor_kdtree;
    descriptor_kdtree.setInputCloud (target);

    const int k = 1;
    vector<int> k_indices (k);
    vector<float> k_squared_distances (k);
    for (size_t i = 0; i < source->size (); ++i)
    {
        descriptor_kdtree.nearestKSearch (*source, i, k, k_indices, k_squared_distances);
        correspondences[i] = k_indices[0];
    }
}

//----------------------------------------------------------------------------

void filter_correspondences ( pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints_,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_ ,
                             vector<int>& source2target_ , vector<int>& target2source_ ,
                             pcl::CorrespondencesPtr correspondences_)
{
    vector<pair<unsigned, unsigned> > correspondences;
    for (unsigned cIdx = 0; cIdx < source2target_.size (); ++cIdx)
        if (target2source_[source2target_[cIdx]] == cIdx)
            correspondences.push_back(make_pair(cIdx, source2target_[cIdx]));

    correspondences_->resize (correspondences.size());
    for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx)
    {
        (*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
        (*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
    }

    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector;
    rejector.setInputCloud(source_keypoints_);
    rejector.setTargetCloud(target_keypoints_);
    rejector.setMaxIterations(20);
    rejector.setInlierThreshold(2);
    rejector.setInputCorrespondences(correspondences_);
    rejector.getCorrespondences(*correspondences_);
}

//----------------------------------------------------------------------------

//Keypoint based initial transform

Eigen::Matrix4f initial_transform (
        pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints_ ,
        pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_ ,
        pcl::CorrespondencesPtr correspondences_
        )
{
    Eigen::Matrix4f initial_transformation_matrix_ = Eigen::Matrix4f::Identity();
    pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Ptr transformation_estimation (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI>);
    transformation_estimation->estimateRigidTransformation (*source_keypoints_, *target_keypoints_, *correspondences_, initial_transformation_matrix_);
    return initial_transformation_matrix_;
}

//----------------------------------------------------------------------------

//ICP final refinement

Eigen::Matrix4f final_transform (
        pcl::PointCloud<pcl::PointXYZI>::Ptr source_transformed_ ,
        pcl::PointCloud<pcl::PointXYZI>::Ptr target_segmented_
        )
{
  
    Eigen::Matrix4f final_transformation_matrix_=Eigen::Matrix4f::Identity ();
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration (new pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>);
    registration->setInputCloud(source_transformed_);
    registration->setInputTarget (target_segmented_);
    registration->setMaxCorrespondenceDistance(0.01); //4
    registration->setRANSACOutlierRejectionThreshold (1000); //4
    registration->setTransformationEpsilon (0.0001); //0.0001
    registration->setRANSACIterations(100);
    //registration->setMaximumIterations (10000); //1
    registration->align(*source_transformed_);
    final_transformation_matrix_ = registration->getFinalTransformation();
    return final_transformation_matrix_;
}

//----------------------------------------------------------------------------

//3D SIFT keypoint based registration, use SIFT keypoints to obtain initial transformation matrix,
//Then use ICP for precise alignment, can be used for fast registration (much faster than ICP
//or recognition using pre-defined CAT models as source pointcloud

//Sample usage:
//sift_registration(cloud_source,cloud_target,cloud_registred)
//All inputs should be XYZRGB pointclouds

//Known issue:
//If there are too many sift keypoints in the source and destination pointclouds, 
//the program can not find correspondence, the error is in line 99 of 
//pcl/registration/impl/correspondence_rejection_sample_consensus.hpp

bool sift_registration (
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr source ,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr registered)
{

    //sift3d keypoint detector
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints (new pcl::PointCloud<pcl::PointXYZI> ());
    sift_detection (source, source_keypoints);
    sift_detection (target, target_keypoints);

    // For testing
    // pcl::io::savePCDFileBinary("./target_keypoints.pcd",*target_keypoints);
    

    //FPFH feature extractor
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features (new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features (new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh_detection (source, source_keypoints, source_features);
    fpfh_detection (target, target_keypoints, target_features);
    // For testing
    //pcl::io::savePCDFileBinary("./target_features.pcd",*target_features);
  

    //Find Correspondences
    vector<int> source2target ;
    vector<int> target2source ;
    find_correspondences (source_features, target_features, source2target);
    find_correspondences (target_features, source_features, target2source);

    //Filter Correnspondences
      pcl::CorrespondencesPtr correspondences (new pcl::Correspondences);
      filter_correspondences (source_keypoints, target_keypoints , source2target , target2source, correspondences);

    //Initial transformation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Matrix4f initial_transformation_matrix = initial_transform (source_keypoints, target_keypoints, correspondences );
   //Initial alignment
    pcl::transformPointCloud(*source, *source_transformed, initial_transformation_matrix);

   //Final ICP alignment
    //Eigen::Matrix4f final_transformation_matrix = final_transform (source_transformed , registered, target);
    pcl::copyPointCloud(*source_transformed, *registered);  
    
	 //For testing
	 //pcl::io::savePCDFileBinary("./source.pcd",*source);
	 //pcl::io::savePCDFileBinary("./target.pcd",*target);
	 //pcl::io::savePCDFileBinary("./registered.pcd",*registered);
   return true;
}


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
