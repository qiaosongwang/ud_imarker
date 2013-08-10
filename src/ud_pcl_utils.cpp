
//----------------------------------------------------------------------------

#include "ud_imarker.hh"

#define DEBUG_FIT_PLANE

//----------------------------------------------------------------------------

void plane_slice(pcl::PointCloud<pcl::PointXYZ> & cloud,
		      pcl::PointCloud<pcl::PointXYZ> & cloud_inliers,
		      pcl::PointCloud<pcl::PointXYZ> & cloud_outliers,
		      pcl::ModelCoefficients & coefficients,
		      double threshold)
{
  cloud_inliers.points.clear();
  cloud_outliers.points.clear();

  double dist;

  for (int i = 0; i < cloud.points.size(); i++) {

    dist = 
      coefficients.values[0] * cloud.points[i].x +
      coefficients.values[1] * cloud.points[i].y +
      coefficients.values[2] * cloud.points[i].z +
      coefficients.values[3];

    if (fabs(dist) < threshold)
      cloud_inliers.points.push_back(cloud.points[i]);
    else 
      cloud_outliers.points.push_back(cloud.points[i]);
  }

}

//----------------------------------------------------------------------------

// use RANSAC to find planar fit to point cloud

bool robust_plane_fit(pcl::PointCloud<pcl::PointXYZ> & cloud,
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

// SampleConsensusModelPerpendicularPlane<pcl::PointXYZ> model (cloud);
//  seg.setAxis (Eigen::Vector3f(axis_x, axis_y, axis_z));  //  (0.0, 0.0, 1.0));
//  seg.setEpsAngle (DEG2RAD(eps_angle_degs)); // 15.0));

  // Optional
  seg.setOptimizeCoefficients (true);

  // Mandatory
  //  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (threshold);
  
  seg.setMaxIterations(500);
  //  seg.setProbability(0.99);

  //  double val;
  // seg.getSamplesMaxDist(val);

#ifdef DEBUG_FIT_PLANE
  printf("max iters = %i, prob = %lf\n", seg.getMaxIterations(), seg.getProbability());
#endif

  seg.setInputCloud(cloud.makeShared());
  //  seg.segment (*inliers, *coefficients);
  seg.segment (*inliers, coefficients);
  
  if (inliers->indices.size () == 0) {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return false;
  }
  
#ifdef DEBUG_FIT_PLANE
  printf("Model coefficients: %lf %lf %lf %lf\n", 
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
    //  extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(cloud_inliers);
    
    extract.setNegative (true);
    extract.filter (cloud_outliers);
    
#ifdef DEBUG_FIT_PLANE
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

//   for (i = 0; i < cloud_inliers.size(); i++)
//     printf("%i: %.3lf %.3lf %.3lf\n", i, cloud_inliers[i].x, cloud_inliers[i].y, cloud_inliers[i].z);
// 
//   fflush(stdout);

//   pcl::toROSMsg(cloud_inliers, inlier_msg);
//   pcl::toROSMsg(cloud_outliers, outlier_msg);
// 
//   inlier_pub.publish(inlier_msg);
//   outlier_pub.publish(outlier_msg);

  return true;
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
