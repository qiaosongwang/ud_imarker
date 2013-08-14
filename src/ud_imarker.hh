#ifndef UD_IMARKER
#define UD_IMARKER

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes_rbf.h>

#include <Eigen/Core>

#include <boost/foreach.hpp>
#include <boost/bind.hpp>

#include <cmath>
#include <vector>
#include <math.h>

//----------------------------------------------------------------------------

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//----------------------------------------------------------------------------

using namespace std;
using namespace visualization_msgs;
using namespace interactive_markers;
using namespace pcl;

//----------------------------------------------------------------------------

void plane_slice(pcl::PointCloud<pcl::PointXYZ> &,
		 pcl::PointCloud<pcl::PointXYZ> &,
		 pcl::PointCloud<pcl::PointXYZ> &,
		 pcl::ModelCoefficients &,
		 double);

bool robust_plane_fit(pcl::PointCloud<pcl::PointXYZ> &,
		      pcl::PointCloud<pcl::PointXYZ> &,
		      pcl::PointCloud<pcl::PointXYZ> &,
		      pcl::ModelCoefficients &,
		      double);

void cylinder_slice(pcl::PointCloud<pcl::PointXYZ> &,
		    pcl::PointCloud<pcl::PointXYZ> &,
		    pcl::PointCloud<pcl::PointXYZ> &,
		    pcl::ModelCoefficients &,
		    double,
		    double, 
		    double);

bool robust_line_fit(pcl::PointCloud<pcl::PointXYZ> &,
		      pcl::PointCloud<pcl::PointXYZ> &,
		      pcl::PointCloud<pcl::PointXYZ> &,
		      pcl::ModelCoefficients &,
		      double);

bool robust_cylinder_fit(pcl::PointCloud<pcl::PointXYZ> &,
			 pcl::PointCloud<pcl::PointXYZ> &,
			 pcl::PointCloud<pcl::PointXYZ> &,
			 pcl::ModelCoefficients &,
			 double);
			 
bool robust_circle_fit(pcl::PointCloud<pcl::PointXYZ> &,
			 pcl::PointCloud<pcl::PointXYZ> &,
			 pcl::PointCloud<pcl::PointXYZ> &,
			 pcl::ModelCoefficients &,
			 double,
                         double);

void compute_line_limits(pcl::PointCloud<pcl::PointXYZ>::Ptr,
			 pcl::ModelCoefficients &,
			 double &, double &);
			 
void change_color (pcl::PointCloud<pcl::PointXYZRGB> &, 
                         int, 
                         int, 
                         int);
void segment_color(pcl::PointCloud<pcl::PointXYZRGB> &,
                         pcl::PointIndices::Ptr,
                         int,
                         int);

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

#endif
