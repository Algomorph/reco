#ifndef PCL_UTILITIES_HPP
#define PCL_UTILITIES_HPP

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>

namespace pcl
{  
  /** \brief Scale pointcloud clouds relative to it's mean point
   * \param[in] cloud_in input pointcloud
   * \param[in] scale_factor scale factor
   * \param[in] cloud_out scaled pointcloud
   */
  template <typename T>
  inline
  void scalePointCloud( const typename pcl::PointCloud<T> &cloud_in,
                        float scale_factor,
                        typename pcl::PointCloud<T> &cloud_out)
  {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid<T>(cloud_in, centroid);
    
    cloud_out.resize(cloud_in.size());
    
    for (int i = 0; i < cloud_in.size(); i++)
    {
      cloud_out.points[i].x = (cloud_in.points[i].x - centroid[0]) * scale_factor + centroid[0];
      cloud_out.points[i].y = (cloud_in.points[i].y - centroid[1]) * scale_factor + centroid[1];
      cloud_out.points[i].z = (cloud_in.points[i].z - centroid[2]) * scale_factor + centroid[2];
    }
  }
}

#endif //PCL_UTILITIES_HPP