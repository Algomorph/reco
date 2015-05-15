#ifndef POINT_LAB_H_
#define POINT_LAB_H_

namespace pcl
{
  /** \brief Members: float x, y, z, L, a, b
  * \ingroup common
  */
  struct PointXYZLAB;
  
  /** \brief Members: float x, y, z, L, a, b, normal[3]
  * \ingroup common
  */
  struct PointXYZLABNormal;   
}

#include <point_lab.hpp>  // Include struct definitions

// register the custom point type in PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::_PointXYZLAB,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, L, L)
    (float, a, a)
    (float, b, b)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZLAB, pcl::_PointXYZLAB)

// register the custom point type in PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::_PointXYZLABNormal,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, L, L)
    (float, a, a)
    (float, b, b)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZLABNormal, pcl::_PointXYZLABNormal)

#endif  //POINT_LAB_H_