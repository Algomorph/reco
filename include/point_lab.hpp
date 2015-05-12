#ifndef PCL_POINT_LAB_HPP_
#define PCL_POINT_LAB_HPP_

// We assume that whaterver code will be using this header will have these headers included already
// #include <Eigen/Core>
// #include <pcl/point_types.h>

namespace pcl
{
#define PCL_ADD_UNION_LAB \
  union \
  { \
    struct \
    { \
      float L; \
      float a; \
      float b; \
    }; \
    float data_lab[4]; \
  }; \

#define PCL_ADD_EIGEN_MAPS_LAB \
  inline Eigen::Vector3f getLABVector3f () { return (Eigen::Vector3f (L, a, b)); } \
  inline const Eigen::Vector3f getLABVector3f () const { return (Eigen::Vector3f (L, a, b)); } \
  
#define PCL_ADD_LAB \
  PCL_ADD_UNION_LAB \
  PCL_ADD_EIGEN_MAPS_LAB
  
  // PointXYZLAB
  struct EIGEN_ALIGN16 _PointXYZLAB
  {
    PCL_ADD_POINT4D;  // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_LAB;      // This adds the members L,a,b which can also be accessed using the point (which is float[4])
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZLAB& p);
  /** \brief A point structure representing Euclidean xyz coordinates and CIELAB colour data
    * \ingroup common
    */    
  struct PointXYZLAB : public _PointXYZLAB
  {
    inline PointXYZLAB (const _PointXYZLAB &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      L = p.L; a = p.a; b = p.b;
    }
    
    inline PointXYZLAB ()
    {
      x = y = z = 0.0f; data[3]     = 1.0f;  // important for homogeneous coordinates
      L = a = b = 0.0f; data_lab[3] = 0.0f;
    }
    
    friend std::ostream& operator << (std::ostream& os, const PointXYZLAB& p);
  };
  
  // PointXYZLABNormal
  struct EIGEN_ALIGN16 _PointXYZLABNormal
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])
    union
    {
      struct
      {
        PCL_ADD_UNION_LAB;
        float curvature;
      };
      float data_c[4];
    };
    PCL_ADD_EIGEN_MAPS_LAB;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };  
  
  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZLABNormal& p);
  /** \brief A point structure representing Euclidean xyz coordinates point normal and CIELAB colour data
    * \ingroup common
    */  
  struct PointXYZLABNormal : public _PointXYZLABNormal
  {
    inline PointXYZLABNormal (const _PointXYZLABNormal &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.0f;
      curvature = p.curvature;
      L = p.L; a = p.a; b = p.b;
    }

    inline PointXYZLABNormal ()
    { 
      x = y = z = 0.0f;
      data[3] = 1.0f;
      L = a = b = 0.0f;
      normal_x = normal_y = normal_z = data_n[3] = 0.0f;
      curvature = 0;
    }
    
    friend std::ostream& operator << (std::ostream& os, const PointXYZLABNormal& p);
  };
  
  
  std::ostream& operator << (std::ostream& os, const PointXYZLAB& p)
  {
    os << "(" << p.x << "," << p.y << "," << p.z << " - "
      << static_cast<int>(p.L) << ","
      << static_cast<int>(p.a) << ","
      << static_cast<int>(p.b) << ")";
    return (os);
  }
  
  std::ostream& operator << (std::ostream& os, const PointXYZLABNormal& p)
  {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.normal[0] << "," << p.normal[1] << "," << p.normal[2] << " - " << p.L << ", " << p.a << ", " << p.b << " - " << p.curvature << ")";
    return (os);
  }
  
}

#endif //#ifndef PCL_POINT_LAB_HPP_
