#ifndef PCL_LAB_RGB_CONVERSIONS_HPP_
#define PCL_LAB_RGB_CONVERSIONS_HPP_

#include <pcl/console/print.h>

namespace pcl
{
  /** \brief Convert an Eigen3i vector with RGB values to an Eigen3f vector with Lab values
    * \param[in] color_RGB RGB vector
    * \return Lab vector
    * \note OpenCV conversion algorithim is used CV_RGB2LAB (http://docs.opencv.org/modules/imgproc/doc/miscellaneous_transformations.html?highlight=cvtcolor#cvtcolor) Scaling to [0, 255] range is not performed
    */  
  inline
  Eigen::Vector3f RGB2Lab (const Eigen::Vector3i& color_RGB)
  {
    float R, G, B, X, Y, Z, L, a, b;
    
    // Cast RGB to float and scale
    R = static_cast<float>(color_RGB[0]) / 255;
    G = static_cast<float>(color_RGB[1]) / 255;
    B = static_cast<float>(color_RGB[2]) / 255;

    // Convert to XYZ
    X = (0.412453 * R + 0.357580 * G + 0.180423 * B) / 0.950456;
    Y = (0.212671 * R + 0.715160 * G + 0.072169 * B);
    Z = (0.019334 * R + 0.119193 * G + 0.950227 * B) / 1.088754;
    
    // Convert to Lab
    float T = 0.008856;
    float fX, fY, fZ;

    if (X > T)
      fX = pow(X, 1./3.);
    else
      fX = 7.787 * X + 16./116.;

    if (Y > T)
    {
      fY = pow(Y, 1./3.);
      L  = 116. * fY - 16.0;
    }
    else
    {
      fY = 7.787 * Y + 16./116.;
      L  = 903.3 * Y;
    }

    if (Z > T)
      fZ = pow(Z, 1./3.);
    else
      fZ = 7.787 * Z + 16./116.;

    a = 500. * (fX - fY);
    b = 200. * (fY - fZ);   

    return Eigen::Vector3f (L, a, b);
  }

  /** \brief Convert an Eigen3f vector with Lab values to an Eigen3i vector with RGB values
    * \param[in] color_Lab Lab vector
    * \return RGB vector
    */  
  inline
  Eigen::Vector3i Lab2RGB (const Eigen::Vector3f& color_Lab)
  {
    console::print_error("[pcl::Lab2RGB not implemented yet]");
    exit(EXIT_FAILURE);
    
    Eigen::Vector3i color_RGB;
    
    return color_RGB;
  }
  
  /** \brief Convert a XYZRGB point to XYZLab point
    * \param[in] cloud_in XYZRGB point
    * \param[out] cloud_out XYZLAB point
    */
  inline void
  PointXYZRGBtoXYZLAB(const PointXYZRGB &in, PointXYZLAB &out)
  {
    out.x = in.x; out.y = in.y; out.z = in.z;
    Eigen::Vector3f Lab = RGB2Lab (in.getRGBVector3i ());
    out.L = Lab[0];
    out.a = Lab[1];
    out.b = Lab[2];
  }

  /** \brief Convert a XYZRGB point to XYZLab point
    * \param[in] cloud_in XYZRGB point
    * \param[out] cloud_out XYZLAB point
    */
  inline void
  PointXYZRGBNormaltoXYZLABNormal(const PointXYZRGBNormal &in, PointXYZLABNormal &out)
  {
    out.x = in.x; out.y = in.y; out.z = in.z;
    out.normal_x = in.normal_x; out.normal_y = in.normal_y; out.normal_z = in.normal_z;
    out.curvature = in.curvature;
    Eigen::Vector3f Lab = RGB2Lab (in.getRGBVector3i ());
    out.L = Lab[0];
    out.a = Lab[1];
    out.b = Lab[2];
  }  

  /** \brief Convert a PointXYZRGB pointcloud to PointXYZLAB pointcloud
    * \param[in] cloud_in PointXYZRGB cloud
    * \param[out] cloud_out PointXYZLAB cloud
    */
  inline void
  PointCloudXYZRGBtoXYZLAB(const PointCloud<PointXYZRGB> &in, PointCloud<PointXYZLAB> &out)
  {
    // Check that input clouds have appropriate fields
    if (getFieldsList(in).find("rgb") == std::string::npos)
    {
      console::print_error("[pcl::PointCloudXYZRGB2XYZLAB] input cloud must contain RGB fields\n");
      exit(EXIT_FAILURE);
    }
    else if (getFieldsList(out).find("L a b") == std::string::npos)
    {
      console::print_error("[pcl::PointCloudXYZRGB2XYZLAB] output cloud must contain Lab fields\n");
      exit(EXIT_FAILURE);
    }
    
    // Prepare output cloud
    out.resize(in.size());
    out.is_dense             = in.is_dense;
    out.sensor_orientation_  = in.sensor_orientation_;
    out.sensor_origin_       = in.sensor_origin_;
    out.header               = in.header;  
    out.width                = in.width;
    out.height               = in.height;
    
    for (size_t i = 0; i < in.size(); i++)
    {
      PointXYZRGBtoXYZLAB(in.points[i], out.points[i]);
    }
  }
  
  /** \brief Convert a PointXYZRGB pointcloud to PointXYZLAB pointcloud
    * \param[in] cloud_in PointXYZRGB cloud
    * \param[out] cloud_out PointXYZLAB cloud
    */
  inline void
  PointCloudXYZRGBNormaltoXYZLABNormal(const PointCloud<PointXYZRGBNormal> &in, PointCloud<PointXYZLABNormal> &out)
  {
    // Check that input clouds have appropriate fields
    if (getFieldsList(in).find("rgb") == std::string::npos)
    {
      console::print_error("[pcl::PointCloudXYZRGBNormaltoXYZLABNormal] input cloud must contain RGB fields\n");
      exit(EXIT_FAILURE);
    }
    else if (getFieldsList(out).find("L a b") == std::string::npos)
    {
      console::print_error("[pcl::PointCloudXYZRGBNormaltoXYZLABNormal] output cloud must contain Lab fields\n");
      exit(EXIT_FAILURE);
    }
    else if (getFieldsList(in).find("normal_x normal_y normal_z") == std::string::npos)
    {
      console::print_error("[pcl::PointCloudXYZRGBNormaltoXYZLABNormal] input cloud must contain normal data\n");
      exit(EXIT_FAILURE);
    }
    else if (getFieldsList(out).find("normal_x normal_y normal_z") == std::string::npos)
    {
      console::print_error("[pcl::PointCloudXYZRGBNormaltoXYZLABNormal] output cloud must contain normal data\n");
      exit(EXIT_FAILURE);
    }
    
    // Prepare output cloud
    out.resize(in.size());
    out.is_dense             = in.is_dense;
    out.sensor_orientation_  = in.sensor_orientation_;
    out.sensor_origin_       = in.sensor_origin_;
    out.header               = in.header;  
    out.width                = in.width;
    out.height               = in.height;
    
    for (size_t i = 0; i < in.size(); i++)
    {
      PointXYZRGBNormaltoXYZLABNormal(in.points[i], out.points[i]);
    }
  }  
  
//   /** \brief Convert a pointcloud containing RGB colour information to a pointcloud containing LAB colour information
//     * \param[in] cloud_in RGB cloud
//     * \param[out] cloud_out LAB cloud
//     */
//   template <typename T1, typename T2>
//   void PointCloudRGB2LAB(const PointCloud<T1> &cloud_rgb, PointCloud<T2> &cloud_lab)
//   {
//     // Check that input clouds have appropriate fields
//     if (getFieldsList(cloud_rgb).find("rgb") == std::string::npos)
//     {
//       console::print_error("[pcl::PointCloudRGB2LAB] input cloud must contain RGB fields\n");
//       exit(EXIT_FAILURE);
//     }
//     else if (getFieldsList(cloud_lab).find("L a b") == std::string::npos)
//     {
//       console::print_error("[pcl::PointCloudRGB2LAB] output cloud must contain Lab fields\n");
//       exit(EXIT_FAILURE);
//     }
//     
//     // Prepare output cloud
//     cloud_lab.resize(cloud_rgb.size());
//     cloud_lab.is_dense             = cloud_rgb.is_dense;
//     cloud_lab.sensor_orientation_  = cloud_rgb.sensor_orientation_;
//     cloud_lab.sensor_origin_       = cloud_rgb.sensor_origin_;
//     cloud_lab.header               = cloud_rgb.header;  
//     cloud_lab.width                = cloud_rgb.width;
//     cloud_lab.height               = cloud_rgb.height;
//     
//     // Convert colour
//     for (size_t i = 0; i < cloud_rgb.size(); i++)
//     {
//       cloud_lab[i].x = cloud_rgb[i].x;
//       cloud_lab[i].y = cloud_rgb[i].y;
//       cloud_lab[i].z = cloud_rgb[i].z;
//       cloud_lab[i].data[3] = 1.0;  // important for homogeneous coordinates
// 
//       Eigen::Vector3f lab = RGB2Lab (cloud_rgb[i].getRGBVector3i ());
//       cloud_lab[i].L = lab[0];
//       cloud_lab[i].a = lab[1];
//       cloud_lab[i].b = lab[2];      
//     }
//   }
//   
//   /** \brief Convert a pointcloud containing LAB colour information to a pointcloud containing RGB colour information
//     * \param[in] cloud_in LAB cloud
//     * \param[out] cloud_out RGB cloud
//     * \note Input and output can not be the same
//     */
//   template <typename T1, typename T2>
//   void PointCloudLAB2RGB(const PointCloud<T1> &cloud_lab, PointCloud<T2> &cloud_rgb)
//   {
//     // Check that input clouds have appropriate fields
//     if (getFieldsList(cloud_rgb).find("rgb") == std::string::npos)
//     {
//       console::print_error("[pcl::PointCloudRGB2LAB] input cloud must contain RGB fields\n");
//       exit(EXIT_FAILURE);
//     }
//     else if (getFieldsList(cloud_lab).find("L a b") == std::string::npos)
//     {
//       console::print_error("[pcl::PointCloudRGB2LAB] output cloud must contain Lab fields\n");
//       exit(EXIT_FAILURE);
//     }
//     
//     // Prepare output cloud
//     cloud_rgb.resize(cloud_lab.size());
//     cloud_rgb.is_dense             = cloud_lab.is_dense;
//     cloud_rgb.sensor_orientation_  = cloud_lab.sensor_orientation_;
//     cloud_rgb.sensor_origin_       = cloud_lab.sensor_origin_;
//     cloud_rgb.header               = cloud_lab.header;  
//     cloud_rgb.width                = cloud_lab.width;
//     cloud_rgb.height               = cloud_lab.height;
//     
//     // Convert colour    
//     for (size_t i = 0; i < cloud_lab.size(); i++)
//     {
//       cloud_rgb[i].x = cloud_lab[i].x;
//       cloud_rgb[i].y = cloud_lab[i].y;
//       cloud_rgb[i].z = cloud_lab[i].z;
//       cloud_rgb[i].data[3] = 1.0;  // important for homogeneous coordinates
// 
//       Eigen::Vector3i rgb = Lab2RGB (cloud_lab[i].getLABVector3f ());
//       cloud_rgb[i].r = rgb[0];
//       cloud_rgb[i].g = rgb[1];
//       cloud_rgb[i].b = rgb[2];      
//     }
//   }
}

#endif //#ifndef PCL_LAB_RGB_CONVERSIONS_HPP_
