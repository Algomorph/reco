#ifndef PCL_OCV_CONVERSIONS_HPP_
#define PCL_OCV_CONVERSIONS_HPP_

// We assume that the code that uses this header already has these included
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/print.h>
#include <pcl/io/io.h>
#include <opencv2/core/core.hpp>

namespace pcl
{
  /** \brief Extract an RGB image from a PCL pointcloud containing RGB data
    * \param[in] cloudn the input point cloud
    * \param[out] image_rgb RGB image (CV_8UC3)
    * \note returned image is BGR not RGB
    */
  template <typename T>
  void pclCloud2cvImageRGB(const pcl::PointCloud<T>& cloud, cv::Mat& image_rgb)
  {
    // Check input
    if (cloud.empty())
    {
      pcl::console::print_error("[pclCloud2cvImageRGB] input pointcloud is empty.\n");
      exit(EXIT_FAILURE);
    }
    else if (!cloud.isOrganized())
    {
      pcl::console::print_error("[pclCloud2cvImageRGB] input pointcloud must be organized.\n");
      exit(EXIT_FAILURE);
    }
    else if (pcl::getFieldsList(cloud).find("rgb") == std::string::npos)
    {
      pcl::console::print_error("[pclCloud2cvImageRGB] Input cloud must contain RGB data");
      exit(EXIT_FAILURE);
    }
    
    // Process
    int xRes = cloud.width;
    int yRes = cloud.height;
    
    image_rgb = cv::Mat(yRes, xRes, CV_8UC3);      // Create an OpenCV matrix of appropriate size and reserve memory for it
    for (size_t y=0; y<yRes; y++)
      for (size_t x=0; x<xRes; x++)
      {
        T point = cloud.at(x, y);
        image_rgb.at<cv::Vec3b>(y,x)[0] = point.b;
        image_rgb.at<cv::Vec3b>(y,x)[1] = point.g;
        image_rgb.at<cv::Vec3b>(y,x)[2] = point.r;
      }
  }

  /** \brief Extract an Lab image from a PCL pointcloud containing Lab data. Lab data is scaled to [0, 255] range and cast to uchar
    * \param[in] cloudn the input point cloud
    * \param[out] image_lab Lab image (CV_8UC3)
    */
  template <typename T>
  void pclCloud2cvImageLab(const pcl::PointCloud<T>& cloud, cv::Mat& image_lab)
  {
    // Check input
    if (cloud.empty())
    {
      pcl::console::print_error("[pclCloud2cvImageRGB] input pointcloud is empty.\n");
      exit(EXIT_FAILURE);
    }
    else if (!cloud.isOrganized())
    {
      pcl::console::print_error("[pclCloud2cvImageRGB] input pointcloud must be organized.\n");
      exit(EXIT_FAILURE);
    }
    else if (pcl::getFieldsList(cloud).find("L a b") == std::string::npos)
    {
      pcl::console::print_error("[pclCloud2cvImageRGB] input cloud must contain Lab data.\n");
      exit(EXIT_FAILURE);
    }
    
    // Process
    int xRes = cloud.width;
    int yRes = cloud.height;
    
    image_lab = cv::Mat(yRes, xRes, CV_8UC3);      // Create an OpenCV matrix of appropriate size and reserve memory for it
    for (size_t y=0; y<yRes; y++)
      for (size_t x=0; x<xRes; x++)
      {
        T point = cloud.at(x, y);
        image_lab.at<cv::Vec3b>(y,x)[0] = static_cast<uchar> (point.L/100*255);
        image_lab.at<cv::Vec3b>(y,x)[1] = static_cast<uchar> (point.a + 128);
        image_lab.at<cv::Vec3b>(y,x)[2] = static_cast<uchar> (point.b + 128);
      }
  }

  /** \brief Extract depth image from a PCL pointcloud
   * \param[in] cloud the input point cloud
   * \param[out] image_depth depth image (CV_32FC1)
   * \note Input cloud must be organized
   */
  template <typename T>
  void pclCloud2cvDepth(const pcl::PointCloud<T>& cloud, cv::Mat& image_depth)
  {
    // Check input
    if (cloud.empty())
    {
      pcl::console::print_error("[pclCloud2cvDepth] input pointcloud is empty.\n");
      exit(EXIT_FAILURE);
    }
    else if (!cloud.isOrganized())
    {
      pcl::console::print_error("[pclCloud2cvDepth] input pointcloud must be organized.\n");
      exit(EXIT_FAILURE);
    }
    else if (pcl::getFieldsList(cloud).find("x y z") == std::string::npos)
    {
      pcl::console::print_error("[pclCloud2cvDepth] input cloud must contain XYZ data");
      exit(EXIT_FAILURE);
    }
    
    // Process
    int xRes = cloud.width;
    int yRes = cloud.height;
    
    image_depth = cv::Mat(yRes, xRes, CV_32FC1);      // Create an OpenCV matrix of appropriate size and reserve memory for it
    for (size_t x=0; x<xRes; x++)
      for (size_t y=0; y<yRes; y++)  
      {
        T point = cloud.at(x, y);
        image_depth.at<float>(y,x) = point.z;
      }
  }

  /** \brief Extract normal data from a PCL pointcloud and store it an OpenCV matrix of floats
    * \param[in] cloud the input point cloud
    * \param[out] normals_cv OpenCV matrix
    * \note Input cloud must be organized
    */
  template <typename T>
  void pclNormals2cvNormals(const pcl::PointCloud<T>& cloud, cv::Mat& normals_cv)
  {
    // Check input
    if (pcl::getFieldsList(cloud).find("normal_x normal_y normal_z") == std::string::npos)
    {
      pcl::console::print_error("[pclNormals2cvNormals] input cloud must contain normal data");
      exit(EXIT_FAILURE);
    }
    else if (!cloud.isOrganized())
    {
      pcl::console::print_error("[pclNormals2cvNormals] input cloud must be organized");
      exit(EXIT_FAILURE);
    }    
    
    // Process
    normals_cv = cv::Mat::zeros(cloud.height, cloud.width, CV_32FC3);
    for(int y = 0; y < cloud.height; y++)
      for(int x = 0; x < cloud.width; x++)
      {
        normals_cv.at<cv::Vec3f>(y,x)[0] = cloud.at(x,y).normal_x;
        normals_cv.at<cv::Vec3f>(y,x)[1] = cloud.at(x,y).normal_y;
        normals_cv.at<cv::Vec3f>(y,x)[2] = cloud.at(x,y).normal_z;
      }
  }  
  
  /** \brief Convert an OpenCV depth image to a pointcloud
    * \param[in] depth OpenCV depth image (CV_16U where depth is expressed in milimetres)
    * \param[in] K depth camera calibration matrix
    * \param[out] cloud PCL pointcloud
    * \note For now the calibration matrix has to be 32F
    */
  template <typename PointT>
  void cvDepth32F2pclCloud(const cv::Mat& depth, const cv::Mat& K, pcl::PointCloud<PointT>& cloud)
  {
    // Check input
    if (pcl::getFieldsList(cloud).find("x y z") == std::string::npos)
    {
      std::cout << pcl::getFieldsList(cloud) << std::endl;
      pcl::console::print_error("[cvDepth2pclCloud] output cloud must contain xyz data");
      exit(EXIT_FAILURE);
    }
    if (K.depth() != CV_32F)
    {
      pcl::console::print_error("[cvDepth2pclCloud] calibration matrix must be CV_32F");
      exit(EXIT_FAILURE);
    }
    
    // Prepare cloud
    const int width   = depth.size().width;
    const int height  = depth.size().height;
    
    cloud.resize(width * height);
    cloud.width = width;
    cloud.height = height;
       
    const float inv_fx  = 1.0 / K.at<float>(0, 0);
    const float inv_fy  = 1.0 / K.at<float>(1, 1);
    const float ox      = K.at<float>(0, 2);
    const float oy      = K.at<float>(1, 2);
    
    bool isDense = true;
    
    for (size_t x = 0; x < width; x++)
    {
      for (size_t y = 0; y < height; y++)
      {
        float z = static_cast<float>(depth.at<float>(y,x)) / 1000;   // Convert milimetres to metres here
        
        if (z < 0.2 || z > 5.0)
        {
          cloud.at(x,y).x = std::numeric_limits<float>::quiet_NaN();
          cloud.at(x,y).y = std::numeric_limits<float>::quiet_NaN();
          cloud.at(x,y).z = std::numeric_limits<float>::quiet_NaN();          
          isDense = false;
        }
        else
        {
          cloud.at(x,y).x = (x-ox)*z*inv_fx;
          cloud.at(x,y).y = (y-oy)*z*inv_fy;
          cloud.at(x,y).z = z;
        }
      } 
    }   
    
    cloud.is_dense = isDense;
  }  
  
  /** \brief Convert an OpenCV depth image to a pointcloud
    * \param[in] depth OpenCV depth image (CV_16U where depth is expressed in milimetres)
    * \param[in] K depth camera calibration matrix
    * \param[out] cloud PCL pointcloud
    * \param[in] depth_scaling scaling applied to depth values
    * \note For now the calibration matrix has to be either 32F or 64F
    */
  template <typename PointT>
  bool cvDepth2pclCloud(const cv::Mat& depth, const cv::Mat& K, pcl::PointCloud<PointT>& cloud, const float depth_scaling = 1.0)
  {
    // Check input
    if (pcl::getFieldsList(cloud).find("x y z") == std::string::npos)
    {
      std::cout <<"[cvDepth2pclCloud] output cloud must contain xyz data" << std::endl;
      return false;
    }
        
    // Prepare cloud
    const int width   = depth.size().width;
    const int height  = depth.size().height;
    
    cloud.resize(width * height);
    cloud.width = width;
    cloud.height = height;
       
    float inv_fx, inv_fy, ox, oy;
    
    if (K.depth() == CV_32F)
    {
      inv_fx  = 1.0 / K.at<float>(0, 0);
      inv_fy  = 1.0 / K.at<float>(1, 1);
      ox      = K.at<float>(0, 2);
      oy      = K.at<float>(1, 2);
    }
    else if (K.depth() == CV_64F)
    {
      inv_fx  = static_cast<float>(1.0 / K.at<double>(0, 0));
      inv_fy  = static_cast<float>(1.0 / K.at<double>(1, 1));
      ox      = static_cast<float>(K.at<double>(0, 2));
      oy      = static_cast<float>(K.at<double>(1, 2));      
    }
    else
    {
      std::cout <<"[cvDepth2pclCloud] camera matrix has to be either CV_32F or CV_64F" << std::endl;
      return false;
    }
    
    bool isDense = true;
    
    for (size_t x = 0; x < width; x++)
    {
      for (size_t y = 0; y < height; y++)
      {
        float z = static_cast<float>(depth.at<unsigned short>(y,x)) / 1000;   // Convert milimetres to metres here
        z*= depth_scaling;
        
        if (z == 0)
        {
          cloud.at(x,y).x = std::numeric_limits<float>::quiet_NaN();
          cloud.at(x,y).y = std::numeric_limits<float>::quiet_NaN();
          cloud.at(x,y).z = std::numeric_limits<float>::quiet_NaN();          
          isDense = false;
        }
        else
        {
          cloud.at(x,y).x = (x-ox)*z*inv_fx;
          cloud.at(x,y).y = (y-oy)*z*inv_fy;
          cloud.at(x,y).z = z;
        }
      } 
    }   
    
    cloud.is_dense = isDense;
    
    return true;
  }  
  
  /** \brief Convert an OpenCV depth image and RGB image pair to a pointcloud
    * \param[in] depth OpenCV depth image (CV_16U where depth is expressed in milimetres)
    * \param[in] rgb   OpenCV RGB image (CV_8UC3)
    * \param[in] K depth camera calibration matrix
    * \param[out] cloud PCL pointcloud
    * \note For now the calibration matrix has to be 32F
    */
  template <typename PointT>
  void cvDepthRGB2pclCloud(const cv::Mat& depth, const cv::Mat& rgb, const cv::Mat& K, pcl::PointCloud<PointT>& cloud)
  {
    // Generate point using depth
    cvDepth2pclCloud(depth, K, cloud);
    
    // Colour points using RGB
    const int width   = depth.size().width;
    const int height  = depth.size().height;

    for (size_t x = 0; x < width; x++)
    {
      for (size_t y = 0; y < height; y++)
      {
        cloud.at(x,y).b = rgb.at<cv::Vec3b>(y,x)[0];
        cloud.at(x,y).g = rgb.at<cv::Vec3b>(y,x)[1];
        cloud.at(x,y).r = rgb.at<cv::Vec3b>(y,x)[2];
      }
    }
  }  
  
//   /* -----------------------------------------------------------------------------
//   * Convert OpenCV normals to PCL normals
//   * -------------------------------------------------------------------------- */
//   inline
//   void cvtNormals_cv2pcl(const cv::Mat& ocvNormals, pcl::PointCloud<pcl::Normal>& pclNormals)
//   {
//     pclNormals.clear();
//     
//     for(int y = 0; y < ocvNormals.rows; y++)
//     {
//       const cv::Point3f* ocvNormalsRow = ocvNormals.ptr<cv::Point3f>(y);
//       for(int x = 0; x < ocvNormals.cols; x++)
//       {
//         const cv::Point3f& ocv_n = ocvNormalsRow[x];
//         pcl::Normal pcl_n (ocv_n.x, ocv_n.y, ocv_n.z);
//         pclNormals.push_back(pcl_n);              
//       }
//     }
//     
//     // Make cloud organized
//     cv::Size s = ocvNormals.size();
//     pclNormals.height = s.height;
//     pclNormals.width = s.width;
//   }  
  
  
}

#endif //#ifndef PCL_OCV_CONVERSIONS_HPP_
