#ifndef POINT_CLOUD_MASK_
#define POINT_CLOUD_MASK_

// PCL includes
#include <pcl/pcl_base.h>
#include <pcl/console/print.h>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <boost/concept_check.hpp>

namespace pcl
{
  typedef std::vector<bool> PointCloudMask;
  typedef boost::shared_ptr <std::vector<bool> > PointCloudMaskPtr;
  typedef boost::shared_ptr <const std::vector<bool> > PointCloudMaskConstPtr;

  /** \brief Convert pointcloud mask to indices
    * \param[in] mask pointcloud mask
    * \param[out] pointcloud indices
    */  
  inline
  void mask2indices (const PointCloudMask &mask, std::vector<int> &indices)
  {    
    indices.resize(mask.size());
    unsigned int numIndices = 0;
    
    for (int i = 0; i < mask.size(); i++)
    {
      if (mask.at(i))
        indices.at(numIndices++) = i;
    }
    indices.resize(numIndices);
  }
  
  /** \brief Convert pointcloud indices to a mask
    * \param[in] indices pointcloud indices
    * \param[in] num_points number of points in the cloud
    * \param[out] mask pointcloud mask
    */
  inline  
  void indices2mask (const std::vector<int> &indices, const unsigned int num_points, PointCloudMask &mask)
  { 
    mask.assign(num_points, false);
    
    for (size_t i = 0; i < indices.size(); i++)
    {
      int curIndex = indices.at(i);
      if (curIndex >= num_points)
        console::print_error("[pcl::indices2mask] index value '%d' exeeds or equals number of points '%d'\n", curIndex, num_points);
      else
        mask.at(curIndex) = true;
    }
  }

  /** \brief Convert an OpenCV matrix representing a binary mask to a PCL binary mask
    * \param[in] mask_cv OpenCV mask
    * \param[in] mask_pcl PCL mask
    */
  inline  
  void cvMask2pclMask (const cv::Mat &mask_cv, PointCloudMask &mask_pcl)
  {
    // Chech input
    CV_Assert(!mask_cv.empty());
    CV_Assert(mask_cv.type() == CV_8U);
    
    // Process
    int xRes = mask_cv.size().width;
    int yRes = mask_cv.size().height;

    mask_pcl.assign(xRes*yRes, false);

    size_t curIndex = 0;
    
    for (int y = 0; y < yRes; y++)
    {
      for (int x = 0; x < xRes; x++)
      {
        if (mask_cv.at<uchar>(y,x) > 0)
        {
          mask_pcl.at(curIndex) = true;    
        }
        curIndex++;
      }
    }
  }

  /** \brief Convert a PCL binary mask to an OpenCV matrix representing a binary mask
    * \param[in] mask_pcl PCL mask
    * \param[in] mask_cv OpenCV mask
    * \param[in] x_res cloud width
    * \param[in] y_res cloud height
    */
  inline  
  void pclMask2cvMask (const PointCloudMask &mask_pcl, cv::Mat &mask_cv, size_t x_res, size_t y_res)
  {
    // Chech input
    CV_Assert(x_res * y_res  == mask_pcl.size());
    
    mask_cv = cv::Mat::zeros(y_res, x_res, CV_8U);

    size_t curIndex = 0;
    for (int y = 0; y < y_res; y++)
    {
      for (int x = 0; x < x_res; x++)
      {
        if (mask_pcl.at(curIndex))
        {
          mask_cv.at<uchar>(y,x) = 255;
        }
        curIndex++;        
      }
    }
  }  
  
  /** \brief Apply a binary mask to the pointcloud
    * \param[in] cloud_in input cloud
    * \param[in] mask binary mask
    * \param[out] cloud_out output cloud
    * \param[in] keep_organized if true and input cloud is organized points are replaced with NaN instead of being removed
    * \note Input and output can be the same
    */
  
  template <typename PointT> inline
  void maskPointCloud(const PointCloud<PointT> &cloud_in, const PointCloudMask &mask, PointCloud<PointT> &cloud_out, bool keep_organized = false)
  {
    // Check input    
    if (mask.size() != cloud_in.size())
    {
      console::print_error("[pcl::maskPointCloud] number of points in the mask and cloud are not equal (cloud_in %d, mask %d)\n", cloud_in.size(), mask.size());
      exit(EXIT_FAILURE);
    }
    if (!cloud_in.isOrganized() && keep_organized)
    {
      console::print_error("[pcl::maskPointCloud] input cloud is not organized but keep_organized is requested\n");
      exit(EXIT_FAILURE);
    }

    // If output is different from input copy the header
    if (&cloud_in != &cloud_out)
    { 
      cloud_out.resize(cloud_in.size());
      cloud_out.is_dense             = cloud_in.is_dense;
      cloud_out.sensor_orientation_  = cloud_in.sensor_orientation_;
      cloud_out.sensor_origin_       = cloud_in.sensor_origin_;
      cloud_out.header               = cloud_in.header;  
      cloud_out.width                = cloud_in.width;
      cloud_out.height               = cloud_in.height;
    }
    
    // Modify data
    size_t numPointsCopied = 0;
    
    if (!keep_organized)
    {
      for (size_t i = 0; i< mask.size(); i++)
      {
        if (mask.at(i))
          cloud_out.points[numPointsCopied++] = cloud_in.points[i];
      }
      
      // If some data was not copied resize and modify the fields
      if (numPointsCopied < cloud_in.size())
      {
        cloud_out.resize(numPointsCopied);
        cloud_out.width    = numPointsCopied;
        cloud_out.height   = 1;
      }
      return;
    }
    
    if (keep_organized)
    {
      // Generate an empty point
      PointT emptyPoint;
      if (pcl::getFieldsList(cloud_in).find("x") != std::string::npos)
        emptyPoint.x = std::numeric_limits<double>::quiet_NaN();
      if (pcl::getFieldsList(cloud_in).find("y") != std::string::npos)
        emptyPoint.y = std::numeric_limits<double>::quiet_NaN();
      if (pcl::getFieldsList(cloud_in).find("z") != std::string::npos)
        emptyPoint.z = std::numeric_limits<double>::quiet_NaN();
      
      // Mask points
      for (size_t i = 0; i< mask.size(); i++)
      {
        if (mask.at(i))
          cloud_out.points[i] = cloud_in.points[i];
        else
          cloud_out.points[i] = emptyPoint;
      }      
    }
  }
  
  /** \brief Apply a binary mask to another binary mask
    * \param[out] mask_out output mask
    * \param[in] mask_in input mask
    * \note Input and output masks can not be the same
    */
  inline
  void maskMask(pcl::PointCloudMask &mask_out, const pcl::PointCloudMask &mask_in)
  {
    // Check input    
    if (&mask_in == &mask_out)
    {
      console::print_error("[pcl::maskMask] output mask must be different from input mask\n");
      exit(EXIT_FAILURE);      
    }
    
    if (mask_in.size() != mask_out.size())
    {
      console::print_error("[pcl::maskMask] mask sizes must be the same (input %d, output %d)\n", mask_in.size(), mask_out.size());
      exit(EXIT_FAILURE);      
    }
    
    // Modify data
    size_t numPointsCopied = 0;
    for (size_t i = 0; i< mask_in.size(); i++)
    {
      if (mask_in.at(i))
      {
        mask_out.at(numPointsCopied++) = mask_out.at(i);
      }
    }
    mask_out.resize(numPointsCopied);
  }

  /** \brief Convert an opencv mask to a vector of indices. Opencv mask has to 
   * be of type CV_8U. All values greater than 0 in the mask are considered 
   * TRUE.
   * \param[in] mask_cv OpenCV mask
   * \param[out] mask_in pcl indices
   */
  inline
  void cvMask2pclIndices (const cv::Mat &mask_cv, std::vector<int> &indices)
  {
    // Chech input
    CV_Assert(!mask_cv.empty());
    CV_Assert(mask_cv.type() == CV_8U);
    
    // Process
    int xRes = mask_cv.size().width;
    int yRes = mask_cv.size().height;
    
    for (int x = 0; x < xRes; x++)
      for (int y = 0; y < yRes; y++)
        if (mask_cv.at<uchar>(y,x) > 0)
          indices.push_back(y*xRes + x);
  }  
  
  /** \brief Convert pcl indices to opencv binary mask.
   * \param[in] indices PCL indices
   * \param[out] mask_cv OpenCV mask
   * \param[in] x_res height of the pointcloud indexed by indeces
   * \param[in] y_res width of the pointcloud indexed by indeces
   */
  inline
  void pclIndices2cvMask (const std::vector<int> &indices, cv::Mat &mask_cv, int x_res, int y_res)
  {
    // TODO: check that indices are not out of bounds
    
    mask_cv = cv::Mat::zeros(y_res, x_res, CV_8U);
    
    for (size_t i = 0; i<indices.size(); i++)
    {
      int x = indices[i] % x_res;
      int y = indices[i] / x_res;
      mask_cv.at<uchar>(y, x) = 255;
    }
  }


  /** \brief Convert pcl indices to opencv binary mask.
   * \param[in] indices PCL indices
   * \param[out] mask_cv OpenCV mask
   * \param[in] cloud pointcloud indexed by indices
   */
  template <typename T> inline
  void pclIndices2cvMask (const std::vector<int> &indices, cv::Mat &mask_cv, const pcl::PointCloud<T> &cloud)
  {
    // Check input
    if (!cloud.isOrganized())
    {
      pcl::console::print_error("[pclIndices2cvMask] Input pointcloud must be organized.\n");
      return;
    }
    
    // Process
    pclIndices2cvMask(indices, mask_cv, (int)cloud.width, (int)cloud.height);
  }
  
//   /** \brief Find the union of two vectors of integers. Resulting vector has 
//    * elements ordered
//    * \param[in] indices_in1 first input vector
//    * \param[in] indices_in2 second input vector
//    * \return union of vevtors
//    */
//   inline
//   std::vector<int> indicesUnion(const std::vector<int> &indices_in1, const std::vector<int> &indices_in2)
//   {
//     std::vector<int> indices_union;
//     std::vector<int> indices_in1_copy = indices_in1;
//     std::vector<int> indices_in2_copy = indices_in2;
//     std::sort(indices_in1_copy.begin(), indices_in1_copy.end());
//     std::sort(indices_in2_copy.begin(), indices_in2_copy.end());
//     std::set_intersection(indices_in1_copy.begin(), indices_in1_copy.begin(), indices_in2_copy.begin(), indices_in2_copy.end(), std::back_inserter(indices_union));
//     
//     return indices_union;
//   }
// 
// /* -----------------------------------------------------------------------------
//  * Find the intersection of two vectors of integers. Resulting vector has elements ordered.
//  * ---------------------------------------------------------------------------*/
//   /** \brief Find the intersection of two vectors of integers. Resulting vector 
//    * has elements ordered.
//    * \param[in] indices_in1 first input vector
//    * \param[in] indices_in2 second input vector
//    * \return intersection of vevtors
//    */
//   inline
//   std::vector<int> indicesIntersection(const std::vector<int> &indices_in1, const std::vector<int> &indices_in2)
//   {
//     std::vector<int> indices_intersection;
//     std::vector<int> indices_in1_copy = indices_in1;
//     std::vector<int> indices_in2_copy = indices_in2;
//     std::sort(indices_in1_copy.begin(), indices_in1_copy.end());
//     std::sort(indices_in2_copy.begin(), indices_in2_copy.end());
//     std::set_intersection(indices_in1_copy.begin(), indices_in1_copy.end(), indices_in2_copy.begin(), indices_in2_copy.end(), std::back_inserter(indices_intersection));
//     
//     return indices_intersection;
//   }

  /** \brief invert a set of indices
   * \param[in] indices_in input indices
   * \param[in] cloud_size size of the indexed cloud
   * \return inverted indices
   */
  inline
  std::vector<int> indicesInvert (const std::vector<int> indices_in, const int cloud_size)
  {
    std::vector<int> indices_inverted;
    
    // Check that the number of indices is smaller then cloud size
    if (indices_in.size() > cloud_size)
    {
      pcl::console::print_error("[invertIndices] Number of indices is greater that cloud size");
      return indices_inverted;
    }
    
    // Construct full indices
    std::vector<int> full_indices (cloud_size);
    for (int fii = 0; fii < cloud_size; fii++)  // fii = full indices iterator
      full_indices[fii] = fii;
    
    // Check if input indices are empty
    if (indices_in.empty())
      return full_indices;
    
    // Sort input indices
    std::vector<int> indices_in_copy = indices_in;
    std::sort(indices_in_copy.begin(), indices_in_copy.end());
    
    // Calculate difference between full indices and input indices
    std::set_difference(full_indices.begin(), full_indices.end(), indices_in_copy.begin(), indices_in_copy.end(), std::back_inserter(indices_inverted));
    
    return indices_inverted;
  }  
  
}

#endif // POINT_CLOUD_MASK_