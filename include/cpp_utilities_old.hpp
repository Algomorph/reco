#ifndef CPP_UTILITIES_OLD_HPP
#define CPP_UTILITIES_OLD_HPP

// STL includes
#include <fstream>
#include <iostream>
#include <math.h>
// #include <stdlib.h>

// OpenCV includes
#include <opencv2/core/core.hpp>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/io/io.h>
#include <pcl/PointIndices.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>

// BOOST
#include <boost/filesystem.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/regex.hpp>

#include <vector_sort.hpp>

/* -----------------------------------------------------------------------------
 * Colour pointcloud
 * -------------------------------------------------------------------------- */
template <typename T>
inline
void colourPointCloud(const typename pcl::PointCloud<T>::Ptr &cloud, int r, int g, int b, const std::vector<int> &indices)
{
  // Check input
  if (pcl::getFieldsList(*cloud).find("rgb") == std::string::npos)
  {
    pcl::console::print_error("This cloud does not have rgb fields. Original cloud return;ed.\n");
    return;
  }
  
  // Process
  for (std::vector<int>::const_iterator it = indices.begin(); it != indices.end(); ++it)
  {
//       cloud->points[*it].r = (uint8_t)std::max(0, std::min((int)cloud->points[*it].r+r, 255));
//       cloud->points[*it].g = (uint8_t)std::max(0, std::min((int)cloud->points[*it].g+g, 255));
//       cloud->points[*it].b = (uint8_t)std::max(0, std::min((int)cloud->points[*it].b+b, 255));    
    
    cloud->points[*it].r = (uint8_t)r;
    cloud->points[*it].g = (uint8_t)g;
    cloud->points[*it].b = (uint8_t)b;    
  }
}

/* -----------------------------------------------------------------------------
 * Colour pointcloud
 * -------------------------------------------------------------------------- */
template <typename T>
inline
void colourPointCloud(const typename pcl::PointCloud<T>::Ptr &cloud, int r, int g, int b)
{
  // Create indices that cover all points
  std::vector<int> indices(cloud->size());
  int i = 0;
  for (std::vector<int>::iterator it = indices.begin(); it != indices.end(); it++)
    *it = i++;
  
  // Colourize
  colourPointCloud<T>(cloud, r, g, b, indices);
}

/* -----------------------------------------------------------------------------
 * Generate a random RGB colour
 * -------------------------------------------------------------------------- */
inline
pcl::RGB randomColour ( )
{
  pcl::RGB colour;
  colour.r = std::rand()%255;
  colour.g = std::rand()%255;
  colour.b = std::rand()%255;
  colour.a = 0;
  
  return colour;
}

/* -----------------------------------------------------------------------------
 * Generate colour palette
 * -------------------------------------------------------------------------- */
inline
void generateColourPalette (std::vector<pcl::RGB> &palette, const int &numColours)
{
  palette.clear();
  
  for (int i = 0; i < numColours; i++)
  {
    palette.push_back(randomColour());
  }
}

/** \brief Replace all occurences of a substring with a new substring
  * \param[in,out] string_in string which is modified
  * \param[in] substring_orig substring that needs to be replaced
  * \param[in] substring_new new substring
  */
inline
void replaceSubstring(std::string &string_in, const std::string &substring_orig, const std::string &substring_new)
{
  std::string::size_type n = 0;
  while ( ( n = string_in.find( substring_orig, n ) ) != std::string::npos )
  {
      string_in.replace( n, substring_orig.size(), substring_new );
      n += substring_new.size();
  }  
}

/** \brief An analogue of the MATLAB dir function
  * \param[in] dir_path input directory
  * \param[out] content_list a vector of strings representing names of files/folders in the directory
  */
inline
void dir(const std::string &dir_path, std::vector<std::string> &content_list)
{
  std::string dirPath (dir_path);
  
  // First normalize the beginning of the path
  // If directory path does not start with '/', './' or '../' append './'
  if  (!(((dirPath.length() >= 1) && (dirPath.substr(0,1) == "/")) || 
      ((dirPath.length() >= 2) && (dirPath.substr(0,2) == "./")) ||
      ((dirPath.length() >= 3) && (dirPath.substr(0,3) == "../"))))
  {
    dirPath.insert(0, "./");
  }
    
  // Find the rightmost '/' delimeter and split along it
  std::string left, right;
  size_t found = dirPath.find_last_of("/");
  if (found == std::string::npos)
  {
    std::cout << "[dir] something wrong\n";
    exit(EXIT_FAILURE);
  }
  else
  {
    left = dirPath.substr(0, found+1);
    right = dirPath.substr(found+1);
  }
  
  // Now analyze the right part
  std::string fileFormatStr;
  
  // If right part is empty list everything in left
  if (right.empty())
  {
    fileFormatStr = "*";
    dirPath = left;
  }
  
  // If right side containts a wildcard or an extension list everythin in left that matches
  else if ((right.find("*") != std::string::npos) || (boost::filesystem::path(right).has_extension()))
  {
    fileFormatStr = right;
    dirPath = left;
  }
  
  // If right is a directory append it to left and list everything
  else if (!boost::filesystem::path(right).has_extension())
  {
    fileFormatStr = "*";
    dirPath = left + right;
  }

  // Generate regex expression
  std::string regexExpression (fileFormatStr);
  replaceSubstring(regexExpression, ".", "[.]");      // Replace all occurences of '.' with '[.]'
  replaceSubstring(regexExpression, "*", ".*");       // Replace all occurences of '*' with '.*'
  boost::regex filenameFormatRgx(regexExpression);  
  
  // List
  content_list.resize(0);
  for (boost::filesystem::directory_iterator file_it(dirPath), end; file_it != end; ++file_it)
  {
    std::string curFileName = file_it->path().leaf().string();                          // Get current file name
    
    if (boost::regex_match(curFileName, filenameFormatRgx))                               // Check if it matches the filename format   
      content_list.push_back(curFileName);
  }
  
  // Sort the list
  boost::sort(content_list);  
}

/* -----------------------------------------------------------------------------
 * Create indices representing a rectangular 2D mask for an organized pointcloud.
 * ---------------------------------------------------------------------------*/
inline
void organizedMask2d (int xRes, int yRes, int xMin, int yMin, int width, int height, std::vector<int> &indices)
{
  
  // TOFO: perform checks that masked area is actually inside the image
  
  // Clear indices vector
  indices.clear();
  
  for (int y=yMin; y<yMin + height; y++)
    for (int x=xMin; x<xMin+width; x++)
      indices.push_back(y*xRes + x);
  
}

/* -----------------------------------------------------------------------------
 * Create indices representing a rectangular 2D mask for an organized pointcloud.
 * ---------------------------------------------------------------------------*/
template <typename T>
inline
void organizedMask2d (const typename pcl::PointCloud<T>::ConstPtr& cloud, int xMin, int yMin, int width, int height, std::vector<int> &indices)
{
  int xRes = cloud->width;
  int yRes = cloud->height;
  
  organizedMask2d (xRes, yRes, xMin, yMin, width, height, indices);
}

/* -----------------------------------------------------------------------------
 * Scale pointcloud around its centroid
 * ---------------------------------------------------------------------------*/
template <typename T>
inline
void scalePointCloud(const typename pcl::PointCloud<T> &cloudIn, float scaleFactor, typename pcl::PointCloud<T> &cloudOut)
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid<T>(cloudIn, centroid);
  
  cloudOut.resize(cloudIn.size());
  
  for (int i = 0; i < cloudIn.size(); i++)
  {
    cloudOut.points[i].x = (cloudIn.points[i].x - centroid[0]) * scaleFactor + centroid[0];
    cloudOut.points[i].y = (cloudIn.points[i].y - centroid[1]) * scaleFactor + centroid[1];
    cloudOut.points[i].z = (cloudIn.points[i].z - centroid[2]) * scaleFactor + centroid[2];
  }
}

template <typename T>
inline
void scalePointCloud(const typename pcl::PointCloud<T>::ConstPtr &cloudIn, const std::vector<int> &indices, float scaleFactor, const typename pcl::PointCloud<T>::Ptr &cloudOut)
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid<T>(*cloudIn, centroid);
  
  cloudOut->resize(indices.size());
  
  for (int i = 0; i < indices.size(); i++)
  {
    cloudOut->points[i].x = (cloudIn->points[indices[i]].x - centroid[0]) * scaleFactor + centroid[0];
    cloudOut->points[i].y = (cloudIn->points[indices[i]].y - centroid[1]) * scaleFactor + centroid[1];
    cloudOut->points[i].z = (cloudIn->points[indices[i]].z - centroid[2]) * scaleFactor + centroid[2];
  }
}

/* -----------------------------------------------------------------------------
 * Copy an organized pointcloud and set all point that are not indexed to empty.
 * Empty points have their point data and normal data (if available) set to NaN.
 * The input cloud must be organized. We assume that the indices provided are 
 * unique.
 * ---------------------------------------------------------------------------*/
template <typename T>
inline
void copyPointCloudOrganized (const typename pcl::PointCloud<T> &cloudIn, const std::vector<int> &indices, typename pcl::PointCloud<T> &cloudOut)
{
  //////////////////////////////////////////////////////////////////////////////
  // Perform input checks

  // TODO Check that input and output clouds exist
  
  // Check that input cloud is organized
  if (!cloudIn.isOrganized())
  {
    pcl::console::print_error("[copyPointCloudOrganized] Input cloud must be organized.");
    return;
  }
  if (indices.size() > cloudIn.size())
  {
    pcl::console::print_error("[copyPointCloudOrganized] Number of indices is greater that then number of points in the cloud");
    return;
  }
  
  // TODO check that none of the indices are out of bounds
  
  // If indices index every single point then just return; the input cloud
  if (indices.size () == cloudIn.size ())
  {
    cloudOut = cloudIn;
    return;
  }
  
  //////////////////////////////////////////////////////////////////////////////
  // Reserve output cloud size and set it's header

  // If input and output cloud are the same pointer just update the is_dense property
  if (&cloudIn == &cloudOut)
    cloudOut.is_dense = false;
  
  // Otherwise allocate space in the output and set header for output cloud
  else
  {
    cloudOut.points.resize (cloudIn.points.size());
    cloudOut.header   = cloudIn.header;
    cloudOut.width    = cloudIn.width;
    cloudOut.height   = cloudIn.height;
    cloudOut.is_dense = false;
    cloudOut.sensor_orientation_ = cloudIn.sensor_orientation_;
    cloudOut.sensor_origin_ = cloudIn.sensor_origin_;
  }
   
  /////////////////////////////////////////////////////////////////////////////     
  // Generate an empty point
  T emptyPoint;
  if (pcl::getFieldsList(cloudIn).find("x") != std::string::npos)
    emptyPoint.x = std::numeric_limits<double>::quiet_NaN();
  if (pcl::getFieldsList(cloudIn).find("y") != std::string::npos)
    emptyPoint.y = std::numeric_limits<double>::quiet_NaN();
  if (pcl::getFieldsList(cloudIn).find("z") != std::string::npos)
    emptyPoint.z = std::numeric_limits<double>::quiet_NaN();
  if (pcl::getFieldsList(cloudIn).find("normal") != std::string::npos)
  {
    emptyPoint.normal_x = std::numeric_limits<double>::quiet_NaN();
    emptyPoint.normal_y = std::numeric_limits<double>::quiet_NaN();
    emptyPoint.normal_z = std::numeric_limits<double>::quiet_NaN();
  }
  
  /////////////////////////////////////////////////////////////////////////////
  // Invert indices
  
  std::vector<int> indicesInverted = indicesInvert(indices, cloudIn.points.size());
  
  /////////////////////////////////////////////////////////////////////////////  
  // Set point at inverted indices to empty point
  for (size_t i = 0; i < indicesInverted.size (); i++)
    cloudOut.points[indicesInverted[i]] = emptyPoint;    

  return;
}

namespace pcl
{
  /** \brief Removes points that have their normals invalid (i.e., equal to NaN)
    *
    * This function only computes the mapping between the points in the input
    * cloud and the cloud that would result from filtering. It does not
    * actually construct and output the filtered cloud.
    *
    * \note This function does not modify the input point cloud!
    *
    * \param cloud_in the input point cloud
    * \param index the mapping (ordered): filtered_cloud.points[i] = cloud_in.points[index[i]]
    *
    * \see removeNaNNormalsFromPointCloud
    * \ingroup filters
    */
  
  template <typename PointT> void
  inline
  removeNaNNormalsFromPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                                std::vector<int> &index)
  {
    // Reserve enough space for the indices
    index.resize (cloud_in.points.size ());
    int j = 0;

    for (int i = 0; i < static_cast<int> (cloud_in.points.size ()); ++i)
    {
      if (!pcl_isfinite (cloud_in.points[i].normal_x) || 
          !pcl_isfinite (cloud_in.points[i].normal_y) || 
          !pcl_isfinite (cloud_in.points[i].normal_z))
        continue;
      index[j] = i;
      j++;
    }
    
    if (j != static_cast<int> (cloud_in.points.size ()))
    {
      // Resize to the correct size
      index.resize (j);
    }
  }
}

/* -----------------------------------------------------------------------------
 * Find points that have NaN normals
 * ---------------------------------------------------------------------------*/
template <typename T>
inline
void findNaNNormalsFromPointCloud (const typename pcl::PointCloud<T>::Ptr &cloudIn, std::vector<int> &indices)
{
  // Check input
  if (pcl::getFieldsList(*cloudIn).find("normal") == std::string::npos)
  {
    pcl::console::print_error("[findNaNNormalsFromPointCloud] Pointcloud has no normal data");
    return;
  }
  
  // Process
  indices.clear();
  for (int i = 0; i < cloudIn->size(); i++)
  {
    if (!pcl_isfinite(cloudIn->points[i].normal_x) ||
        !pcl_isfinite(cloudIn->points[i].normal_y) ||
        !pcl_isfinite(cloudIn->points[i].normal_z) )
      indices.push_back(i);
  }
}

/* -----------------------------------------------------------------------------
 * Set points with NaN normals to NaN. If input cloud is organized then it is kept
 * organized
 * ---------------------------------------------------------------------------*/

template <typename T>
inline
void setPointsWithNaNNormalsToNaN (typename pcl::PointCloud<T>::Ptr &cloudIn)
{
  // Check input
  if (pcl::getFieldsList(*cloudIn).find("normal") == std::string::npos)
  {
    pcl::console::print_error("[setPointsWithNaNNormalsToNaN] Pointcloud has no normal data");
    return;
  }  
  
  // Process
  std::vector<int> indices;
  findNaNNormalsFromPointCloud<T>(cloudIn, indices);
    
  for (size_t i = 0; i < indices.size(); i++)
  {
    cloudIn->points[indices[i]].x = std::numeric_limits<double>::quiet_NaN();
    cloudIn->points[indices[i]].y = std::numeric_limits<double>::quiet_NaN();
    cloudIn->points[indices[i]].z = std::numeric_limits<double>::quiet_NaN();
  }
}

  /** \brief Find a transformation matrix that aligns two normals
    * \param target_normal target normal
    * \param source_normal source normal
    * \return 3x3 rotation matrix
    */
  template <typename Scalar>
  inline
  Eigen::Matrix<Scalar,3,3> alignNormals(const Eigen::Matrix<Scalar,1,3> target_normal, const Eigen::Matrix<Scalar,1,3> source_normal)
  {
    Scalar angle                    = acos(target_normal.dot(source_normal));
    Eigen::Matrix<Scalar,1,3> axis  = source_normal.cross(target_normal);
    axis.normalize();
    Eigen::Matrix<Scalar,3,3> R     = Eigen::AngleAxis<Scalar>(angle, axis).toRotationMatrix();
    return R;
  }
    
/** \brief Return a RGB colour value given a scalar v in the range [vmin,vmax]
 * In this case each colour component ranges from 0 (no contribution) to
 * 1 (fully saturated), modifications for other ranges is trivial.
 * The colour is clipped at the end of the scales if v is outside
 * the range [vmin,vmax]
 * \param[in] v reference value
 * \param[in] v_min minimal value
 * \param[in] v_max maximal value
 * \return a vector of vectors of floats representing the colors
 * \note Implementation from here http://paulbourke.net/texture_colour/colourspace/
 */
inline
std::vector<float> getColour(double v,double v_min,double v_max)
{
   std::vector<float> c;
   c.resize(3, 1.0);
   double dv;

   if (v < v_min)
      v = v_min;
   if (v > v_max)
      v = v_max;
   dv = v_max - v_min;

   if (v < (v_min + 0.25 * dv)) {
      c[0] = 0;
      c[1] = 4 * (v - v_min) / dv;
   } else if (v < (v_min + 0.5 * dv)) {
      c[0] = 0;
      c[2] = 1 + 4 * (v_min + 0.25 * dv - v) / dv;
   } else if (v < (v_min + 0.75 * dv)) {
      c[0] = 4 * (v - v_min - 0.5 * dv) / dv;
      c[2] = 0;
   } else {
      c[1] = 1 + 4 * (v_min + 0.75 * dv - v) / dv;
      c[2] = 0;
   }

   return(c);
}

/** \brief Generate a set of RGB colours
  * \param num_colors number of colours to generate
  * \return a vector of arrays of floats representing the colors
  */
  inline
  std::vector<std::vector<float> > colourPalette (size_t n_colours)
  {
    
    // Generate linear space
    float linspace[n_colours];
    for (size_t i = 0; i < n_colours; i++)
      linspace[i] = i * 1.0/(static_cast<float>(n_colours)-1.0);
    
    // Permute linear space to make adjacent colours distinguishable
    float linspacePermuted[n_colours];
    size_t numRegions = 3;

    if (n_colours > numRegions*2)
    {
      size_t shift = n_colours / numRegions;
      size_t curIdx = 0;
      for (size_t i = 0; i < shift; i++)
      {
        size_t k = (n_colours-i+shift-1)/shift;
        for (size_t j = 0; j < k; j++)
        {
          linspacePermuted[curIdx++] = linspace[i + shift * j];
        }
      }  
    }
    else
    {
      for (size_t i = 0; i < n_colours; i++)
      {
        linspacePermuted[i] = linspace[i];
      }
    }
    
    // Generate colours
    std::vector<std::vector<float> > colours;
    colours.resize(n_colours);
    for (size_t i = 0; i < n_colours; i++)
    {
      colours[i] = getColour(linspacePermuted[i], 0, 1);
    }

    return colours;
  }  

namespace pcl
{
  /** \brief Save cloud point correspondence data to a file in ASCII format
    * \param[in] filename filename
    * \param[in] correspondences correspondneces that are saved
    */      
  inline
  bool saveCorrespondences(const std::string filename, const pcl::Correspondences &correspondences)
  {
    std::ofstream out(filename.c_str(), std::ios::out);
    if (out.is_open())
    {
      for (size_t i = 0; i < correspondences.size(); i++)
      {
        out << correspondences[i].index_query << " " << correspondences[i].index_match << std::endl; 
      }
      out.close();
    }
    else
    {
      std::cout << "[pcl::saveCorrespondences] could not open correspondences file '" << filename << "'\n";
      return false;
    }
    return true;
  }

  /** \brief Load cloud point correspondence data to from a file in ASCII format
    * \param[in] filename filename
    * \param[out] correspondences correspondneces that are saved
    */        
  inline
  bool loadCorrespondences(const std::string filename, pcl::Correspondences &correspondences)
  {
    std::ifstream in(filename.c_str(), std::ios::in);
    if (in.is_open())
    {
      int index_query, index_match; 
      while (in >> index_query >> index_match)
      {
        correspondences.push_back(pcl::Correspondence(index_query, index_match, std::numeric_limits<float>::max ()));
      }
      in.close();
    }
    else
    {
      std::cout << "[pcl::loadCorrespondences] could not open correspondences file '" << filename << "'\n";
      return false;
    }
    return true;
  }  
}

/** \brief Limit a value from above and below
  * \param[in] value value to be limited
  * \param[in] minValue minimum value
  * \param[in] maxValue maximum value
  * \return bounded value
  */        
template <typename Scalar>
inline
Scalar LimitBelowAbove(const Scalar &value, const Scalar &minValue, const Scalar &maxValue)
{
  Scalar valueBounded = value;
  valueBounded = std::max(valueBounded, minValue);
  valueBounded = std::min(valueBounded, maxValue);
  return valueBounded;
}

/** \brief Compute a occurences of integer values in a vector
  * \param[in] values a vector of integer values
  * \return a vector of pairs where first value is integer value and second is 
  * number of occurences of the value
  * \note this is an inefficient implementation
  */        
inline
std::vector<std::pair<int, int> > intVectorCount(const std::vector<int> &values)
{ 
  // First get all the unique values of the vector
  std::vector<int> uniqueValues = values;
  std::sort(uniqueValues.begin(), uniqueValues.end());
  uniqueValues.erase(std::unique(uniqueValues.begin(), uniqueValues.end()), uniqueValues.end());
  
  // Count the occurence of each value
  std::vector<std::pair<int, int> > counts (uniqueValues.size());
  
  for (size_t i = 0; i < uniqueValues.size(); i++)
  {
    int count = std::count(values.begin(), values.end(), uniqueValues[i]);
    counts[i] = std::pair<int, int>(uniqueValues[i], count);
  }
 
  return counts;
}


#endif  // CPP_UTILITIES_OLD_HPP