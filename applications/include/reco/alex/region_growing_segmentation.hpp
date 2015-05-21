#ifndef REGION_GROWING_SEGMENTATION_HPP
#define REGION_GROWING_SEGMENTATION_HPP

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/angles.h>

namespace utl
{
  /** \brief Segment pointcloud using regiongrowing segmentation. It is assumed that input cloud was downsampled to some
    * spatial resolution (using voxelGrid filter)
    * \param[in]  cloud                 input pointcloud
    * \param[in]  segments              indices of points belonging to different segments
    * \param[in]  spatial_resolution    spatial resolution of the input cloud (points per meter)
    * \param[in]  max_normal_variation  maximum allowed variation of normal (radians per meter)
    * \param[in]  min_segment_radius    minimum radius of a patch accepted as a segment (metres)
    * \param[in]  curvature_thresh      curvature threshold
    * \param[in]  num_neighbours        number of neighbours used in growing each point
    * \note For best results the surface normals of the cloud should be smoothed to avoid ovsersegmentation due to noisy normals.
    * This can be achieved by downsampling the cloud to lower resolution. Segmentation results will break down if cloud normals 
    * are noisy and clouds's spatial resolution is close to the resolution at which it was acquired.
    */
  template <typename PointT>
  void regionGrowingSegmentation( const typename pcl::PointCloud<PointT>::Ptr &cloud,
                                std::vector<std::vector<int> > &segments,
                                const float &spatial_resolution,
                                const float &max_normal_variation,
                                const float &min_segment_radius,
                                const float &curvature_thresh = 1.0,
                                const int   &num_neighbours = 10
                              )
  {
    //---------------------------------------------------------------------------
    // First figure out parameters
    //---------------------------------------------------------------------------
    
    // Number of points in smallest segment
    float S = M_PI * pow(min_segment_radius, 2);
    int minSegmentPoints = pow(min_segment_radius * spatial_resolution, 2);
    
    // Maximum normal angle difference between neighbouring points
    float normalAngleThresh = max_normal_variation / spatial_resolution;
        
    //---------------------------------------------------------------------------
    // Create segmentation object
    //---------------------------------------------------------------------------
    
    typename pcl::search::KdTree<PointT>::Ptr searchTree (new pcl::search::KdTree<PointT>);
    pcl::RegionGrowing<PointT, PointT> reg;
    reg.setSearchMethod (searchTree);
    reg.setSmoothnessThreshold (normalAngleThresh);                               // Threshold controlling the maximum allowed angle between normals
    reg.setCurvatureThreshold (curvature_thresh);                                 // Threshold controlling the ovsersegmentation/udnersegmentation. Higher value - more ovsersegmentation        
    reg.setMinClusterSize (minSegmentPoints);                                     // Minimum number of points in a segment
    reg.setMaxClusterSize (cloud->size());                                        // Maximum number of points in a segment
    reg.setNumberOfNeighbours (num_neighbours);                                  // Number of neighbours for KNN search
    reg.setInputCloud (cloud);
    reg.setInputNormals (cloud);
    
    //---------------------------------------------------------------------------  
    // Segment
    //---------------------------------------------------------------------------  
    
    std::vector <pcl::PointIndices> segments_tmp;
    reg.extract (segments_tmp);
    
    // Convert output
    segments.resize(segments_tmp.size());
    for (size_t segId = 0; segId < segments_tmp.size(); segId++)
      segments[segId] = segments_tmp[segId].indices;  
  }
}


#endif  // REGION_GROWING_SEGMENTATION_HPP