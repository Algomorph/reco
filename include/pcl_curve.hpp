#ifndef PCL_CURVE_HPP
#define PCL_CURVE_HPP

#include <pcl/common/distances.h>
#include <pcl/common/pca.h>
#include <pcl/common/io.h>

namespace pcl
{
  
  /** Invert curve direction
  * \param[in] curve             curve
  * \return inverted curve
  */  
  template <typename PointT>    
  typename pcl::PointCloud<PointT> invertCurveDirection (const typename pcl::PointCloud<PointT> &curve)
  {
    pcl::PointCloud<PointT> inverted_curve;
    inverted_curve.resize(curve.size());
    for (size_t i = 0; i < curve.size(); i++)
      inverted_curve[curve.size()-1-i] = curve[i];
    
    return inverted_curve;
  }
  
  /** Estimate the tangent and curvature of a curve in 3D space.
  * \param[in] curve             curve
  * \param[in] radius            radius within which neighbours are searched
  */  
  template <typename PointT>    
  void estimateCurveTangentRadius (typename pcl::PointCloud<PointT>::Ptr &curve, const float &radius)
  {
    // For all points
    for (size_t pointId = 0; pointId < curve->size(); pointId++)
    {
      //////////////////////////////////////////////////////////////////////////
      // Find neighbours within radius
      std::vector<int> neighboursPos, neighboursNeg;
      pcl::PointIndicesPtr neighbours (new pcl::PointIndices);
      
      // Right neighbours
      for (size_t nbrPosId = pointId+1; nbrPosId < curve->size(); nbrPosId++)
      {
        if (pcl::euclideanDistance(curve->points[pointId], curve->points[nbrPosId]) < radius)
          neighboursPos.push_back(nbrPosId);
        else
          break;
      }

      // Left neighbours
      for (size_t nbrNegId = pointId-1; nbrNegId < curve->size(); nbrNegId++)
      {
        if (pcl::euclideanDistance(curve->points[pointId], curve->points[nbrNegId]) < radius)
          neighboursPos.push_back(nbrNegId);
        else
          break;
      }
      
      // Combine all neighbours
      neighbours->indices.push_back(pointId);
      neighbours->indices.insert(neighbours->indices.end(), neighboursPos.begin(), neighboursPos.end());
      neighbours->indices.insert(neighbours->indices.end(), neighboursNeg.begin(), neighboursNeg.end());
    
      //////////////////////////////////////////////////////////////////////////
      // Run PCA on points
      
      pcl::PCA<PointT> pcaSolver;
      pcaSolver.setInputCloud(curve);
      pcaSolver.setIndices(neighbours);
      
      curve->points[pointId].getNormalVector3fMap() = pcaSolver.getEigenVectors().col(0);
      Eigen::Vector3f eiv = pcaSolver.getEigenValues();
      curve->points[pointId].curvature = (eiv(1) + eiv(2))/ eiv.sum(); 
    }
  }  

  /** Estimate the tangent and curvature of a curve in 3D space.
  * \param[in] curve             curve
  * \param[in] radius            radius within which neighbours are searched
  */  
  template <typename PointT>    
  void estimateCurveTangentNearestK (typename pcl::PointCloud<PointT>::Ptr &curve, const int &k)
  {
    // For all points
    for (size_t pointId = 0; pointId < curve->size(); pointId++)
    {
      //////////////////////////////////////////////////////////////////////////
      // Find neighbours within radius
      int neighbourIdMin = std::max(0, static_cast<int>(pointId) - k);
      int neighbourIdMax = std::min(curve->size(), pointId + k);
      pcl::PointIndicesPtr neighbours (new pcl::PointIndices);
      for (size_t neighbourId = neighbourIdMin; neighbourId < neighbourIdMax; neighbourId++)
        neighbours->indices.push_back(neighbourId);
      
      ////////////////////////////////////////////////////////////////////////////    
      // Run PCA on points
      
      pcl::PCA<PointT> pcaSolver;
      pcaSolver.setInputCloud(curve);
      pcaSolver.setIndices(neighbours);
      
      curve->points[pointId].getNormalVector3fMap() = pcaSolver.getEigenVectors().col(0);
      Eigen::Vector3f eiv = pcaSolver.getEigenValues();
      curve->points[pointId].curvature = (eiv(1) + eiv(2))/ eiv.sum(); 
    }
  }  
  
  /** Enumerator for different order of linking two curves depending on which endpoints are joined
   */
  enum CurveLinkOrder {START_START, START_END, END_START, END_END};
  
  /** Get curve link order given two flags
  * \param[in] start_1    is the endpoint of first curve at start?
  * \param[in] start_2    is the endpoint of second curve at start?
  * \return curve link order
  */
  CurveLinkOrder getCurveLinkOrder (const bool start_1, const bool start_2)
  {
    if (start_1 && start_2)
      return START_START;
    
    if (start_1 && !start_2)
      return START_END;
    
    if (!start_1 && start_2)
      return END_START;
    
    if (!start_1 && !start_2)
      return END_END;    
  }
  
  /** Link two curves into a single curve
  * \param[in] curve1             first curve
  * \param[in] curve2             second curve
  * \param[in] curve_link_order   which endpoint should be connected
  * \param[out] curve_linked      linked curve
  * \param[out] join_link         indices of the points in the linked curve where curves were joined
  * \return false if unknown order is used, true otherwise
  */
  template <typename PointT>
  bool linkCurves ( const typename pcl::PointCloud<PointT> &curve1,
                    const typename pcl::PointCloud<PointT> &curve2,
                    const CurveLinkOrder curve_link_order,  
                    typename pcl::PointCloud<PointT> &curve_linked,
                    std::pair<int, int> &join_link
                  )
  {
    switch (curve_link_order)
    {
      case END_START:
        pcl::copyPointCloud(curve1, curve_linked);
        curve_linked += curve2;
        join_link = std::pair<int,int> (curve1.size()-1, curve1.size());
        break;
        
      case START_END:
        pcl::copyPointCloud(curve2, curve_linked);
        curve_linked += curve1;
        join_link = std::pair<int,int> (curve2.size()-1, curve2.size());
        break;
        
      case START_START:
        curve_linked = invertCurveDirection<PointT>(curve1) + curve2;
        join_link = std::pair<int,int> (curve1.size()-1, curve1.size());
        break;
        
      case END_END:
        pcl::copyPointCloud(curve1, curve_linked);
        curve_linked += invertCurveDirection<PointT>(curve2);
        join_link = std::pair<int,int> (curve1.size()-1, curve1.size());
        break;
        
      default:
        std::cout << "[pcl::linkCurves] unknown link order\n";
        return false;
    }   
    
    return true;
  }  
}

#endif  // PCL_CURVE_HPP