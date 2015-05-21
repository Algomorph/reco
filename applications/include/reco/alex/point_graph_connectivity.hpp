#ifndef POINT_GRAPH_CONNECTIVITY_HPP
#define POINT_GRAPH_CONNECTIVITY_HPP

#include <adjacency_list.hpp>
#include <pcl/search/kdtree.h>

namespace utl
{ 
  /** Generate graph structure representing local connectivity between points in
    * the pointcloud. Each point is connected to its k nearest neighbors.
    * \param[in] cloud             input cloud
    * \param[in] num_neighbours    maximum number of neighbours
    * \param[out] g                graph
    * \return false if no edges were found, true otherwise
    */
  template <typename PointT>
  inline
  bool getLocalConnectivityGraphNearestK (  const typename pcl::PointCloud<PointT>::ConstPtr  &cloud,
                                            const int                                         &num_neighbours,
                                            Graph                                             &g
                                 )
  {
    // Prepare graph structure
    g.clear();
    g.resize(cloud->size());
    
    // Prepare search tree
    pcl::search::KdTree<PointT> searchTree;
    searchTree.setInputCloud(cloud);

    // Loop over all points
    for (size_t pointId = 0; pointId < cloud->size(); pointId++)
    { 
      // Find nearest neighbours
      std::vector<float>  distances(num_neighbours);
      std::vector<int>    neighbors(num_neighbours);
      searchTree.nearestKSearch(pointId, num_neighbours, neighbors, distances);
          
      // Add corresponding edges to the graph
      for (size_t nbrId = 1; nbrId < neighbors.size(); nbrId++)
        addEdge(pointId, neighbors[nbrId], g);
    }
        
    // If there are no edges - return false
    if (getNumEdges(g) < 1)
    {
      std::cout << "[utl::getLocalConnectivityGraph] no neighbouring points were found\n";
      return false;
    }
      
    // Otherwise return true
    return true;
  }
  
  /** Generate graph structure representing local connectivity between points in
    * the pointcloud. Each point is connected to it's k nearest neighbors within a
    * radius r.
    * \param[in] cloud             input cloud
    * \param[in] radius            radius within which neighbours are searched
    * \param[out] g                 graph
    * \param[in] num_neighbours    maximum number of neighbours (if set to 0 - all neighbours will be included)
    * \return false if no edges were found, true otherwise
    */
  template <typename PointT>
  inline
  bool getLocalConnectivityGraphRadius (  const typename pcl::PointCloud<PointT>::ConstPtr  &cloud,
                                          const double                                      &radius,                                          
                                          Graph                                             &g,
                                          const int                                         &num_neighbours=0
                                 )
  {
    // Prepare graph structure
    g.clear();
    g.resize(cloud->size());
    
    // Prepare search tree
    pcl::search::KdTree<PointT> searchTree;
    searchTree.setInputCloud(cloud);

    // Loop over all points
    for (size_t pointId = 0; pointId < cloud->size(); pointId++)
    { 
      // Find nearest neighbours
      std::vector<float>  distances(num_neighbours);
      std::vector<int>    neighbors(num_neighbours);
      searchTree.radiusSearch(pointId, radius, neighbors, distances, num_neighbours);
          
      // Add corresponding edges to the graph
      for (size_t nbrId = 1; nbrId < neighbors.size(); nbrId++)
        addEdge(pointId, neighbors[nbrId], g);        
    }
        
    // If there are no edges - return false
    if (getNumEdges(g) < 1)
    {
      std::cout << "[utl::getLocalConnectivityGraph] no neighbouring points were found\n";
      return false;
    }
      
    // Otherwise return true
    return true;
  }  
}

#endif  // POINT_GRAPH_CONNECTIVITY_HPP