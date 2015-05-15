#ifndef SUPERVOXEL_SEG_HPP
#define SUPERVOXEL_SEG_HPP

#include <pcl/common/angles.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/surface/gp3.h>
#include <cpp_utilities.hpp>
#include <adjacency_list.hpp>

namespace utl
{
  // Typedefs
  typedef pcl::PointXYZ         Point;
  typedef pcl::Normal           PointN;
  typedef pcl::PointNormal      PointNT;
  typedef pcl::PointXYZL        PointLT;
  typedef pcl::PointXYZLNormal  PointLNT;

  /** \brief Convert between tow types of representing segmentation infromation
    * \param[in]  point_labels  a vector where each element represents a point in the pointcloud and holds the cluster id it was assigned to
    * \return     a vector where each element represents a cluster and holds the indices of points in the input cloud belonging to the cluster
    */
  std::vector<std::vector<int> > pointLabels2clusters (const std::vector<int> &point_labels)
  { 
    // Get the number of the highest cluster id
    int numClusters = *std::max_element(point_labels.begin(), point_labels.end());
    numClusters = numClusters+1;
      
    // Get cluster assignment
    std::vector<std::vector<int> >  clusters(numClusters);
    for (size_t i = 0; i < point_labels.size(); i++)
      clusters[point_labels[i]].push_back(i);
    
    return clusters;
  }

  /** \brief Convert between tow types of representing segmentation infromation
    * \param[in]  clusters  a vector where each element represents a cluster and holds the indices of points in the input cloud belonging to the cluster
    * \return     a vector where each element represents a point in the pointcloud and holds the cluster id it was assigned to
    * \note       Points that are not assigned to any cluster have label -1
    */
  std::vector<int> clusters2pointLabels (const std::vector<std::vector<int> > &clusters)
  {
    // Get the number of points
    int numPoints = 0;
    for (size_t clusterId = 0; clusterId < clusters.size(); clusterId++)
      numPoints = std::max(numPoints, *std::max_element(clusters[clusterId].begin(), clusters[clusterId].end()));
    numPoints = numPoints+1;
    
    // Get point assignment
    std::vector<int> pointLabels(numPoints, -1);
    
    for (size_t clusterId = 0; clusterId < clusters.size(); clusterId++)
    {
      for (size_t pointId = 0; pointId < clusters[clusterId].size(); pointId++)
      {
        pointLabels[clusters[clusterId][pointId]] = clusterId;
      }
    }
    
    return pointLabels;
  }

  /** \brief Oversegment input pointcloud
    * \param[in]  cloud               input pointcloud
    * \param[in]  voxel_resolution    spatial resolution of the voxelgrid filter applied before segmentation
    * \param[in]  seed_resolution     distance between cluster seeds
    * \param[in]  spatial_importance  weight of the Euclidean distance term used for calculating distances between points and cluster seeds
    * \param[in]  normal_importance   weight of the normal difference term used for calculating distances between points and cluster seeds
    * \param[out] clusters            a vector where each element represents a cluster and holds the indices of points in the input cloud belonging to the cluster
    * \param[out] adjacency           a vector where each element represents a cluster and holds the indices of its neighbouring clusters
    * \param[out] centroids           a vector where each element represents a cluster and holds cluster centroid location and normal
    */
  template <typename PointT>
  bool supervoxelSegmentation(  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                const float voxel_resolution,
                                const float seed_resolution,
                                const float spatial_importance,
                                const float normal_importance,
                                std::vector<int> &point_labels,
                                utl::Graph &adjacency,
                                pcl::PointCloud<PointNT> &centroids
                             )
  {
    // Parameters
    int numRefinementIterations = 5;
    
    // Extract normal and point data into separate clouds
    pcl::PointCloud<Point>::Ptr cloudPoints  (new pcl::PointCloud<Point>);
    pcl::PointCloud<PointN>::Ptr cloudNormals (new pcl::PointCloud<PointN>);
    pcl::copyPointCloud(*cloud, *cloudPoints);
    pcl::copyPointCloud(*cloud, *cloudNormals);

    // Setup supervoxel clustering
    pcl::SupervoxelClustering<Point> super (voxel_resolution, seed_resolution, false);
    super.setInputCloud(cloudPoints);
    super.setNormalCloud(cloudNormals);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);
    super.setColorImportance(0.0f);

    // Segment
    std::map <uint32_t, pcl::Supervoxel<Point>::Ptr >  supervoxel_clusters;      
    std::multimap<uint32_t, uint32_t>                   label_adjacency;
    super.extract (supervoxel_clusters);                                          // Get supervoxel clusters
    super.refineSupervoxels(numRefinementIterations, supervoxel_clusters);                              // Refine clusters
    super.getSupervoxelAdjacency (label_adjacency);                               // Get supervoxel adjacency
    pcl::PointCloud<PointLT>::Ptr labeledCloud = super.getLabeledCloud ();        // Get input cloud labels
                 
    // Extract cluster assignment information
    point_labels.resize(labeledCloud->size());
    for (size_t i = 0; i < labeledCloud->size(); i++)
      point_labels[i] = labeledCloud->points[i].label;

    // Extract adjacency information
    adjacency.resize(super.getMaxLabel()+1);
    for (auto edgeIt = label_adjacency.begin(); edgeIt != label_adjacency.end(); edgeIt++)
    {
      if (!utl::addEdge(edgeIt->first, edgeIt->second, adjacency))
        std::cout << edgeIt->first << std::endl;
    }

    // Extract centroids
    centroids.resize(super.getMaxLabel()+1);
    for (auto centroidIt = supervoxel_clusters.begin(); centroidIt != supervoxel_clusters.end(); centroidIt++)
      centroidIt->second->getCentroidPointNormal(centroids[centroidIt->first]);
    
    // Reorient centroid normals
    pcl::search::KdTree<PointT> searchTree;
    searchTree.setInputCloud(cloud);
  
    for (size_t centroidId = 0; centroidId < centroids.size(); centroidId++)
    {
      // Find points nearest to the centroid center
      int numNeighbours = 5;
      std::vector<int> neighbours (numNeighbours);
      std::vector<float> distances (numNeighbours);
      
      pcl::PointXYZRGBNormal ctrdPt;
      ctrdPt.getVector3fMap() = centroids.at(centroidId).getVector3fMap();
      searchTree.nearestKSearch(ctrdPt, numNeighbours, neighbours, distances);
      
      int numFlip = 0;
      int numNoFlip = 0;
      
      for (size_t nbrIdIt = 0; nbrIdIt < numNeighbours; nbrIdIt++)
      {
        if (cloud->at(neighbours[nbrIdIt]).getNormalVector3fMap().dot(centroids.at(centroidId).getNormalVector3fMap()) < 0)
          numFlip++;
        else
          numNoFlip++;
        
        if (numFlip > numNoFlip)
          centroids.at(centroidId).getNormalVector3fMap() *= -1;
      }
    }    
    
    return true;
  }  
  
  /** \brief Oversegment input pointcloud
    * \param[in]  cloud               input pointcloud
    * \param[in]  voxel_resolution    spatial resolution of the voxelgrid filter applied before segmentation
    * \param[in]  seed_resolution     distance between cluster seeds
    * \param[in]  spatial_importance  weight of the Euclidean distance term used for calculating distances between points and cluster seeds
    * \param[in]  normal_importance   weight of the normal difference term used for calculating distances between points and cluster seeds
    * \param[out] clusters            a vector where each element represents a cluster and holds the indices of points in the input cloud belonging to the cluster
    * \param[out] adjacency           a vector where each element represents a cluster and holds the indices of its neighbouring clusters
    * \param[out] centroids           a vector where each element represents a cluster and holds cluster centroid location and normal
    */
  template <typename PointT>
  bool supervoxelSegmentation_old(  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                const float voxel_resolution,
                                const float seed_resolution,
                                const float spatial_importance,
                                const float normal_importance,
                                std::vector<int> &point_labels,
                                utl::Graph &adjacency,
                                pcl::PointCloud<PointNT> &centroids
                             )
  {
    // Parameters
    int numRefinementIterations = 5;
    
    // Extract normal and point data into separate clouds
    pcl::PointCloud<Point>::Ptr cloudPoints  (new pcl::PointCloud<Point>);
    pcl::PointCloud<PointN>::Ptr cloudNormals (new pcl::PointCloud<PointN>);
    pcl::copyPointCloud(*cloud, *cloudPoints);
    pcl::copyPointCloud(*cloud, *cloudNormals);

    // Setup supervoxel clustering
    pcl::SupervoxelClustering<Point> super (voxel_resolution, seed_resolution, false);
    super.setInputCloud(cloudPoints);
    super.setNormalCloud(cloudNormals);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);
    super.setColorImportance(0.0f);

    // Segment
    std::map <uint32_t, pcl::Supervoxel<Point>::Ptr >  supervoxel_clusters;      
    std::multimap<uint32_t, uint32_t>                   label_adjacency;
    super.extract (supervoxel_clusters);                                          // Get supervoxel clusters
    super.refineSupervoxels(numRefinementIterations, supervoxel_clusters);                              // Refine clusters
    super.getSupervoxelAdjacency (label_adjacency);                               // Get supervoxel adjacency
    pcl::PointCloud<PointLT>::Ptr labeledCloud = super.getLabeledCloud ();        // Get input cloud labels
    centroids = *super.makeSupervoxelNormalCloud(supervoxel_clusters);                                   
                 
    // First store the mapping from cluster ID to cluster position in the vector
    std::map<uint32_t, int> clusterIdMapping;
    std::map <uint32_t, pcl::Supervoxel<Point>::Ptr >::iterator cluster_it = supervoxel_clusters.begin();
    int i = 0;
    for ( ; cluster_it != supervoxel_clusters.end(); cluster_it++)
      clusterIdMapping.insert(std::pair<uint32_t, int>(cluster_it->first,i++));
                                               
    // Extract cluster assignment information
    point_labels.resize(labeledCloud->size());
    for (size_t i = 0; i < labeledCloud->size(); i++)
    {
      int clusterIdMapped = clusterIdMapping.find(labeledCloud->points[i].label)->second;
      point_labels[i] =clusterIdMapped;
    }                                             

    // Extract adjacency information
    adjacency.resize(supervoxel_clusters.size());
    for (std::map <uint32_t, pcl::Supervoxel<Point>::Ptr >::iterator cluster_it = supervoxel_clusters.begin(); cluster_it != supervoxel_clusters.end(); cluster_it++)
    {
      int sourceId = (cluster_it->first);
          
      std::pair<std::multimap<uint32_t, uint32_t>::iterator, std::multimap<uint32_t, uint32_t>::iterator > ppp;
      ppp = label_adjacency.equal_range(sourceId);
      for (std::multimap<uint32_t, uint32_t>::iterator adj_cluster_it = ppp.first; adj_cluster_it != ppp.second;  ++adj_cluster_it)
      {
        int targetId = (adj_cluster_it->second);
        int sourceIdMapped = clusterIdMapping.find(sourceId)->second;
        int targetIdMapped = clusterIdMapping.find(targetId)->second;
        utl::addEdge(sourceIdMapped, targetIdMapped, adjacency);
      }
    }
    
    // Reorient normals
    pcl::search::KdTree<PointT> searchTree;
    searchTree.setInputCloud(cloud);
  
    for (size_t centroidId = 0; centroidId < centroids.size(); centroidId++)
    {
      // Find points nearest to the centroid center
      int numNeighbours = 5;
      std::vector<int> neighbours (numNeighbours);
      std::vector<float> distances (numNeighbours);
      
      pcl::PointXYZRGBNormal ctrdPt;
      ctrdPt.getVector3fMap() = centroids.at(centroidId).getVector3fMap();
      searchTree.nearestKSearch(ctrdPt, numNeighbours, neighbours, distances);
      
      int numFlip = 0;
      int numNoFlip = 0;
      
      for (size_t nbrIdIt = 0; nbrIdIt < numNeighbours; nbrIdIt++)
      {
        if (cloud->at(neighbours[nbrIdIt]).getNormalVector3fMap().dot(centroids.at(centroidId).getNormalVector3fMap()) < 0)
          numFlip++;
        else
          numNoFlip++;
        
        if (numFlip > numNoFlip)
          centroids.at(centroidId).getNormalVector3fMap() *= -1;
      }
    }
    
    return true;  
  }

  /** \brief Merge supervoxels given a merge graph indicating which supervoxels should be merged together
   * \param[in]  merge_graph         a graph indicacating which supervoxels must be merged together
   * \param[in]  clusters            a vector where each element represents a cluster and holds the indices of points in the input cloud belonging to the cluster
   * \param[in]  adjacency           a vector where each element represents a cluster and holds the indices of its neighbouring clusters
   * \param[in]  centroids           a vector where each element represents a cluster and holds cluster centroid location and normal
   * \param[out]  clusters_merged    a vector where each element represents a cluster and holds the indices of points in the input cloud belonging to the cluster
   * \param[out]  adjacency_merged   a vector where each element represents a cluster and holds the indices of its neighbouring clusters
   * \param[out]  centroids_merged   a vector where each element represents a cluster and holds cluster centroid location and normal
   */
  void mergeSupervoxels(  const utl::Graph                      &merge_graph,
                          const std::vector<int>                &point_labels,
                          const utl::Graph                      &adjacency,
                          const pcl::PointCloud<PointNT>        &centroids,
                          std::vector<int>                      &point_labels_merged,
                          utl::Graph                            &adjacency_merged,
                          pcl::PointCloud<PointNT>              &centroids_merged
                        )
  {
    //////////////////////////////////////////////////////////////////////////////
    // Next find all connected components in this graph and find a mapping from 
    // original segment id to merged segment id
    
    std::vector<std::vector<int> > mergedSeg2SegMap = utl::getConnectedComponents(merge_graph);
    
    std::vector<int> seg2mergedSegMap (centroids.size());
    for (size_t CCId = 0; CCId < mergedSeg2SegMap.size(); CCId++)
    {
      for (size_t segIdIt = 0; segIdIt < mergedSeg2SegMap[CCId].size(); segIdIt++)
      {
        int segId = mergedSeg2SegMap[CCId][segIdIt];
        seg2mergedSegMap[segId] = CCId;
      }      
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Point segment labels -> point merged segment labels
    
    point_labels_merged.resize(point_labels.size());
    for (size_t pointId = 0; pointId < point_labels.size(); pointId++)
      point_labels_merged[pointId] = seg2mergedSegMap[point_labels[pointId]];
    
    //////////////////////////////////////////////////////////////////////////////
    // Adjacency -> merged adjaceny
    
    adjacency_merged.resize(mergedSeg2SegMap.size());
    
    for (size_t mergedSegId = 0; mergedSegId < mergedSeg2SegMap.size(); mergedSegId++)
    {
      for (size_t origSegIdIt = 0; origSegIdIt < mergedSeg2SegMap[mergedSegId].size(); origSegIdIt++)
      {
        int origSegId = mergedSeg2SegMap[mergedSegId][origSegIdIt];
        
        for (size_t origSegNghbrIdIt = 0; origSegNghbrIdIt < adjacency[origSegId].size(); origSegNghbrIdIt++)
        {
          int origSegNghbrId = adjacency[origSegId][origSegNghbrIdIt];
          int mergedSegNghbrId = seg2mergedSegMap[origSegNghbrId];
          
          // Don't need to add link between vertex and itself
          if (mergedSegId != mergedSegNghbrId)
            utl::addEdge(mergedSegId, mergedSegNghbrId, adjacency_merged);
        }
      }    
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Centroids -> merged centroids
    centroids_merged.resize(mergedSeg2SegMap.size());
    for (size_t mergeSegId = 0; mergeSegId < mergedSeg2SegMap.size(); mergeSegId++)
    {
      PointNT centroid;
      centroid.getVector3fMap()       = Eigen::Vector3f(0.0, 0.0, 0.0);
      centroid.getNormalVector3fMap() = Eigen::Vector3f(0.0, 0.0, 0.0);
      centroid.curvature              = 0;
      
      int numOrigSegs = mergedSeg2SegMap[mergeSegId].size();
      
      for (size_t origSegIdItr = 0; origSegIdItr < numOrigSegs; origSegIdItr++)
      {
        int origSegId = mergedSeg2SegMap[mergeSegId][origSegIdItr];
        centroid.getVector3fMap()         += centroids[origSegId].getVector3fMap();
        centroid.getNormalVector3fMap()   += centroids[origSegId].getNormalVector3fMap();
        centroid.curvature                += centroids[origSegId].curvature;
      }
      
      centroid.getVector3fMap()       /= numOrigSegs;
      centroid.getNormalVector3fMap() /= numOrigSegs;
      centroid.curvature              /= numOrigSegs;
      
      centroids_merged.points[mergeSegId] = centroid;
    }
  }

  /** \brief Merge supervoxels using the centroid normal criterion
    * \param[in]  clusters            a vector where each element represents a cluster and holds the indices of points in the input cloud belonging to the cluster
    * \param[in]  adjacency           a vector where each element represents a cluster and holds the indices of its neighbouring clusters
    * \param[in]  centroids           a vector where each element represents a cluster and holds cluster centroid location and normal
    * \param[in]  normal_difference_thresh    angular difference between normals of two supervoxels
    */
  void mergeSupervoxelsCentroidNormal(  const std::vector<int>                &point_labels,
                                        const utl::Graph                      &adjacency,
                                        const pcl::PointCloud<PointNT>        &centroids,
                                        std::vector<int>                      &point_labels_merged,
                                        utl::Graph                            &adjacency_merged,
                                        pcl::PointCloud<PointNT>              &centroids_merged,
                                        const float                           &normal_difference_thresh
                        )
  {
    // First generate a graph structure where vertices correspond to supervoxels
    // and edges correspond to merge between supervoxels
    utl::Graph mergeGraph(centroids.size());
    std::vector<std::pair<int, int> > adjacencyEdges = graph2EdgePairs(adjacency);
    
    for (auto edgeIt = adjacencyEdges.begin(); edgeIt != adjacencyEdges.end(); edgeIt++)
    {
      Eigen::Vector3f sourceNormal = centroids.points[edgeIt->first].getNormalVector3fMap();
      Eigen::Vector3f targetNormal = centroids.points[edgeIt->second].getNormalVector3fMap();      
      float normalDiffAngle   = acos(utl::clampValue(sourceNormal.dot(targetNormal), -1.0f, 1.0f));

      if (normalDiffAngle < normal_difference_thresh)
        utl::addEdge(edgeIt->first, edgeIt->second, mergeGraph);
    }

    // Then merge supervoxles given the merge graph
    mergeSupervoxels(mergeGraph, point_labels, adjacency, centroids, point_labels_merged, adjacency_merged, centroids_merged);

  }

  /** \brief Merge supervoxels greedily
    * \param[in]  clusters            a vector where each element represents a cluster and holds the indices of points in the input cloud belonging to the cluster
    * \param[in]  adjacency           a vector where each element represents a cluster and holds the indices of its neighbouring clusters
    * \param[in]  centroids           a vector where each element represents a cluster and holds cluster centroid location and normal
    * \param[in]  normal_difference_thresh    angular difference between normals of two supervoxels
    */
  void mergeSupervoxelsBoundaryCurvature( const std::vector<pcl::PointCloud<PointNT> >  &boundary_points,
                                          const std::vector<std::pair<int, int> >       &boundary_point_labels,
                                          const std::vector<int>                        &point_labels,
                                          const utl::Graph                              &adjacency,
                                          const pcl::PointCloud<PointNT>                &centroids,
                                          std::vector<int>                              &point_labels_merged,
                                          utl::Graph                                    &adjacency_merged,
                                          pcl::PointCloud<PointNT>                      &centroids_merged,
                                          const float                                   &min_average_curvature                                        
                        )
  {
    //////////////////////////////////////////////////////////////////////////////
    // First generate a graph structure where vertices correspond to supervoxels
    // and edges correspond to merge between supervoxels
    // NOTE: the criterion for merging two pathces should be improved (it should depend on the curvature at the edge)
    utl::Graph mergeGraph(centroids.size());

    for (size_t bdrId = 0; bdrId < boundary_point_labels.size(); bdrId++)
    {
      // Compute average curvature for a boundary
      float averageCurvature = 0;
      for (size_t pointId = 0; pointId < boundary_points[bdrId].size(); pointId++)
        averageCurvature += boundary_points[bdrId].points[pointId].curvature;
      averageCurvature /= boundary_points[bdrId].size();
      
      // If it is smaller than threshold - merge corresponding segments
      if (averageCurvature < min_average_curvature)
        utl::addEdge(boundary_point_labels[bdrId].first, boundary_point_labels[bdrId].second, mergeGraph);
      
    }
    
    // Then merge supervoxles given the merge graph
    mergeSupervoxels(mergeGraph, point_labels, adjacency, centroids, point_labels_merged, adjacency_merged, centroids_merged);
  }

  /** \brief Merge supervoxels greedily
    * \param[in]  clusters            a vector where each element represents a cluster and holds the indices of points in the input cloud belonging to the cluster
    * \param[in]  adjacency           a vector where each element represents a cluster and holds the indices of its neighbouring clusters
    * \param[in]  centroids           a vector where each element represents a cluster and holds cluster centroid location and normal
    * \param[in]  normal_difference_thresh    angular difference between normals of two supervoxels
    */
  void mergeSupervoxelsBoundaryCurvatureCentroidNormal (
                                          const std::vector<pcl::PointCloud<PointNT> >  &boundary_points,
                                          const std::vector<std::pair<int, int> >       &boundary_point_labels,
                                          const std::vector<int>                        &point_labels,
                                          const utl::Graph                              &adjacency,
                                          const pcl::PointCloud<PointNT>                &centroids,
                                          std::vector<int>                              &point_labels_merged,
                                          utl::Graph                                    &adjacency_merged,
                                          pcl::PointCloud<PointNT>                      &centroids_merged,
                                          const float                                   &min_average_curvature,                                        
                                          const float                                   &normal_difference_thresh                                        
                        )
  {
    //////////////////////////////////////////////////////////////////////////////
    // First generate a graph structure where vertices correspond to supervoxels
    // and edges correspond to merge between supervoxels
    // NOTE: the criterion for merging two pathces should be improved (it should depend on the curvature at the edge)
    utl::Graph mergeGraph(centroids.size());

    for (size_t bdrId = 0; bdrId < boundary_point_labels.size(); bdrId++)
    {
      int sourceSegId = boundary_point_labels[bdrId].first;
      int targetSegId = boundary_point_labels[bdrId].second;
      
      // Compute average curvature for a boundary
      float averageCurvature = 0;
      for (size_t pointId = 0; pointId < boundary_points[bdrId].size(); pointId++)
        averageCurvature += boundary_points[bdrId].points[pointId].curvature;
      averageCurvature /= boundary_points[bdrId].size();
      
      // Compute centroid normal difference between two suoervoxels
      Eigen::Vector3f sourceNormal = centroids.points[sourceSegId].getNormalVector3fMap();
      Eigen::Vector3f targetNormal = centroids.points[targetSegId].getNormalVector3fMap();      
      float normalDiffAngle   = acos(sourceNormal.dot(targetNormal));
      
      // If it is smaller than threshold - merge corresponding segments
      if ((averageCurvature < min_average_curvature) || (normalDiffAngle < normal_difference_thresh))
        utl::addEdge(sourceSegId, targetSegId, mergeGraph);
      
    }
    
    // Then merge supervoxles given the merge graph
    mergeSupervoxels(mergeGraph, point_labels, adjacency, centroids, point_labels_merged, adjacency_merged, centroids_merged);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Go through all pointcloud points. If sufficiently many neighbours have a
  // different label - switch the label of the current point.
  template <typename PointT>
  int cleanupSegmentation(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::vector<int> &point_svoxels, std::vector<int> &point_svoxels_cleaned)
  {
    // Parameters
    int numNeighbours = 8;
    int thresholdRatio = 5;
    
    // Prepare search tree
    pcl::search::KdTree<PointT> searchTree;
    searchTree.setInputCloud(cloud);
    
    int labelsChanged = 0;
    point_svoxels_cleaned = point_svoxels;
    
    // Loop over all points
    for (size_t pointId = 0; pointId < point_svoxels.size(); pointId++)
    {    
      // Find nearest neighbours
      std::vector<float>  distances(numNeighbours);
      std::vector<int>    neighbors(numNeighbours);
      searchTree.nearestKSearch(pointId, numNeighbours, neighbors, distances);
      
      // Get the labels of neighbours
      std::vector<int> labels;
      for (size_t i = 1; i < numNeighbours; i++)
        labels.push_back(point_svoxels[neighbors[i]]);
      
      // Get counts for each label
      std::vector<std::pair<int, int> > labelHistogram = utl::vectorCount<int>(labels);
      
      // Assign the value of the most frequent label
      int maxLabel  = -1;
      int maxFreq   = -1;
      int origFreq  = -1;
      for (size_t i = 0; i < labelHistogram.size(); i++)
      {
        if (labelHistogram[i].first == point_svoxels[pointId])
          origFreq = labelHistogram[i].second;
        
        if (labelHistogram[i].second > maxFreq)
        {
          maxLabel  = labelHistogram[i].first;
          maxFreq   = labelHistogram[i].second;
        }
      }
      
      if ((maxLabel != point_svoxels[pointId]) && (origFreq <  (numNeighbours - thresholdRatio)))
      {
        point_svoxels_cleaned[pointId] = maxLabel;
        labelsChanged++;
      }
    }
    
    return labelsChanged;
  }

  /** \brief Generate a mesh for an input pointcloud
    * \param[in]  cloud               input pointcloud
    * \param[out] triangles           output mesh
    */
  template <typename PointT>
  void meshSupervoxel (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, pcl::PolygonMesh &triangles)
  {
    // Set verbosity level to ERROR only
    pcl::console::VERBOSITY_LEVEL curVerbosityLevel = pcl::console::getVerbosityLevel();
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    
    // Create search tree*
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
    
    // Use Greedy Projection Triangulation
    pcl::GreedyProjectionTriangulation<PointT> gpt;
    gpt.setSearchRadius(0.05);                           // Maximum distance between connected points
    gpt.setMu(2.0);                                      // Radius multiplier ???
    gpt.setMaximumNearestNeighbors(20);                   // Maximum number of neighbours
    gpt.setMaximumSurfaceAngle(pcl::deg2rad(45.0));      // Maximum normal angle difference between connected points
    gpt.setMinimumAngle(pcl::deg2rad(10.0));            // Minimum angle of each surface trinagle
    gpt.setMaximumAngle(pcl::deg2rad(120.0));           // Maximim angle of each triangle
    gpt.setNormalConsistency(true);                     // Set if the normals in the input cloud are consistent
    
    gpt.setInputCloud(cloud);
    gpt.setSearchMethod(tree);
    gpt.reconstruct(triangles);
    
    // Revert verbosity level
    pcl::console::setVerbosityLevel(curVerbosityLevel);
  }
}

#endif // SUPERVOXEL_SEG_HPP