#ifndef PCL_QUICK_VIS_HPP_
#define PCL_QUICK_VIS_HPP_

// Includes
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <sstream>

// VTK
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkScalarBarActor.h>
#include <vtkActor2DCollection.h>

// My includes
#include <cpp_utilities.hpp>
#include <adjacency_list.hpp>
#include <vtk_colormaps.hpp>

// PCL
#include <pcl/common/angles.h>

namespace pclvis = pcl::visualization;
typedef pclvis::PCLVisualizer PclVis;


namespace pcl
{ 
  /** \brief Create a new visualizer
   */
  inline
  pcl::visualization::PCLVisualizer createVisualizer()
  {
    pcl::visualization::PCLVisualizer visualizer;
    visualizer.setBackgroundColor (0, 0.0, 0.15);
    visualizer.initCameraParameters ();
    return visualizer;
  }
  
  /** \brief visualize correspondences between two clouds
   *  \param[in] visualizer pointer to the visualizer object
   *  \param[in] target_cloud target cloud (match)
   *  \param[in] source_cloud source cloud (query)
   *  \param[in] correspondences correspondences from source to target
   *  \param[in] num_corresp_displayed number of correspondences that should be displayed
   *  \param[in] sourceVisRt transformation that is applied to the source cloud before visualization
   *  \param[in] id a string identifier for the correspondence object
   */
//   template <typename PointT>
//   inline
//   void showCorrespondences(void *visualizer,
//                            const pcl::PointCloud<PointT> &target_cloud,
//                            const pcl::PointCloud<PointT> &source_cloud,
//                            const pcl::Correspondences &correspondences,
//                            size_t num_corresp_displayed,
//                            Eigen::Affine3f &sourceVisRt = Eigen::Affine3f::Identity(),
//                            const std::string &id = "correspondences")
//   {
//     
//     pcl::visualization::PCLVisualizer* visualizer_ = reinterpret_cast<pcl::visualization::PCLVisualizer*> (visualizer);
// 
//     // Figure out how step size
//     size_t stepSize;
//     if (correspondences.size() < num_corresp_displayed)
//     {
//       num_corresp_displayed = correspondences.size();
//       stepSize = 1;
//     }
//     else
//     {
//       stepSize = correspondences.size() / num_corresp_displayed;
//     }
//     
//     // First shift source cloudg
//     typename pcl::PointCloud<PointT> sourceCloudShifted;
//     pcl::transformPointCloudWithNormals<PointT>(source_cloud, sourceCloudShifted, sourceVisRt);
// 
//     // Select the required number of correspondences from the existing correspondneces
//     typename pcl::PointCloud<PointT>::Ptr displayCloud(new pcl::PointCloud<PointT>);
//     typename pcl::PointCloud<PointT> sourceCorrespondenceCloud;
//     typename pcl::PointCloud<PointT> targetCorrespondenceCloud;
//     pcl::Correspondences correspondencesDisplayed;
//     sourceCorrespondenceCloud.resize(num_corresp_displayed);
//     targetCorrespondenceCloud.resize(num_corresp_displayed);
//     correspondencesDisplayed.resize(num_corresp_displayed);
//     
//     for (size_t i = 0; i < num_corresp_displayed; i++)
//     {
//       size_t queryIndex = correspondences.at(i*stepSize).index_query;
//       size_t matchIndex = correspondences.at(i*stepSize).index_match;
//       
//       sourceCorrespondenceCloud.at(i) = sourceCloudShifted.at(queryIndex);
//       targetCorrespondenceCloud.at(i) = target_cloud.at(matchIndex);
//       correspondencesDisplayed.at(i).index_query = i;
//       correspondencesDisplayed.at(i).index_match = i;    
//     }
//     
//     // Display pointclouds
//     *displayCloud = sourceCloudShifted + target_cloud;
//     if (pcl::getFieldsList(*displayCloud).find("rgb") == std::string::npos)
//     {
//       if (!visualizer_->updatePointCloud<PointT> (displayCloud, "cloud"))
//       {
//         visualizer_->addPointCloud<PointT> (displayCloud, "cloud");
//       }
//     }
//     else
//     {
//       pcl::visualization::PointCloudColorHandlerRGBField<PointT> color_handler (displayCloud);
//       if (!visualizer_->updatePointCloud<PointT> (displayCloud, color_handler, "cloud"))
//       {
//         visualizer_->addPointCloud<PointT> (displayCloud, color_handler, "cloud");
//         visualizer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
//       }
//     }
// 
//     // Add correspondences
//     pcl::PointCloud<pcl::PointXYZ>::Ptr defaultFrame (new pcl::PointCloud<pcl::PointXYZ>);
//     float frameSize = 0.01;
//     defaultFrame->push_back(pcl::PointXYZ( frameSize,  -frameSize,  0.0));
//     defaultFrame->push_back(pcl::PointXYZ( frameSize,   frameSize,  0.0));
//     defaultFrame->push_back(pcl::PointXYZ(-frameSize,   frameSize,  0.0));
//     defaultFrame->push_back(pcl::PointXYZ(-frameSize,  -frameSize,  0.0));
//     
//     Eigen::Vector3f frameNormal (0.0, 0.0, 1.0);  
//     Eigen::Vector3f cameraCenter = visualizer_->getViewerPose().translation();
//     Eigen::Vector3f cameraNormal = visualizer_->getViewerPose().rotation().col(2);
//     Eigen::Vector3f cameraUp     = visualizer_->getViewerPose().rotation().col(1);
//     std::vector<std::vector<float> > colours = colourPalette(num_corresp_displayed);
//     
//     for (size_t i = 0; i < num_corresp_displayed; i++)
//     {
//       // Find all parameters
//       Eigen::Vector3f sourceCenter = sourceCorrespondenceCloud.points[i].getVector3fMap();
//       Eigen::Vector3f sourceCameraNormal = (cameraCenter-sourceCenter);
//       sourceCameraNormal.normalize();
//       sourceCenter+= 0.02 * sourceCameraNormal;
//       pcl::PointXYZ sourceCenterPoint (sourceCenter[0], sourceCenter[1], sourceCenter[2]);
//       
//       Eigen::Vector3f targetCenter = targetCorrespondenceCloud.points[i].getVector3fMap();
//       Eigen::Vector3f targetCameraNormal = (cameraCenter-targetCenter);
//       targetCameraNormal.normalize();
//       targetCenter+= 0.02 * targetCameraNormal;
//       pcl::PointXYZ targetCenterPoint (targetCenter[0], targetCenter[1], targetCenter[2]);    
// 
//       // Draw correspondences
//       std::stringstream correspondenceStr;
//       correspondenceStr << "correspondece_" << i;
//       visualizer_->addLine(sourceCenterPoint, targetCenterPoint, correspondenceStr.str());
//       visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colours[i][0], colours[i][1], colours[i][2], correspondenceStr.str());
//       visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, correspondenceStr.str());
//       
//       // Draw source frame
//       Eigen::Matrix3f R_sourceCameraCenter = Eigen::alignNormals<float>(frameNormal, sourceCameraNormal);                           // make the frame point at the camera
//       Eigen::Vector3f sourceUpProjected = cameraUp - sourceCameraNormal * (sourceCameraNormal.dot(cameraUp));
//       sourceUpProjected.normalize();
//       Eigen::Matrix3f R_sourceCameraUp = Eigen::alignNormals<float>(R_sourceCameraCenter.col(0), sourceUpProjected);                      // aligns the up direction of the frame with the up direction of the camera
// 
//       Eigen::Affine3f sourceFrameRt;                        // Combine transformations
//       sourceFrameRt.translation() = sourceCenter;
//       sourceFrameRt.linear() = R_sourceCameraUp * R_sourceCameraCenter;
//           
//       pcl::PointCloud<pcl::PointXYZ>::Ptr sourceFrame (new pcl::PointCloud<pcl::PointXYZ>);   // Transform pointcloud
//       pcl::transformPointCloud<pcl::PointXYZ>(*defaultFrame, *sourceFrame, sourceFrameRt);
//       
//       std::stringstream sourceFrameStr;                   // Display
//       for (size_t j = 0 ; j < sourceFrame->size(); j++)
//       {
//         sourceFrameStr.str() = "";
//         sourceFrameStr << "source_frame_" << i << "_line_" << j;
//         visualizer_->addLine(sourceFrame->points[j], sourceFrame->points[(j+1) % sourceFrame->size()], sourceFrameStr.str());
//         visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colours[i][0], colours[i][1], colours[i][2], sourceFrameStr.str());
//         visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, sourceFrameStr.str());      
//       }
//       
//       // Draw target frame
//       Eigen::Matrix3f R_targetCameraCenter = Eigen::alignNormals<float>(frameNormal, targetCameraNormal);                           // make the frame point at the camera
//       Eigen::Vector3f targetUpProjected = cameraUp - targetCameraNormal * (targetCameraNormal.dot(cameraUp));
//       targetUpProjected.normalize();
//       Eigen::Matrix3f R_targetCameraUp = Eigen::alignNormals<float>(R_targetCameraCenter.col(0), targetUpProjected);                      // aligns the up direction of the frame with the up direction of the camera
// 
//       Eigen::Affine3f targetFrameRt;                        // Combine transformations
//       targetFrameRt.translation() = targetCenter;
//       targetFrameRt.linear() = R_targetCameraUp * R_targetCameraCenter;
//       
//       pcl::PointCloud<pcl::PointXYZ>::Ptr targetFrame (new pcl::PointCloud<pcl::PointXYZ>);
//       pcl::transformPointCloud<pcl::PointXYZ>(*defaultFrame, *targetFrame, targetFrameRt);
//       
//       std::stringstream targetFrameStr;
//       for (size_t j = 0 ; j < targetFrame->size(); j++)
//       {
//         targetFrameStr.str() = "";
//         targetFrameStr << "target_frame_" << i << "_line_" << j;
//         visualizer_->addLine(targetFrame->points[j], targetFrame->points[(j+1) % targetFrame->size()], targetFrameStr.str());
//         visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colours[i][0], colours[i][1], colours[i][2], targetFrameStr.str());
//         visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, targetFrameStr.str());      
//       }
//     }  
//   }
  
  /** \brief Generate a set of polygons forming a cube with side_length centered at pose
   *  \param[out] polygons vector contatining polygons
   *  \param[in] pose pose of the center of the cube
   *  \param[in] side_length side length of the cube
   */
  inline
  void generateCube (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &polygons, const Eigen::Affine3f &pose, float side_length)
  {
    float h = side_length / 2;
    polygons.resize(0);
    
    // Generate cube at (0, 0, 0)
    pcl::PointCloud<pcl::PointXYZ>::Ptr face1 (new pcl::PointCloud<pcl::PointXYZ>);
    face1->push_back(pcl::PointXYZ(h/2, -h/2, -h/2));
    face1->push_back(pcl::PointXYZ(h/2, -h/2,  h/2));
    face1->push_back(pcl::PointXYZ(h/2,  h/2,  h/2));
    face1->push_back(pcl::PointXYZ(h/2,  h/2, -h/2));
    polygons.push_back(face1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr face2 (new pcl::PointCloud<pcl::PointXYZ>);
    face2->push_back(pcl::PointXYZ(-h/2, -h/2, -h/2));
    face2->push_back(pcl::PointXYZ(-h/2, -h/2,  h/2));
    face2->push_back(pcl::PointXYZ(-h/2,  h/2,  h/2));
    face2->push_back(pcl::PointXYZ(-h/2,  h/2, -h/2));
    polygons.push_back(face2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr face3 (new pcl::PointCloud<pcl::PointXYZ>);
    face3->push_back(pcl::PointXYZ( h/2,  h/2,  h/2));
    face3->push_back(pcl::PointXYZ( h/2,  h/2, -h/2));
    face3->push_back(pcl::PointXYZ(-h/2,  h/2, -h/2));
    face3->push_back(pcl::PointXYZ(-h/2,  h/2,  h/2));
    polygons.push_back(face3);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr face4 (new pcl::PointCloud<pcl::PointXYZ>);
    face4->push_back(pcl::PointXYZ( h/2, -h/2,  h/2));
    face4->push_back(pcl::PointXYZ( h/2, -h/2, -h/2));
    face4->push_back(pcl::PointXYZ(-h/2, -h/2, -h/2));
    face4->push_back(pcl::PointXYZ(-h/2, -h/2,  h/2));
    polygons.push_back(face4);  
    
    // Transform to pose
    for (size_t i = 0; i < polygons.size(); i++)
      pcl::transformPointCloud(*(polygons[i]), *(polygons[i]), pose);
  }  
  
  /** \brief Display a cube with side_length centered at pose
   *  \param[in] pose pose of the center of the cube
   *  \param[in] side_length side length of the cube
   * \param[in] visualizer object
   */
  inline
  void showCube(const Eigen::Affine3f &pose, float side_length, pcl::visualization::PCLVisualizer &visualizer)
  {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cubePolys;
    generateCube(cubePolys, pose, side_length);
    
    for (size_t i = 0; i < cubePolys.size(); i++)
    {
      std::ostringstream convert;
      convert << i;
      std::string faceName = "face_" + convert.str();
      visualizer.addPolygon<pcl::PointXYZ>(cubePolys[i], 1.0, 1.0, 1.0, faceName);
    }
  }
  
  //----------------------------------------------------------------------------
  // Oversegmentation visualization
  //----------------------------------------------------------------------------  
  
  /** \brief visualize a segmentation of the pointcloud
   *  \param[in] visualizer visualizer object
   *  \param[in] cloud input pointcloud
   *  \param[in] segments assignment of point to segments
   *  \param[in] id the point cloud object id (default: cloud_seg)
   *  \param[in] point_size size of the points used for visualization
   */
  template <typename PointT>
  inline
  void showSegmentation ( PclVis &visualizer,
                          const pcl::PointCloud<PointT> &cloud,
                          const std::vector<int> point_labels,
                          const std::string &id = "cloud_seg",
                          const float &point_size = 1.0
                        )
  {
    // Check input
    if (cloud.size() != point_labels.size())
    {
      std::cout << "[pcl::showSegmentation] number of points in the cloud and number of labels must be the same\n";
      return;
    }
    
    // Copy point coordinates
    pcl::PointCloud<pcl::PointXYZL>::Ptr displayCloud (new pcl::PointCloud<pcl::PointXYZL>);
    pcl::copyPointCloud(cloud, *displayCloud);

    // Assign labels
    for (size_t pointId = 0; pointId < cloud.size(); pointId++)
      displayCloud->points[pointId].label = point_labels[pointId];
          
    // Display
    pcl::visualization::PointCloudColorHandlerLabelField<pcl::PointXYZL> color_handler (displayCloud);
    visualizer.addPointCloud(displayCloud, id);        
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
  }

  /** \brief visualize a segmentation of the pointcloud
   *  \param[in] visualizer visualizer object
   *  \param[in] segments a vector of clouds where each cloud corresponds to a segment
   *  \param[in] id_prefix the point cloud object id prefix (default: segment_)
   *  \param[in] point_size size of the points used for visualization
   */
  template <typename PointT>
  inline
  void showSegmentation ( PclVis &visualizer,
                          const std::vector<pcl::PointCloud<PointT> > &segments,
                          const std::string &id_prefix = "segment_",
                          const float &point_size = 1.0)
  {
    for (size_t segId = 0; segId < segments.size(); segId++)
    {
      pcl::RGB colour = pcl::getGlasbeyColor (segId % pcl::getGlasbeyLUTSize ());
      std::string id = id_prefix + "_" + std::to_string(segId);
      visualizer.addPointCloud<PointT>(segments[segId].makeShared(), id);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
                                                   static_cast<double>(colour.r)/255,
                                                   static_cast<double>(colour.g)/255,
                                                   static_cast<double>(colour.b)/255, id);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
    }
  }

  /** \brief visualize a segmentation of the pointcloud
   *  \param[in] visualizer visualizer object
   *  \param[in] cloud input pointcloud
   *  \param[in] segments a vector of vector of indices each representing a segment
   *  \param[in] id_prefix the point cloud object id prefix (default: segment_)
   *  \param[in] point_size size of the points used for visualization
   */
  template <typename PointT>
  inline
  void showSegmentation ( PclVis &visualizer,
                          const pcl::PointCloud<PointT> &cloud,                          
                          const std::vector<std::vector<int> > &segments,
                          const std::string &id_prefix = "segment_",
                          const float &point_size = 1.0)
  {
    typename pcl::PointCloud<PointT>::Ptr displayCloud (new pcl::PointCloud<PointT>);
    
    for (size_t segId = 0; segId < segments.size(); segId++)
    {
      // Get segment cloud
      pcl::copyPointCloud(cloud, segments[segId], *displayCloud);
      
      pcl::RGB colour = pcl::getGlasbeyColor (segId % pcl::getGlasbeyLUTSize ());
      std::string id = id_prefix + "_" + std::to_string(segId);
      visualizer.addPointCloud<PointT>(displayCloud, id);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
                                                   static_cast<double>(colour.r)/255,
                                                   static_cast<double>(colour.g)/255,
                                                   static_cast<double>(colour.b)/255, id);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
    }
  }
  
  //----------------------------------------------------------------------------
  // Foreground segmentation visualization
  //----------------------------------------------------------------------------
  
  inline
  void showFGsegmentationMesh (PclVis &visualizer, const std::vector<pcl::PolygonMesh> &mesh, std::vector<int> &segmentation)
  { 
    // Get segmentation mask
    std::vector<bool> segmentationMask (mesh.size(), false);
    for (size_t segId = 0; segId < segmentation.size(); segId++)
      segmentationMask[segmentation[segId]] = true;
    
    std::vector<double> FGcolour = {1.0, 1.0, 1.0};
    std::vector<double> BGcolour = {0.2, 0.2, 0.2};
    
    for (size_t segId = 0; segId < mesh.size(); segId++)
    {
      std::string segIdStr = "segment_" + std::to_string(segId);
      visualizer.addPolygonMesh(mesh[segId], segIdStr);
      if (segmentationMask[segId])
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, FGcolour[0], FGcolour[1], FGcolour[2], segIdStr);
      else
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, BGcolour[0], BGcolour[1], BGcolour[2], segIdStr);
    }                  
  }  

  /** \brief visualize a a freground background segmentation of a pointcloud
   *  \param[in] visualizer visualizer object
   *  \param[in] cloud input pointcloud
   *  \param[in] fg_indices indices of the foreground points
   *  \param[in] point_size size of the points used for visualization
   */  
  template <typename PointT>
  inline
  void showFGsegmentation (PclVis &visualizer, const pcl::PointCloud<PointT> &cloud, std::vector<int> &fg_indices, const float point_size = 1.0f)
  { 
    // Get background points
    std::vector<int> all_indices (cloud.size());
    for (size_t pointId = 0; pointId < cloud.size(); pointId++)
      all_indices[pointId] = pointId;    
    std::vector<int> bg_indices = utl::vectorDifference(all_indices, fg_indices);
    
    // Get FG and BG clouds
    typename pcl::PointCloud<PointT>::Ptr fgCloud (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr bgCloud (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(cloud, fg_indices, *fgCloud);
    pcl::copyPointCloud(cloud, bg_indices, *bgCloud);
    
    // Display
    std::vector<double> FGcolour = {1.0, 1.0, 1.0};
    std::vector<double> BGcolour = {0.2, 0.2, 0.2};

    visualizer.addPointCloud<PointT>(fgCloud, "foreground");
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, FGcolour[0], FGcolour[1], FGcolour[2], "foreground");
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "foreground");
    
    visualizer.addPointCloud<PointT>(bgCloud, "backgound");
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, BGcolour[0], BGcolour[1], BGcolour[2], "backgound");
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "backgound");
  }
  
  /** \brief visualize a a freground background segmentation of a pointcloud
   *  \param[in] visualizer visualizer object
   *  \param[in] cloud input pointcloud
   *  \param[in] fg_indices indices of the foreground points
   *  \param[in] point_size size of the points used for visualization
   */  
  template <typename PointT>
  inline
  void showFGsegmentationColour (PclVis &visualizer, const pcl::PointCloud<PointT> &cloud, std::vector<int> &fg_indices, const float point_size = 1.0f)
  { 
    // Get background points
    std::vector<int> all_indices (cloud.size());
    for (size_t pointId = 0; pointId < cloud.size(); pointId++)
      all_indices[pointId] = pointId;    
    std::vector<int> bg_indices = utl::vectorDifference(all_indices, fg_indices);
    
    // Get FG and BG clouds
    typename pcl::PointCloud<PointT>::Ptr displayCloud (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(cloud, *displayCloud);
    
    // Display
    std::vector<double> BGcolour = {220, 220, 220};
    for (auto pointIt = bg_indices.begin(); pointIt != bg_indices.end(); pointIt++)
    {
      displayCloud->points[*pointIt].r = BGcolour[0];
      displayCloud->points[*pointIt].g = BGcolour[1];
      displayCloud->points[*pointIt].b = BGcolour[2];
    }
    
//     for (auto pointIt = fg_indices.begin(); pointIt != fg_indices.end(); pointIt++)
//     {
//       displayCloud->points[*pointIt].r = utl::clampValue(displayCloud->points[*pointIt].r + 50, 0, 255);
//       displayCloud->points[*pointIt].g = utl::clampValue(displayCloud->points[*pointIt].g + 50, 0, 255);
//       displayCloud->points[*pointIt].b = utl::clampValue(displayCloud->points[*pointIt].b + 50, 0, 255);
//     }
    
    pclvis::PointCloudColorHandlerRGBField<PointT> color_handler(displayCloud);
    visualizer.addPointCloud<PointT>(displayCloud, color_handler, "fg_segmentation");
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "fg_segmentation");
  }  
  
  //----------------------------------------------------------------------------
  // Mesh visualization
  //----------------------------------------------------------------------------    
  
  /** \brief visualize a set of meshes
   *  \param[in] visualizer visualizer object
   *  \param[in] meshes vector of meshes
   *  \param[in] id_prefix the point cloud object id prefix (default: mesh_)
   *  \param[in] colour colour of the meshes [0, 1]
   */  
  inline
  void showMeshes(  PclVis &visualizer,
                    const std::vector<pcl::PolygonMesh> &meshes,
                    const std::string id_prefix = "mesh_",
                    const std::vector<double> &colour = {1.0, 1.0, 1.0}
                 )
  {     
    for (size_t meshId = 0; meshId < meshes.size(); meshId++)
    {
      std::string meshIdStr = "segment_" + std::to_string(meshId);
      visualizer.addPolygonMesh(meshes[meshId], meshIdStr);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colour[0], colour[1], colour[2], meshIdStr);
    }                  
  }
  
  /** \brief visualize a set of meshes coloured according to the scalar data vector
   *  \param[in] visualizer visualizer object
   *  \param[in] meshes vector of meshes
   *  \param[in] data data vector
   *  \param[in,out]  colormap  colormap object (if range limits are equal to NaN they are updated to min max of the data vector)
   *  \param[in] id_prefix the point cloud object id prefix (default: mesh_)
   */  
  template <typename Scalar>
  inline
  void showMeshesWithData(  PclVis &visualizer,
                            const std::vector<pcl::PolygonMesh> &meshes,
                            const std::vector<Scalar> &data,
                            utl::Colormap &colormap,
                            const std::string id_prefix = "mesh_"
                        )
  {     
    utl::Colours colours = colormap.getColoursFromData(data);
    
    for (size_t meshId = 0; meshId < meshes.size(); meshId++)
    {
      std::string meshIdStr = "segment_" + std::to_string(meshId);
      visualizer.addPolygonMesh(meshes[meshId], meshIdStr);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colours[meshId][0], colours[meshId][1], colours[meshId][2], meshIdStr);
    }                  
  }    
  
  //----------------------------------------------------------------------------
  // Graph visualization
  //----------------------------------------------------------------------------

  /** \brief visualize a graph defined on points in 3D space
   *  \param[in] visualizer visualizer object
   *  \param[in] points     points
   *  \param[in] edges      edges in the graph
   *  \param[in] id_prefix  prefix to be used for line objects (default: adj_line_)
   *  \param[in] line_width width of the lines used for display (default 1.0)
   */  
  template <typename PointT>
  inline
  void showPointGraph ( pcl::visualization::PCLVisualizer &visualizer,
                        const pcl::PointCloud<PointT> &points,
                        const std::vector<std::pair<int, int> > &edges,
                        const std::string &id_prefix = "adj_line_",
                        const double line_width = 1.0
                     )
  {
    for (size_t edgeId = 0; edgeId < edges.size(); edgeId++)
    {
      int sourceVtxId = edges[edgeId].first;
      int targetVtxId = edges[edgeId].second;
      std::string id_string = id_prefix + std::to_string(edgeId) + "_"  + std::to_string(edgeId);
      visualizer.addLine(points.points[sourceVtxId], points.points[targetVtxId], id_string);
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, id_string);
    }
  }
  
  /** \brief visualize a graph defined on points in 3D space
   *  \param[in] visualizer visualizer object
   *  \param[in] points     points
   *  \param[in] graph      graph adjacency list
   *  \param[in] id_prefix  prefix to be used for line objects (default: adj_line_)
   *  \param[in] line_width width of the lines used for display (default 1.0)
   */  
  template <typename PointT>
  inline
  void showPointGraph ( pcl::visualization::PCLVisualizer &visualizer,
                        const pcl::PointCloud<PointT> &points,
                        const utl::Graph &graph,
                        const std::string &id_prefix = "adj_line_",
                        const double line_width = 1.0
                     )
  {
    // Get graph edges and their weights
    std::vector<std::pair<int, int> > edges;
    edges = utl::graph2EdgePairs(graph);
    showPointGraph<PointT>(visualizer, points, edges, id_prefix, line_width);
  }  
  
  /** \brief visualize a weighted graph defined on points in 3D space
   *  \param[in] visualizer visualizer object
   *  \param[in] points points
   *  \param[in] edges graph edges
   *  \param[in] edge_weights edge weights
   *  \param[in] colormap colormap used to visualize weights
   *  \param[in] id_prefix prefix to be used for line objects (default: adj_line_)
   *  \param[in] line_width width of the lines used for display (default 1.0)
   */  
  template <typename PointT>
  inline
  void showPointGraphWeighted ( pcl::visualization::PCLVisualizer &visualizer,
                                const pcl::PointCloud<PointT> &points,
                                const std::vector<std::pair<int, int> > &edges,
                                const std::vector<float> &edge_weights,
                                utl::Colormap &colormap,
                                const std::string &id_prefix = "edge_",
                                const double line_width = 1.0
                     )
  {
    // Get colours
    utl::Colours colours = colormap.getColoursFromData<float>(edge_weights);

    // Display graphs
    for (size_t edgeId = 0; edgeId < edges.size(); edgeId++)
    {
      int sourceVtxId = edges[edgeId].first;
      int targetVtxId = edges[edgeId].second;
      std::string id_string = id_prefix + std::to_string(edgeId) + "_"  + std::to_string(edgeId);
      visualizer.addLine(points.points[sourceVtxId], points.points[targetVtxId], id_string);
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, id_string);
      visualizer.setShapeRenderingProperties(pclvis::PCL_VISUALIZER_COLOR, colours[edgeId][0], colours[edgeId][1], colours[edgeId][2], id_string);
    }
  }
  
  /** \brief visualize a weighted graph defined on points in 3D space
   *  \param[in] visualizer visualizer object
   *  \param[in] points points
   *  \param[in] graph adjacency between points
   *  \param[in] graph_weights graph weights
   *  \param[in] id_prefix prefix to be used for line objects (default: adj_line_)
   *  \param[in] line_width width of the lines used for display (default 1.0)
   */  
  template <typename PointT>
  inline
  void showPointGraphWeighted ( pcl::visualization::PCLVisualizer &visualizer,
                                const pcl::PointCloud<PointT> &points,
                                const utl::Graph &graph,
                                const utl::GraphWeights &graph_weights,
                                utl::Colormap &colormap,
                                const std::string &id_prefix = "edge_",
                                const double line_width = 1.0
                     )
  {
    // Get graph edges and their weights
    std::vector<std::pair<int, int> > edges;
    std::vector<float> edgeWeights;
    utl::graphWeighted2EdgePairs(graph, graph_weights, edges, edgeWeights);
    
    // Visualize
    showPointGraphWeighted<PointT>(visualizer, points, edges, edgeWeights, colormap, id_prefix, line_width);
  }  
  
  /** \brief visualize a 3d curve represented as an ordered set of points
   *  \param[in]  visualizer  visualizer object
   *  \param[in]  curve       curve
   *  \param[in]  id_prefix   prefix to be used for line objects (default: curve_)
   *  \param[in]  line_width  width of the lines used for display (default 1.0)
   *  \param[in]  colour      colour of the lines (default gray)
   */  
  template <typename PointT>
  inline
  void showCurve (pcl::visualization::PCLVisualizer &visualizer,
                  const pcl::PointCloud<PointT> &curve,
                  const std::string &id_prefix = "curve_",
                  const double line_width = 1.0,
                  pcl::RGB colour = pcl::RGB()
                )
  {
    // Default colour is gray
    if (colour.r == 0 && colour.g == 0 && colour.b == 0)
      colour.r = colour.g = colour.b = 100;
    
    for (size_t linkId = 0; linkId < curve.size()-1; linkId++)
    {
      std::string id = id_prefix + "_" + std::to_string(linkId);
      visualizer.addLine(curve[linkId], curve[linkId+1], id);
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, id);
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
                                              static_cast<double>(colour.r)/255,
                                              static_cast<double>(colour.g)/255,
                                              static_cast<double>(colour.b)/255, id);

    }
  }

  //----------------------------------------------------------------------------
  // Pointcloud with data visualization
  //----------------------------------------------------------------------------  
  
  /** \brief visualize a pointcloud coloured according to the scalar data vector
   *  \param[in]  visualizer    visualizer object
   *  \param[in]  cloud         pointcloud
   *  \param[in]  data          data vector
   *  \param[in,out]  colormap  colormap object (if range limits are equal to NaN they are updated to min max of the data vector)
   *  \param[in]  id            the point cloud object id (default: cloud_seg)
   *  \param[in]  point_size    size of the point (default 1.0)
   */    
  template <typename PointT, typename Scalar>
  inline
  void showPointCloudWithData ( pcl::visualization::PCLVisualizer &visualizer,
                                const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                const Eigen::Matrix<Scalar, 1, Eigen::Dynamic> &data,
                                utl::Colormap &colormap,
                                const std::string &id = "cloud",
                                const float &point_size = 1.0
                              )
  {
    utl::Colours colours = colormap.getColoursFromData<Scalar>(data);
    pclvis::PointCloudColorHandlerCustomIndividual<PointT> color_handler (cloud, colours);

    visualizer.addPointCloud<PointT> (cloud, color_handler, id);
    visualizer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
  }
  
  /** \brief visualize a pointcloud coloured according to the scalar data vector
   *  \param[in]  visualizer    visualizer object
   *  \param[in]  cloud         pointcloud
   *  \param[in]  data          data vector
   *  \param[in,out]  colormap  colormap object (if range limits are equal to NaN they are updated to min max of the data vector)
   *  \param[in]  id            the point cloud object id (default: cloud_seg)
   *  \param[in]  point_size    size of the point (default 1.0)
   */    
  template <typename PointT, typename Scalar>
  inline
  void showPointCloudWithData ( pcl::visualization::PCLVisualizer &visualizer,
                                const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                const std::vector<Scalar> &data,
                                utl::Colormap &colormap,
                                const std::string &id = "cloud",
                                const float &point_size = 1.0
                              )
  {
    utl::Colours colours = colormap.getColoursFromData<Scalar>(data);
    pclvis::PointCloudColorHandlerCustomIndividual<PointT> color_handler (cloud, colours);

    visualizer.addPointCloud<PointT> (cloud, color_handler, id);
    visualizer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
  }  

  //----------------------------------------------------------------------------
  // Geometric primitives
  //----------------------------------------------------------------------------  
  
  /** \brief visualize a plane by plotting a square polygon 
   *  \param[in]  visualizer      visualizer object
   *  \param[in]  plane_point     plane point
   *  \param[in]  plane_normal    plane normal
    * \param[in]  id              plane object id (default: plane)
   *  \param[in]  side_width      width of the square side
   */    
  inline
  void showPlane  ( pcl::visualization::PCLVisualizer &visualizer,
                    const Eigen::Vector3f &plane_point,
                    const Eigen::Vector3f &plane_normal,
                    const std::string &id = "plane",
                    const float &side_width = 0.05
                    )
  {
    // First generate two vectors orthogonal to the normal
    Eigen::Vector3f normalOrth1 = Eigen::Vector3f(-plane_normal[1]/plane_normal[0], 1, 0);
    normalOrth1.normalize();
    Eigen::Vector3f normalOrth2 = normalOrth1.cross(plane_normal);

    // Then generate symmetry plane polygon
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_polygon (new pcl::PointCloud<pcl::PointXYZ>);
    plane_polygon->resize(4);
    plane_polygon->at(0).getVector3fMap() = plane_point + (  normalOrth1 + normalOrth2) * side_width/2;
    plane_polygon->at(1).getVector3fMap() = plane_point + (  normalOrth1 - normalOrth2) * side_width/2;
    plane_polygon->at(2).getVector3fMap() = plane_point + (- normalOrth1 - normalOrth2) * side_width/2;
    plane_polygon->at(3).getVector3fMap() = plane_point + (- normalOrth1 + normalOrth2) * side_width/2;
   
    visualizer.addPolygon<pcl::PointXYZ>(plane_polygon, 0.0, 1.0, 0.0, id);
    visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, id);
    
    // DEBUG: show symmetry axes
//     pcl::PointXYZ lineStart, normalLineEnd, orth1LineEnd, orth2LineEnd;
//     lineStart.getVector3fMap() = plane_point;
//     normalLineEnd.getVector3fMap() = plane_point + plane_normal;
//     orth1LineEnd.getVector3fMap()  = plane_point + normalOrth1;
//     orth2LineEnd.getVector3fMap()  = plane_point + normalOrth2;
//     
//     visualizer.addLine(lineStart, normalLineEnd, "symmetry_plane_vector_1");
//     visualizer.addLine(lineStart, orth1LineEnd, "symmetry_plane_vector_2");
//     visualizer.addLine(lineStart, orth2LineEnd, "symmetry_plane_vector_3");
//     visualizer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "symmetry_plane_vector_1");
//     visualizer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "symmetry_plane_vector_2");
//     visualizer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "symmetry_plane_vector_3");        
  }
  
  //----------------------------------------------------------------------------
  // Miscelanious
  //----------------------------------------------------------------------------    
  
  /** \brief update the colorbar actor of PCLInteractorStyle
   *  \param[in]  visualizer      visualizer object
   *  \param[in]  colormap        colormap object
   */    
  inline
  bool updateColorbar ( pcl::visualization::PCLVisualizer &visualizer,
                        const utl::Colormap &colormap
                    )
  {
    // Create LUT
    vtkSmartPointer<vtkLookupTable> colormapLUT = colormap.getColorLookupTable();
    
    // Update colorbar
    vtkSmartPointer<pclvis::PCLVisualizerInteractorStyle> interactorStyle = visualizer.getInteractorStyle();
    vtkRenderer *curRenderer = interactorStyle->GetCurrentRenderer();
    if (curRenderer != nullptr)
    {
      // Get 2D actors
      vtkActor2DCollection *curActors = curRenderer->GetActors2D();
      
      // Loop over actors and find the colorbar actor
      curActors->InitTraversal();
      for (size_t i = 0; i < curActors->GetNumberOfItems(); i++)
      {
        vtkActor2D *curActor = curActors->GetNextActor2D();
        if (!curActor->IsA("vtkScalarBarActor"))
          continue;
        
        // Update the LUT table
        vtkScalarBarActor *colorbarActor = vtkScalarBarActor::SafeDownCast(curActor);
        colorbarActor->SetLookupTable(colormapLUT);
        
        return true;
      }
        
      std::cout <<
        "[pcl::updateColorbar] Could not find 'vtkLookupTable' actor in current renderer. \
        This probably means that colorbar was not being displayed when this function was called" << std::endl;
    }
    else
    {
      std::cout << "[pcl::updateColorbar] CurrentRenderer does not exist yet" << std::endl;      
    }
    
    return false;
  }
  
  /** \brief show a visualization of camera view frustum
   *  \param[in]  visualizer      visualizer object
   *  \param[in]  K               camera matrix
   *  \param[in]  height          camera height
   *  \param[in]  width           camera width
   *  \param[in]  pose            camera pose
   *  \param[in]  id              name id of the camera
   */
  inline
  void showCamera ( pcl::visualization::PCLVisualizer &visualizer,
                    const Eigen::Matrix3f &K,
                    const int height, const int width,
                    const Eigen::Affine3f &pose,
                    const std::string &id = "cam"
                  )
  {
    float focal = (K(0,0) + K(1,1)) / 2;
    float height_f = static_cast<float>(height);
    float width_f = static_cast<float>(width);
    
    // create a 5-point visual for each camera
    pcl::PointXYZ p1, p2, p3, p4, p5;
    p1.x=0; p1.y=0; p1.z=0;
    float angleX = pcl::rad2deg (2.0 * std::atan (width_f / (2.0*focal)));
    float angleY = pcl::rad2deg (2.0 * std::atan (height_f / (2.0*focal)));
    float dist = 0.1;
    float minX, minY, maxX, maxY;
    maxX = dist*tan (std::atan (width_f / (2.0*focal)));
    minX = -maxX;
    maxY = dist*tan (std::atan (height_f / (2.0*focal)));
    minY = -maxY;
    p2.x=minX; p2.y=minY; p2.z=dist;
    p3.x=maxX; p3.y=minY; p3.z=dist;
    p4.x=maxX; p4.y=maxY; p4.z=dist;
    p5.x=minX; p5.y=maxY; p5.z=dist;
        
    p1=pcl::transformPoint (p1, pose);
    p2=pcl::transformPoint (p2, pose);
    p3=pcl::transformPoint (p3, pose);
    p4=pcl::transformPoint (p4, pose);
    p5=pcl::transformPoint (p5, pose);
    std::stringstream ss;
    visualizer.addText3D(id, p1, 0.01, 1.0, 1.0, 1.0, id);

    visualizer.addLine (p1, p2, id + "_line1");
    visualizer.addLine (p1, p3, id + "_line2");
    visualizer.addLine (p1, p4, id + "_line3");
    visualizer.addLine (p1, p5, id + "_line4");
    visualizer.addLine (p2, p5, id + "_line5");
    visualizer.addLine (p5, p4, id + "_line6");
    visualizer.addLine (p4, p3, id + "_line7");
    visualizer.addLine (p3, p2, id + "_line8");
  }
  
  /** \brief show a visualization of camera view frustum
   *  \param[in]  visualizer      visualizer object
   *  \param[in]  K               camera matrix
   *  \param[in]  height          camera height
   *  \param[in]  width           camera width
   *  \param[in]  pose            camera pose
   *  \param[in]  id              name id of the camera
   */
  inline
  void removeCamera ( pcl::visualization::PCLVisualizer &visualizer, const std::string &id = "cam")
  {
    visualizer.removeText3D(id);
    visualizer.removeShape(id + "_line1");
    visualizer.removeShape(id + "_line2");
    visualizer.removeShape(id + "_line3");
    visualizer.removeShape(id + "_line4");
    visualizer.removeShape(id + "_line5");
    visualizer.removeShape(id + "_line6");
    visualizer.removeShape(id + "_line7");
    visualizer.removeShape(id + "_line8");
  }  
}
#endif // PCL_QUICK_VIS_HPP_