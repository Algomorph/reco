#ifndef VISUALIZE_CORRESPONDENCES_HPP_
#define VISUALIZE_CORRESPONDENCES_HPP_

#include <pcl/visualization/pcl_visualizer.h>         // Visualization
#include <pcl/common/transforms.h>                    // Geometric transformations

template <typename PointT>
void visualizeCorrespondences(void *visualizer,
                              const pcl::PointCloud<PointT> &target_cloud,
                              const pcl::PointCloud<PointT> &source_cloud,
                              const pcl::Correspondences &correspondences,
                              size_t num_corresp_displayed,
                              Eigen::Affine3f &sourceVisRt = Eigen::Affine3f::Identity(),
                              const std::string &id = "correspondences")
{
  
  pcl::visualization::PCLVisualizer* visualizer_ = reinterpret_cast<pcl::visualization::PCLVisualizer*> (visualizer);

  // Figure out how step size
  size_t stepSize;
  if (correspondences.size() < num_corresp_displayed)
  {
    num_corresp_displayed = correspondences.size();
    stepSize = 1;
  }
  else
  {
    stepSize = correspondences.size() / num_corresp_displayed;
  }
  
  // First shift source cloudg
  typename pcl::PointCloud<PointT> sourceCloudShifted;
  pcl::transformPointCloudWithNormals<PointT>(source_cloud, sourceCloudShifted, sourceVisRt);

  // Select the required number of correspondences from the existing correspondneces
  typename pcl::PointCloud<PointT>::Ptr displayCloud(new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT> sourceCorrespondenceCloud;
  typename pcl::PointCloud<PointT> targetCorrespondenceCloud;
  pcl::Correspondences correspondencesDisplayed;
  sourceCorrespondenceCloud.resize(num_corresp_displayed);
  targetCorrespondenceCloud.resize(num_corresp_displayed);
  correspondencesDisplayed.resize(num_corresp_displayed);
  
  for (size_t i = 0; i < num_corresp_displayed; i++)
  {
    size_t queryIndex = correspondences.at(i*stepSize).index_query;
    size_t matchIndex = correspondences.at(i*stepSize).index_match;
    
    sourceCorrespondenceCloud.at(i) = sourceCloudShifted.at(queryIndex);
    targetCorrespondenceCloud.at(i) = target_cloud.at(matchIndex);
    correspondencesDisplayed.at(i).index_query = i;
    correspondencesDisplayed.at(i).index_match = i;    
  }
  
  // Display pointclouds
  *displayCloud = sourceCloudShifted + target_cloud;
  if (pcl::getFieldsList(*displayCloud).find("rgb") == std::string::npos)
  {
    if (!visualizer_->updatePointCloud<PointT> (displayCloud, "cloud"))
    {
      visualizer_->addPointCloud<PointT> (displayCloud, "cloud");
    }
  }
  else
  {
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> color_handler (displayCloud);
    if (!visualizer_->updatePointCloud<PointT> (displayCloud, color_handler, "cloud"))
    {
      visualizer_->addPointCloud<PointT> (displayCloud, color_handler, "cloud");
      visualizer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    }
  }

  // Add correspondences
  pcl::PointCloud<pcl::PointXYZ>::Ptr defaultFrame (new pcl::PointCloud<pcl::PointXYZ>);
  float frameSize = 0.01;
  defaultFrame->push_back(pcl::PointXYZ( frameSize,  -frameSize,  0.0));
  defaultFrame->push_back(pcl::PointXYZ( frameSize,   frameSize,  0.0));
  defaultFrame->push_back(pcl::PointXYZ(-frameSize,   frameSize,  0.0));
  defaultFrame->push_back(pcl::PointXYZ(-frameSize,  -frameSize,  0.0));
  
  Eigen::Vector3f frameNormal (0.0, 0.0, 1.0);  
  Eigen::Vector3f cameraCenter = visualizer_->getViewerPose().translation();
  Eigen::Vector3f cameraNormal = visualizer_->getViewerPose().rotation().col(2);
  Eigen::Vector3f cameraUp     = visualizer_->getViewerPose().rotation().col(1);
  std::vector<std::vector<float> > colours = colourPalette(num_corresp_displayed);
  
  for (size_t i = 0; i < num_corresp_displayed; i++)
  {
    // Find all parameters
    Eigen::Vector3f sourceCenter = sourceCorrespondenceCloud.points[i].getVector3fMap();
    Eigen::Vector3f sourceCameraNormal = (cameraCenter-sourceCenter);
    sourceCameraNormal.normalize();
    sourceCenter+= 0.02 * sourceCameraNormal;
    pcl::PointXYZ sourceCenterPoint (sourceCenter[0], sourceCenter[1], sourceCenter[2]);
    
    Eigen::Vector3f targetCenter = targetCorrespondenceCloud.points[i].getVector3fMap();
    Eigen::Vector3f targetCameraNormal = (cameraCenter-targetCenter);
    targetCameraNormal.normalize();
    targetCenter+= 0.02 * targetCameraNormal;
    pcl::PointXYZ targetCenterPoint (targetCenter[0], targetCenter[1], targetCenter[2]);    

    // Draw correspondences
    std::stringstream correspondenceStr;
    correspondenceStr << "correspondece_" << i;
    visualizer_->addLine(sourceCenterPoint, targetCenterPoint, correspondenceStr.str());
    visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colours[i][0], colours[i][1], colours[i][2], correspondenceStr.str());
    visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, correspondenceStr.str());
    
    // Draw source frame
    Eigen::Matrix3f R_sourceCameraCenter = alignNormals(sourceCameraNormal, frameNormal);                           // make the frame point at the camera
    Eigen::Vector3f sourceUpProjected = cameraUp - sourceCameraNormal * (sourceCameraNormal.dot(cameraUp));
    sourceUpProjected.normalize();
    Eigen::Matrix3f R_sourceCameraUp = alignNormals(sourceUpProjected, R_sourceCameraCenter.col(0));                      // aligns the up direction of the frame with the up direction of the camera

    Eigen::Affine3f sourceFrameRt;                        // Combine transformations
    sourceFrameRt.translation() = sourceCenter;
    sourceFrameRt.linear() = R_sourceCameraUp * R_sourceCameraCenter;
        
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceFrame (new pcl::PointCloud<pcl::PointXYZ>);   // Transform pointcloud
    pcl::transformPointCloud<pcl::PointXYZ>(*defaultFrame, *sourceFrame, sourceFrameRt);
    
    std::stringstream sourceFrameStr;                   // Display
    for (size_t j = 0 ; j < sourceFrame->size(); j++)
    {
      sourceFrameStr.str() = "";
      sourceFrameStr << "source_frame_" << i << "_line_" << j;
      visualizer_->addLine(sourceFrame->points[j], sourceFrame->points[(j+1) % sourceFrame->size()], sourceFrameStr.str());
      visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colours[i][0], colours[i][1], colours[i][2], sourceFrameStr.str());
      visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, sourceFrameStr.str());      
    }
    
    // Draw target frame
    Eigen::Matrix3f R_targetCameraCenter = alignNormals(targetCameraNormal, frameNormal);                           // make the frame point at the camera
    Eigen::Vector3f targetUpProjected = cameraUp - targetCameraNormal * (targetCameraNormal.dot(cameraUp));
    targetUpProjected.normalize();
    Eigen::Matrix3f R_targetCameraUp = alignNormals(targetUpProjected, R_targetCameraCenter.col(0));                      // aligns the up direction of the frame with the up direction of the camera

    Eigen::Affine3f targetFrameRt;                        // Combine transformations
    targetFrameRt.translation() = targetCenter;
    targetFrameRt.linear() = R_targetCameraUp * R_targetCameraCenter;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetFrame (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud<pcl::PointXYZ>(*defaultFrame, *targetFrame, targetFrameRt);
    
    std::stringstream targetFrameStr;
    for (size_t j = 0 ; j < targetFrame->size(); j++)
    {
      targetFrameStr.str() = "";
      targetFrameStr << "target_frame_" << i << "_line_" << j;
      visualizer_->addLine(targetFrame->points[j], targetFrame->points[(j+1) % targetFrame->size()], targetFrameStr.str());
      visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colours[i][0], colours[i][1], colours[i][2], targetFrameStr.str());
      visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, targetFrameStr.str());      
    }
  }
  
//   if (!visualizer_->updateCorrespondences<PointT>(sourceCorrespondenceCloud, targetCorrespondenceCloud, correspondencesDisplayed, 1, id))
//     visualizer_->addCorrespondences<PointT>(sourceCorrespondenceCloud, targetCorrespondenceCloud, correspondencesDisplayed, 1, id);
//   visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.0, id);

}

#endif // VISUALIZE_CORRESPONDENCES_HPP_