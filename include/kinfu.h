#ifndef KINFU_H_
#define KINFU_H_

#include <Eigen/Eigen>

// ARPG includes
#include <sophus/se3.hpp>

#include <pangolin/pangolin.h>
#include <pangolin/glcuda.h>
#include <pangolin/glvbo.h>

#include <SceneGraph/SceneGraph.h>

#include <kangaroo/kangaroo.h>
#include <kangaroo/BoundedVolume.h>
#include <kangaroo/MarchingCubes.h>
#include <kangaroo/extra/ImageSelect.h>
#include <kangaroo/extra/BaseDisplayCuda.h>
#include <kangaroo/extra/DisplayUtils.h>
#include <kangaroo/extra/Handler3dGpuDepth.h>
#include <kangaroo/extra/SavePPM.h>
#include <kangaroo/extra/SaveMeshlab.h>

// OpenCV includes
#include <opencv2/core/core.hpp>

// // PCL includes
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/io/pcd_io.h>

using namespace std;

class Kinfu
{  
public:
  Kinfu (int w, int h, bool useColour, roo::ImageIntrinsics K_depth, int volRes, float volRad, float mincostheta=0.1, float knear = 0.4, float kfar = 4.0, float trunc_dist_factor = 2.0);
 
  // Get viewonly_ flag
  bool getViewonly();
  
  // Set viewonly_ flag
  void setViewonly (bool viewonly);
  
  // Add a frame to fuse. Frame is only added if viewonly_ is false and run is true
  bool addFrame(const cv::Mat &depth, const cv::Mat &rgb);
  
  // Set the transform of the first frame camera pose to the volume center
  void setVolumeOrigin(const Eigen::Affine3d vol_origin);
  
  // Spin once
  bool spinOnce();
  
  // Extract a mesh from current TSDF and save it to stl file
  void saveMesh(const std::string filename);
  
  // Get current camera pose
  Eigen::Matrix4d getCameraPose();
  
//   // Save SDF to a file
//   void saveSDF(const string filename);
  
private:
  
  // Sensor calibration parameters
  const int w_, h_;  
  const roo::ImageIntrinsics K_depth_;         // Depth sensor calibration parameters
  const double knear_;                        // Near and far depth clipping planes
  const double kfar_;

  // Sensor calibration variables
  Eigen::Vector3d c_d_;
  Sophus::SE3d T_cd_;  
  
  // Kinfu algorithm parameters
  bool useColour_;
  // NOTE: this class should be templated on the number of pyramid levels
  const int maxLevels_ = 4;
  const int its_[4] = {3,3,3,3};              // Number of iteration in each level
  const float volRad_;                       // Radius of the volume cube in metres
  const int volRes_;                       // Number of voxels for each dimension of the volume cube
  
  // Kinfu variables
  Sophus::SE3d volOrigin_;
  Sophus::SE3d T_wl_;                         // Transform of the camera to volume origin
  roo::BoundingBox reset_bb_;
  
  // NOTE: that the integer template value, maxLevels_ and size of its_[] must all be the same
  roo::Image<float, roo::TargetDevice, roo::Manage> dKinect_;      // Depth image
  roo::Image<uchar3, roo::TargetDevice, roo::Manage> drgb_;                 // RGB image
  roo::Image<float, roo::TargetDevice, roo::Manage> dKinectMeters_;         // Depth image in metres
  roo::Pyramid<float, 4, roo::TargetDevice, roo::Manage> kin_d_;            // Depth pyramind
  roo::Pyramid<float4, 4, roo::TargetDevice, roo::Manage> kin_v_;   // Volume???
  roo::Pyramid<float4, 4, roo::TargetDevice, roo::Manage> kin_n_;   // Normal pyramid
  roo::Image<float4, roo::TargetDevice, roo::Manage>  dDebug_;              // ???
  roo::Image<unsigned char, roo::TargetDevice, roo::Manage> dScratch_;      // ???
  
  roo::Pyramid<float, 4, roo::TargetDevice, roo::Manage> ray_i_;    // Ray image
  roo::Pyramid<float, 4, roo::TargetDevice, roo::Manage> ray_d_;    // Ray depth
  roo::Pyramid<float4, 4, roo::TargetDevice, roo::Manage> ray_n_;   // Ray normal
  roo::Pyramid<float4, 4, roo::TargetDevice, roo::Manage> ray_v_;   // Ray ???
  roo::Pyramid<float4, 4, roo::TargetDevice, roo::Manage> ray_c_;   // Ray colour
  roo::BoundedVolume<roo::SDF_t, roo::TargetDevice, roo::Manage> vol_;
  roo::BoundedVolume<float, roo::TargetDevice, roo::Manage> colorVol_;
  
  vector<std::unique_ptr<KinectKeyframe> > keyframes_;
  roo::Mat<roo::ImageKeyframe<uchar3>,10> kfs_;

  
  // Visualization parameters
  pangolin::View& container_;
  SceneGraph::GLSceneGraph glgraph_;
  SceneGraph::GLAxis glcamera_;
  SceneGraph::GLAxisAlignedBox glboxfrustum_;
  SceneGraph::GLAxisAlignedBox glboxvol_;
  
  // UI parameters
  pangolin::Var<bool> run_;
  pangolin::Var<bool> showcolor_;
  pangolin::Var<bool> viewonly_;
  pangolin::Var<bool> fuse_;
  pangolin::Var<bool> reset_;

  pangolin::Var<int> show_level_;

  // TODO: This needs to be a function of the inverse depth
  pangolin::Var<int> biwin_;
  pangolin::Var<float> bigs_;
  pangolin::Var<float> bigr_;

  pangolin::Var<bool> pose_refinement_;
  pangolin::Var<float> icp_c_;
  pangolin::Var<float> trunc_dist_factor_;

  pangolin::Var<float> max_w_;
  pangolin::Var<float> mincostheta_;

  pangolin::Var<bool> save_kf_;
  pangolin::Var<float> rgb_fl_;
  pangolin::Var<float> max_rmse_;
  pangolin::Var<float> rmse_;
  
  pangolin::ActivateDrawPyramid<float,4> adrayimg_;
  pangolin::ActivateDrawPyramid<float4,4> adraycolor_;
  pangolin::ActivateDrawPyramid<float4,4> adraynorm_;
  pangolin::ActivateDrawPyramid<float4,4> adnormals_;
  pangolin::ActivateDrawImage<float4> addebug_;
  
  pangolin::OpenGlRenderState s_cam_;
  Handler3DDepth<float,roo::TargetDevice> rayhandler_;

  bool newFrame_;
};

#endif // KINFU_H_
