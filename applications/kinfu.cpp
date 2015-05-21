#include <kinfu.h>

////////////////////////////////////////////////////////////////////////////////
template<typename T>
bool isFinite(const T& x)
		{
	return x == x;
}

////////////////////////////////////////////////////////////////////////////////
Kinfu::Kinfu(int w, int h, bool useColour, roo::ImageIntrinsics K_depth, int volRes, float volRad,
		float mincostheta, float knear, float kfar, float trunc_dist_factor)
// Input parameters
:
		w_(w)
				, h_(h)
				, useColour_(useColour)
				, K_depth_(K_depth)
				, knear_(knear)
				, kfar_(kfar)
				, volRad_(volRad)
				, volRes_(volRes)
				, volOrigin_(Eigen::Affine3d::Identity())

		// CUDA objects
				, reset_bb_(make_float3(-volRad_, -volRad_, knear_),
				make_float3(volRad_, volRad_, knear_ + 2 * volRad_))
						, dKinect_(w, h)
				, drgb_(w, h)
				, dKinectMeters_(w, h)
				, kin_d_(w, h)
				, kin_v_(w, h)
				, kin_n_(w, h)
				, dDebug_(w, h)
				, dScratch_(w * sizeof(roo::LeastSquaresSystem<float, 12>), h)
				, ray_i_(w, h)
				, ray_d_(w, h)
				, ray_n_(w, h)
				, ray_v_(w, h)
				, ray_c_(w, h)
				, vol_(volRes_, volRes_, volRes_, reset_bb_)
				, colorVol_(volRes_, volRes_, volRes_, reset_bb_)

		// Visualization
				, container_(SetupPangoGLWithCuda(1024, 768))
				, glcamera_(0.1)

		// UI parameters
				, run_("ui.run", true, true)
				, showcolor_("ui.show color", false, true)
				, viewonly_("ui.view only", false, true)
				, fuse_("ui.fuse", true, true)
				, reset_("ui.reset", true, false)
				, show_level_("ui.Show Level", 0, 0, maxLevels_ - 1)
				// TODO: This needs to be a function of the inverse depth
				, biwin_("ui.size", 3, 1, 20)
				, bigs_("ui.gs", 1.5, 1E-3, 5)
				, bigr_("ui.gr", 0.1, 1E-6, 0.2)
				, pose_refinement_("ui.Pose Refinement", true, true)
				, icp_c_("ui.icp c", 0.2, 1E-3, 1)
				, trunc_dist_factor_("ui.trunc vol factor", trunc_dist_factor, 1, 4)
				, max_w_("ui.max w", 1000, 1E-2, 1E3)
				, mincostheta_("ui.min cos theta", mincostheta, 0, 1)
				, save_kf_("ui.Save KF", false, false)
				, rgb_fl_("ui.RGB focal length", 535.7, 400, 600)
				, max_rmse_("ui.Max RMSE", 0.10, 0, 0.5)
				, rmse_("ui.RMSE", 0)

		, adrayimg_(ray_i_, GL_LUMINANCE32F_ARB, true, true)
				, adraycolor_(ray_c_, GL_RGBA32F, true, true)
				, adraynorm_(ray_n_, GL_RGBA32F, true, true)
				, adnormals_(kin_n_, GL_RGBA32F_ARB, false, true)
				, addebug_(dDebug_, GL_RGBA32F_ARB, false, true)

		, s_cam_(
				pangolin::ProjectionMatrixRDF_TopLeft(w_, h_, K_depth_.fu, K_depth_.fv, K_depth_.u0,
						K_depth_.v0, 0.1, 1000),
				pangolin::ModelViewLookAtRDF(0, 0, -2, 0, 0, 0, 0, -1, 0))
						, rayhandler_(ray_d_[0], s_cam_, pangolin::AxisNone)

		, newFrame_(false)
{
	// Print out the GPU device used
	cudaDeviceProp gpuProperties;
	int gpuID;
	cudaGetDevice(&gpuID);
	cudaGetDeviceProperties(&gpuProperties, gpuID);
	cout << "Using " << gpuProperties.name << endl;

	c_d_ = Eigen::Vector3d(0.0, 0.0, 0.0);
	T_cd_ = Sophus::SE3d(Sophus::SO3d(), c_d_).inverse();

	// Initialize Kinfu stuff
	cout << "TSDF volume size: " << vol_.SizeUnits() << " metres" << endl;
	cout << "TSDF volume voxel size: " << vol_.VoxelSizeUnits() * 100 << " centimetres" << endl;
	cout << "TSDF truncation distance: " << trunc_dist_factor_ * length(vol_.VoxelSizeUnits()) * 100
			<< " centimetres" << endl;

	// Initialise visualization window
	SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();

	glboxvol_.SetBounds(roo::ToEigen(vol_.bbox.Min()), roo::ToEigen(vol_.bbox.Max()));
	glgraph_.AddChild(&glcamera_);
	glgraph_.AddChild(&glboxvol_);
	glgraph_.AddChild(&glboxfrustum_);

	SetupContainer(container_, 4, (float) w_ / h_);
	container_[0].SetDrawFunction(std::ref(adrayimg_))
			.SetHandler(&rayhandler_);
	container_[1].SetDrawFunction(SceneGraph::ActivateDrawFunctor(glgraph_, s_cam_))
			.SetHandler(new pangolin::Handler3D(s_cam_, pangolin::AxisNone));
	container_[2].SetDrawFunction(std::ref(useColour_ ? adraycolor_ : adraynorm_))
			.SetHandler(&rayhandler_);
	container_[3].SetDrawFunction(std::ref(adnormals_));

	// Register callbacks (we don't really need them)
//   pangolin::RegisterKeyPressCallback(' ', [&reset_,&viewonly_]() { reset_ = true; viewonly_=false;} );
//   pangolin::RegisterKeyPressCallback('l', [&vol_,&viewonly_]() {LoadPXM("save.vol", vol_); viewonly_ = true;} );
//   pangolin::RegisterKeyPressCallback('s', [&vol_]() {SavePXM("save.vol", vol_); } );

}

////////////////////////////////////////////////////////////////////////////////
bool Kinfu::getViewonly() {
	return viewonly_;
}
void Kinfu::setViewonly(bool viewonly) {
	viewonly_ = viewonly;
}
;

////////////////////////////////////////////////////////////////////////////////
void Kinfu::saveMesh(const string filename)
		{
	roo::SaveMesh(filename, vol_);
}

////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d Kinfu::getCameraPose()
{
//   Eigen::Affine3d T;
//   T.matrix() = T_wl_.matrix();

	return T_wl_.matrix();
}

// ////////////////////////////////////////////////////////////////////////////////
// void Kinfu::saveSDF(const string filename)
// {
//   roo::BoundedVolume<roo::SDF_t,roo::TargetHost,roo::Manage> hvol(vol_.w, vol_.h, vol_.d, vol_.bbox.Min(), vol_.bbox.Max());
//   hvol.CopyFrom(vol_);
//   
//   const float3 fScale = vol_.VoxelSizeUnits();
//   pcl::PointCloud<pcl::PointXYZI> SDFCloud;
// //   SDFCloud.resize(vol_.w * vol_.h * vol_.w);
//   int point_id = 0;
//   
//   for(GLint iX = 0; iX < hvol.Voxels().x-1; iX++) {
//     for(GLint iY = 0; iY < hvol.Voxels().y-1; iY++) {
//       for(GLint iZ = 0; iZ < hvol.Voxels().z-1; iZ++)
//       {
//         pcl::PointXYZI point;
//         roo::SDF_t SDF_value = hvol.Get(iX, iY, iZ);
//         
// //         if (isFinite(SDF_value.val) && std::abs(SDF_value.val) < 0.01)
// //         {
//           const float3 p = vol_.VoxelPositionInUnits(iX,iY,iZ); 
//           point.intensity = SDF_value.val;
//           point.x = p.x * fScale.x;
//           point.y = p.y * fScale.y;
//           point.z = p.z * fScale.z;
//           SDFCloud.push_back(point);
// //           SDFCloud.points[point_id++]= point;
// //         }
//       }
//     }
//   }
//   
//   pcl::io::savePCDFileBinary(filename, SDFCloud);
// }

////////////////////////////////////////////////////////////////////////////////
void Kinfu::setVolumeOrigin(const Eigen::Affine3d vol_origin)
		{
	Eigen::Affine3d centerTransform = Eigen::Affine3d::Identity();
	centerTransform.translation() = Eigen::Vector3d(0.0, 0.0, volRad_ + knear_);
	volOrigin_ = Sophus::SE3d(centerTransform * vol_origin);
}

////////////////////////////////////////////////////////////////////////////////
bool Kinfu::addFrame(const cv::Mat &depth, const cv::Mat &rgb)
		{
	if (viewonly_ || !run_)
		return false;

	// Add depth frame
	CV_Assert(depth.cols == w_);
	CV_Assert(depth.rows == h_);
//   CV_Assert(depth.type() == CV_16UC1);
	dKinect_.CopyFrom(roo::Image<float, roo::TargetHost>((float*) depth.data,     // Pointer to data
			depth.cols,                         // Width
			depth.rows,                         // Height
			depth.elemSize() * depth.cols));    // Pitch

	roo::ElementwiseScaleBias<float, float, float>(dKinectMeters_, dKinect_, 1.0f / 1000.0f); // Rescale depth to metres
	roo::BilateralFilter<float, float>(kin_d_[0], dKinectMeters_, bigs_, bigr_, biwin_, 0.2); // Smooth depth map

	roo::BoxReduceIgnoreInvalid<float, 4, float>(kin_d_);
	for (int l = 0; l < maxLevels_; ++l)
			{
		roo::DepthToVbo<float>(kin_v_[l], kin_d_[l], K_depth_[l]); // Generate pointcloud from depth
		roo::NormalsFromVbo(kin_n_[l], kin_v_[l]);                  // Calculate normals
	}

	// Add RGB frame
	if (useColour_)
	{
		CV_Assert(rgb.cols == w_);
		CV_Assert(rgb.rows == h_);
		CV_Assert(rgb.type() == CV_8UC3);
		drgb_.CopyFrom(roo::Image<uchar3, roo::TargetHost>((uchar3*) rgb.data,
				rgb.cols,
				rgb.rows,
				rgb.elemSize() * rgb.cols));
	}

	newFrame_ = true;
	return true;
}

////////////////////////////////////////////////////////////////////////////////
bool Kinfu::spinOnce()
{
	if (pangolin::ShouldQuit())
		return true;

	const float trunc_dist = trunc_dist_factor_ * length(vol_.VoxelSizeUnits());

	/////////////////////////////////////////
	// Initialize TSDF and fuse first frame if required
	if (pangolin::Pushed(reset_) || !isfinite(rmse_))
			{

		T_wl_ = volOrigin_;

		vol_.bbox = reset_bb_;
		roo::SdfReset(vol_, std::numeric_limits<float>::quiet_NaN());
		keyframes_.clear();

		colorVol_.bbox = reset_bb_;
		roo::SdfReset(colorVol_);

		// Fuse first kinect frame in.
		const float trunc_dist = trunc_dist_factor_ * length(vol_.VoxelSizeUnits());
		if (useColour_) {
			roo::SdfFuse(vol_, colorVol_, kin_d_[0], kin_n_[0], T_wl_.inverse().matrix3x4(),
					K_depth_, drgb_, (T_cd_ * T_wl_.inverse()).matrix3x4(),
					roo::ImageIntrinsics(rgb_fl_, drgb_), trunc_dist, max_w_, mincostheta_);
		} else {
			roo::SdfFuse(vol_, kin_d_[0], kin_n_[0], T_wl_.inverse().matrix3x4(), K_depth_,
					trunc_dist, max_w_, mincostheta_);
		}
	}

	/////////////////////////////////////////
	// If we are viewing only
	if (viewonly_) {

		Sophus::SE3d T_vw(s_cam_.GetModelViewMatrix());
		const roo::BoundingBox roi(T_vw.inverse().matrix3x4(), w_, h_, K_depth_, 0, 50);
		roo::BoundedVolume<roo::SDF_t> work_vol = vol_.SubBoundingVolume(roi);
		roo::BoundedVolume<float> work_colorVol = colorVol_.SubBoundingVolume(roi);
		if (work_vol.IsValid()) {
			if (showcolor_) {
				roo::RaycastSdf(ray_d_[0], ray_n_[0], ray_i_[0], work_vol, work_colorVol,
						T_vw.inverse().matrix3x4(), K_depth_, 0.1, 50, trunc_dist, true);
			} else {
				roo::RaycastSdf(ray_d_[0], ray_n_[0], ray_i_[0], work_vol,
						T_vw.inverse().matrix3x4(), K_depth_, 0.1, 50, trunc_dist, true);
			}

			if (keyframes_.size() > 0) {
				// populate kfs
				for (int k = 0; k < kfs_.Rows(); k++)
						{
					if (k < keyframes_.size())
							{
						kfs_[k].img = keyframes_[k]->img;
						kfs_[k].T_iw = keyframes_[k]->T_iw.matrix3x4();
						kfs_[k].K = roo::ImageIntrinsics(rgb_fl_, kfs_[k].img);
					}
					else
					{
						kfs_[k].img.ptr = 0;
					}
				}
				roo::TextureDepth<float4, uchar3, 10>(ray_c_[0], kfs_, ray_d_[0], ray_n_[0],
						ray_i_[0], T_vw.inverse().matrix3x4(), K_depth_);
			}
		}
	}

	/////////////////////////////////////////
	// Fuse current frame
	else
	{
		bool tracking_good = true;

		const roo::BoundingBox roi(
				roo::BoundingBox(T_wl_.matrix3x4(), w_, h_, K_depth_, knear_, kfar_));
		roo::BoundedVolume<roo::SDF_t> work_vol = vol_.SubBoundingVolume(roi);
		if (work_vol.IsValid())
		{
			for (int l = 0; l < maxLevels_; ++l) {
				if (its_[l] > 0) {
					const roo::ImageIntrinsics Kl = K_depth_[l];
					if (showcolor_) {
						roo::RaycastSdf(ray_d_[l], ray_n_[l], ray_i_[l], work_vol, colorVol_,
								T_wl_.matrix3x4(), Kl, knear_, kfar_, trunc_dist, true);
					} else {
						roo::RaycastSdf(ray_d_[l], ray_n_[l], ray_i_[l], work_vol,
								T_wl_.matrix3x4(), Kl, knear_, kfar_, trunc_dist, true);
					}
					roo::DepthToVbo<float>(ray_v_[l], ray_d_[l], Kl);
				}
			}

			if (pose_refinement_)
			{
				Sophus::SE3d T_lp;
				for (int l = maxLevels_ - 1; l >= 0; --l)
						{
					const Eigen::Matrix3d Kdepth = K_depth_[l].Matrix();

					for (int i = 0; i < its_[l]; ++i) {
						const Eigen::Matrix<double, 3, 4> mKT_lp = Kdepth * T_lp.matrix3x4();
						const Eigen::Matrix<double, 3, 4> mT_pl = T_lp.inverse().matrix3x4();
						roo::LeastSquaresSystem<float, 6> lss =
								roo::PoseRefinementProjectiveIcpPointPlane(
										kin_v_[l], ray_v_[l], ray_n_[l], mKT_lp, mT_pl, icp_c_,
										dScratch_, dDebug_.SubImage(0, 0, w_ >> l, h_ >> l)
												);

						Eigen::Matrix<double, 6, 6> sysJTJ = lss.JTJ;
						Eigen::Matrix<double, 6, 1> sysJTy = lss.JTy;

						// Add a week prior on our pose
						const double motionSigma = 0.2;
						const double depthSigma = 0.1;
						sysJTJ += (depthSigma / motionSigma)
								* Eigen::Matrix<double, 6, 6>::Identity();

						rmse_ = sqrt(lss.sqErr / lss.obs);
						tracking_good = rmse_ < max_rmse_;

						if (l == maxLevels_ - 1 && maxLevels_ > 1) {
							// Solve for rotation only
							Eigen::FullPivLU<Eigen::Matrix<double, 3, 3> > lu_JTJ(
									sysJTJ.block<3, 3>(3, 3));
							Eigen::Matrix<double, 3, 1> x = -1.0
									* lu_JTJ.solve(sysJTy.segment<3>(3));
							T_lp = T_lp
									* Sophus::SE3d(Sophus::SO3d::exp(x), Eigen::Vector3d(0, 0, 0));
						} else {
							Eigen::FullPivLU<Eigen::Matrix<double, 6, 6> > lu_JTJ(sysJTJ);
							Eigen::Matrix<double, 6, 1> x = -1.0 * lu_JTJ.solve(sysJTy);
							if (isFinite(x)) {
								T_lp = T_lp * Sophus::SE3d::exp(x);
							}
						}
					}
				}

				if (tracking_good) {
					T_wl_ = T_wl_ * T_lp.inverse();
//             cout << T_wl_.translation() << endl;
				}
			}
		}

		if (pose_refinement_ && fuse_) {
			if (tracking_good) {
				const roo::BoundingBox roi(T_wl_.matrix3x4(), w_, h_, K_depth_, knear_, kfar_);
				roo::BoundedVolume<roo::SDF_t> work_vol = vol_.SubBoundingVolume(roi);
				roo::BoundedVolume<float> work_colorVol = colorVol_.SubBoundingVolume(roi);
				if (work_vol.IsValid()) {
					const float trunc_dist = trunc_dist_factor_ * length(vol_.VoxelSizeUnits());
					if (useColour_) {
						roo::SdfFuse(work_vol, work_colorVol, kin_d_[0], kin_n_[0],
								T_wl_.inverse().matrix3x4(), K_depth_, drgb_,
								(T_cd_ * T_wl_.inverse()).matrix3x4(),
								roo::ImageIntrinsics(rgb_fl_, drgb_), trunc_dist, max_w_,
								mincostheta_);
					} else {
//             roo::SdfFuse(work_vol, kin_d_[0], kin_n_[0], dKinectDisc_, T_wl_.inverse().matrix3x4(), KDepth_, trunc_dist, max_w_, mincostheta_ );
						roo::SdfFuse(work_vol, kin_d_[0], kin_n_[0], T_wl_.inverse().matrix3x4(),
								K_depth_, trunc_dist, max_w_, mincostheta_);

					}
				}
			}
		}
	}

	/////////////////////////////////////////
	// Display

	glcamera_.SetPose(T_wl_.matrix());
	roo::BoundingBox bbox_work(T_wl_.matrix3x4(), w_, h_, K_depth_.fu, K_depth_.fv, K_depth_.u0,
			K_depth_.v0, knear_, kfar_);
	bbox_work.Intersect(vol_.bbox);
	glboxfrustum_.SetBounds(roo::ToEigen(bbox_work.Min()), roo::ToEigen(bbox_work.Max()));
	addebug_.SetImage(dDebug_.SubImage(0, 0, w_ >> show_level_, h_ >> show_level_));
	adnormals_.SetLevel(show_level_);
	adrayimg_.SetLevel(viewonly_ ? 0 : show_level_);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor3f(1, 1, 1);
	pangolin::FinishFrame();

//   // If this is the last frame processed -> save model
//   if ((frameGrabbed > endFrame || done) && !saved)
//   {
//     cout << frameCaptured << " frames processed" << endl;
//     std::string meshFilename = filename.substr(0, filename.length() - 4) + ".stl";
//     cout << "Saving mesh\n";
//     roo::SaveMesh_dbg(meshFilename, vol);
//     cout << "Mesh saved to " << meshFilename << endl;
//     
// //       SaveMesh(std::string filename, const BoundedVolume<T,TargetHost> vol, const BoundedVolume<TColor,TargetHost> volColor );
//     
// //       cout << "Mesh saved to " << meshFilename << endl;
//     saved = true;
//     viewonly = true;
// 
//   }

	return false;
}
