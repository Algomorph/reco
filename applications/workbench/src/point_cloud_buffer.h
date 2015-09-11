/*
 * point_cloud_buffer.h
 *
 *  Created on: Sep 10, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef RECO_WORKBENCH_POINTCLOUDBUFFER_H_
#define RECO_WORKBENCH_POINTCLOUDBUFFER_H_

//PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/compression/octree_pointcloud_compression.h>

//QT includes
#include <QObject>

namespace reco {
namespace workbench {

class point_cloud_buffer : public QObject {
	Q_OBJECT

private:
	std::stringstream compressed_data;
	pcl::io::compression_Profiles_e compression_profile;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_encoder;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_decoder;
	std::vector<uint64_t> cloud_positions;
	size_t playback_counter;
	size_t size;
public:
	void append_point_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr grab_point_cloud();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr grab_point_cloud(uint ix_frame);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr grab_next_point_cloud();
	void clear();
	point_cloud_buffer();
	virtual ~point_cloud_buffer();
signals:
	void size_changed(size_t new_size);
};

} /* namespace workbench */
} /* namespace reco */

#endif /* RECO_WORKBENCH_POINTCLOUDBUFFER_H_ */
