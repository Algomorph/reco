/*
 * point_cloud_buffer.cpp
 *
 *  Created on: Sep 10, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */


#include <src/point_cloud_buffer.h>
#include <pcl/compression/compression_profiles.h>

namespace reco {
namespace workbench {

point_cloud_buffer::point_cloud_buffer():
	compression_profile(pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR),
	point_cloud_encoder(new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile, false)),
	point_cloud_decoder(new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> ()),
	playback_counter(0),
	size(0){
	cloud_positions.push_back(0);
}

point_cloud_buffer::~point_cloud_buffer(){
	delete point_cloud_encoder;
	delete point_cloud_decoder;
}

/**
 * @brief Append another point cloud to the end of the stream
 * Emits the size_changed(size_t) event
 * @param cloud point cloud to append
 *
 */
void point_cloud_buffer::append_point_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud) {
	//go to the end of the stream
	compressed_data.seekp(0,ios_base::end);
	//compress
	point_cloud_encoder->encodePointCloud(cloud, compressed_data);
	cloud_positions.push_back(compressed_data.tellp());
	size++;
	emit size_changed(size);
}

/**
 * @brief Clear out the contents of the point cloud.
 * Also emits the size_changed(size_t) event
 */
void point_cloud_buffer::clear(){
	compressed_data.clear();
	cloud_positions.clear();
	playback_counter = 0;
	compressed_data.seekp(0,ios_base::beg);
	size = 0;
	emit size_changed(size);
}

/**
 * Grab the point cloud at the current stream position
 * @return point cloud at the current stream position
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_buffer::grab_point_cloud(){
	if(!compressed_data.eof()){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB> ());
		point_cloud_decoder->decodePointCloud(compressed_data,cloud_out);
		return cloud_out;
	}else{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty;
		return empty;
	}
}

/**
 * Grab the point cloud at the specified frame number
 * @param ix_frame
 * @return point cloud at the specified frame number
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_buffer::grab_point_cloud(uint ix_frame){
	if(ix_frame < size){
		compressed_data.seekp(cloud_positions[ix_frame]);
		return grab_point_cloud();
	}else{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty;
		return empty;
	}
}

/**
 * Grab the cloud pointed to by the frame counter and advance the frame counter
 * @return the cloud at the counter position
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_buffer::grab_next_point_cloud(){
	if(playback_counter < size){
		compressed_data.seekp(cloud_positions[playback_counter]);
		playback_counter++;
		return grab_point_cloud();
	}else{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty;
		return empty;
	}
}

/**
 * Set the frame counter to specific frame
 * @param ix_frame
 * @return true on success, false if the given frame exceeds the current size
 */
bool point_cloud_buffer::go_to_frame(uint ix_frame){
	if(ix_frame < size){
		playback_counter = ix_frame;
		return true;
	}
	return false;
}


} /* namespace workbench */
} /* namespace reco */
