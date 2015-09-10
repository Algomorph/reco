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

void point_cloud_buffer::append_point_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud) {
	//go to the end of the stream
	compressed_data.seekp(0,ios_base::end);
	//compress
	point_cloud_encoder->encodePointCloud(cloud, compressed_data);
	frame_positions.push_back(compressed_data.tellp());
	size++;
}

void point_cloud_buffer::clear(){
	compressed_data.clear();
	frame_positions.clear();
	playback_counter = 0;
	compressed_data.seekp(0,ios_base::beg);
	size = 0;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_buffer::grab_point_cloud(){
	if(!compressed_data.eof()){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB> ());
		point_cloud_decoder->decodePointCloud(compressed_data,cloud_out);
		return cloud_out;
	}else{
		return NULL;
	}
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_buffer::grab_point_cloud(uint ix_frame){
	if(ix_frame < size){
		compressed_data.seekp(frame_positions[ix_frame]);
		return grab_point_cloud();
	}else{
		return NULL;
	}
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_buffer::grab_next_point_cloud(){
	if(playback_counter < size){
		compressed_data.seekp(frame_positions[playback_counter]);
		playback_counter++;
		return grab_point_cloud();
	}else{
		return NULL;
	}
}

point_cloud_buffer::point_cloud_buffer():
	compression_profile(pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR),
	point_cloud_encoder(new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile, false)),
	point_cloud_decoder(new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> ()),
	playback_counter(0),
	size(0){
	frame_positions.push_back(0);
}

point_cloud_buffer::~point_cloud_buffer(){
	delete point_cloud_encoder;
	delete point_cloud_decoder;
}

} /* namespace workbench */
} /* namespace reco */
