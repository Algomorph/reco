/*
 * point_cloud_player.cpp
 *
 *  Created on: Sep 14, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <src/point_cloud_player.h>




namespace reco {
namespace workbench {

point_cloud_player::point_cloud_player(std::shared_ptr<point_cloud_buffer> cloud_buffer,
		QVTKWidget* hosting_widget):
		worker(),
		cloud_buffer(cloud_buffer),
		result_viewer(new pcl::visualization::PCLVisualizer("result view", false))
		{
	hosting_widget->SetRenderWindow(result_viewer->getRenderWindow());
	result_viewer->setupInteractor(hosting_widget->GetInteractor(),
				hosting_widget->GetRenderWindow());
	result_viewer->setCameraPosition(
					0.0, 0.0, 0.0,   // camera position
					0.0, 0.0, 1.0,   // viewpoint
					0.0, -1.0, 0.0,  // normal
					0.0);            // viewport
	hosting_widget->update();
}

point_cloud_player::~point_cloud_player(){}

bool point_cloud_player::do_unit_of_work(){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloud_buffer->grab_next_point_cloud();
	if(cloud){
		if (!result_viewer->updatePointCloud(cloud)) {
			result_viewer->addPointCloud(cloud);
		}
		result_viewer->spinOnce();
		this->pause();
		return true;
	}
	return true;
}


} /* namespace workbench */
} /* namespace reco */
