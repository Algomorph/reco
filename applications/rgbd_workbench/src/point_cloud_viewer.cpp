/*
 * point_cloud_player.cpp
 *
 *  Created on: Sep 14, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/utils/debug_util.h>
#include "point_cloud_viewer.h"

namespace reco {
namespace workbench {

point_cloud_viewer::point_cloud_viewer(std::shared_ptr<point_cloud_buffer> cloud_buffer,
		QVTKWidget* hosting_widget):
		worker(),
		cloud_buffer(cloud_buffer),
		visualizer(new pcl::visualization::PCLVisualizer("result view", false)),
		hosting_widget(hosting_widget)
		{
	hosting_widget->SetRenderWindow(visualizer->getRenderWindow());
	visualizer->setupInteractor(hosting_widget->GetInteractor(),
				hosting_widget->GetRenderWindow());
	visualizer->setCameraPosition(
					0.0, 0.0, 0.0,   // camera position
					0.0, 0.0, 1.0,   // viewpoint
					0.0, -1.0, 0.0,  // normal
					0.0);            // viewport
	hosting_widget->update();
}

point_cloud_viewer::~point_cloud_viewer(){
	//unpause and stop
	stop();
}

bool point_cloud_viewer::do_unit_of_work(){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloud_buffer->grab_next_point_cloud();
	if(cloud){
		if (!visualizer->updatePointCloud(cloud)) {
			visualizer->addPointCloud(cloud);
		}
		//TODO: is this necessary? maybe, try redraw() instead?
		hosting_widget->update();
	}else{
		this->pause();
	}
	return true;
}


} /* namespace workbench */
} /* namespace reco */
