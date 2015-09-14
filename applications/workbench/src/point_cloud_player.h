/*
 * point_cloud_player.h
 *
 *  Created on: Sep 14, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef WORKBENCH_POINT_CLOUD_PLAYER_H_
#define WORKBENCH_POINT_CLOUD_PLAYER_H_


//utils
#include <reco/utils/worker.h>

//QT
#include <qobject.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


//workbench
#include "point_cloud_buffer.h"

namespace reco {
namespace workbench {

class point_cloud_player:
		public QObject,
		public utils::worker {
Q_OBJECT
private:
	std::shared_ptr<point_cloud_buffer> cloud_buffer;
	std::shared_ptr<pcl::visualization::PCLVisualizer> result_viewer;
protected:
	virtual bool do_unit_of_work();
public:

	point_cloud_player(std::shared_ptr<point_cloud_buffer> cloud_buffer, QVTKWidget* hosting_widget);
	virtual ~point_cloud_player();
};

} /* namespace workbench */
} /* namespace reco */

#endif /* WORKBENCH_POINT_CLOUD_PLAYER_H_ */
