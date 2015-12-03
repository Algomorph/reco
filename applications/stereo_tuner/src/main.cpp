/*
 * main.cpp
 *
 *  Created on: Dec 17, 2014
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2014
 */
//local
#include <QApplication>
//opencv
#include <opencv2/core/core.hpp>

//std
#include <stdexcept>
#include <iostream>

//utils
#include <reco/utils/cpp_exception_util.h>
#include "main_window.h"

int main(int argc, char* argv[]){
	using namespace reco::stereo_tuner;

	Q_INIT_RESOURCE(application);
	QApplication app(argc, argv);
	app.setOrganizationName("Reco");
	app.setApplicationName("Stereo Workbench");
	main_window main_window;
	main_window.show();
	return app.exec();
}


