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
	using namespace reco::workbench;

	Q_INIT_RESOURCE(application);
	QApplication app(argc, argv);
	app.setOrganizationName("QtProject");
	app.setApplicationName("Application Example");
	main_window main_window;
	main_window.show();
	return app.exec();
}


