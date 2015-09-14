/*
 * TypeRegistration.h
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#ifndef RECO_DATAPIPE_STATICTYPEREGISTRATION_H_
#define RECO_DATAPIPE_STATICTYPEREGISTRATION_H_
#pragma once

//qt
#include <QMetaType>
#include <QDebug>

//opencv
#include <opencv2/core/core.hpp>

//std
#include <vector>
#include <memory>

Q_DECLARE_METATYPE(cv::Mat);

namespace reco{
namespace datapipe{
class static_type_registration{
	static bool registrationDone;
public:
	static int initialize(){

		if(!registrationDone){
			registrationDone = true;
			qRegisterMetaType<size_t>("size_t");
			qRegisterMetaType<cv::Mat>("cv::Mat");
			qRegisterMetaType<std::vector<cv::Mat>>("std::vector<cv::Mat>");
			qRegisterMetaType<std::shared_ptr<std::vector<cv::Mat>>>("std::shared_ptr<std::vector<cv::Mat>>");
		}
		return 0;
	};
};
static int static_initialization_dummy = static_type_registration::initialize();
}//end namespace datapipe
}//end namespace reco

#endif /* RECO_DATAPIPE_STATICTYPEREGISTRATION_H_ */
