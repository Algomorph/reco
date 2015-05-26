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

#include <QMetaType>
#include <opencv2/core/core.hpp>
#include <QDebug>

Q_DECLARE_METATYPE(cv::Mat);

namespace reco{
namespace datapipe{
class static_type_registration{
	static bool registrationDone;
public:
	static int initialize(){
		if(!registrationDone){
			registrationDone = true;
			qRegisterMetaType<cv::Mat>("Mat");
		}
		return 0;
	};
};
static int static_initialization_dummy = static_type_registration::initialize();
}//end namespace datapipe
}//end namespace reco

#endif /* RECO_DATAPIPE_STATICTYPEREGISTRATION_H_ */
