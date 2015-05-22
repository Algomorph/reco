/*
 * TypeRegistration.h
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#ifndef MODULES_VIDEO_STATICTYPEREGISTRATION_H_
#define MODULES_VIDEO_STATICTYPEREGISTRATION_H_
#pragma once

#include <QMetaType>
#include <opencv2/core/core.hpp>
#include <QDebug>

Q_DECLARE_METATYPE(cv::Mat);

namespace augmentarium{
namespace video{
class StaticTypeRegistration{
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
static int staticInitializationDummy = StaticTypeRegistration::initialize();
}
}

#endif /* MODULES_VIDEO_STATICTYPEREGISTRATION_H_ */
