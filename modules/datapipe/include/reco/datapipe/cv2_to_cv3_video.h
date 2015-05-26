/*
 * Cv2To3Video.h
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#ifndef RECO_DATAPIPE_CV2TO3VIDEO_H_
#define RECO_DATAPIPE_CV2TO3VIDEO_H_
#pragma once

#if CV_VERSION_EPOCH < 3
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#else
#error "Fix this header to include OpenCV3 video"
#endif

#endif /* RECO_DATAPIPE_CV2TO3VIDEO_H_ */
