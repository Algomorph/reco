/*
 * debug_util.h
 *
 *  Created on: Sep 3, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef RECO_UTILS_DEBUG_UTIL_H_
#define RECO_UTILS_DEBUG_UTIL_H_

#include <iostream>

#ifndef dpt

#ifdef _DEBUG
#define dpt(x) std::cout << x << std::endl;
#else
#define dpt(x)
#endif

#endif



#endif /* RECO_UTILS_DEBUG_UTIL_H_ */
