/*
 * color_util.cpp
 *
 *  Created on: Sep 10, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */
#include <reco/utils/color_util.h>
#include <chrono>
#include <random>

namespace reco{
namespace utils{
static std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
static std::uniform_int_distribution<uint32_t> distribution(0,255);

uint32_t generate_random_color(){
	uint32_t r = distribution(generator);
	uint32_t g = distribution(generator);
	uint32_t b = distribution(generator);
	return ((r << 16) | (g << 8) | b);
}

}//end namespace utils
}//end namespace reco




