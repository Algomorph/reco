/*
 * reconstructor.cpp
 *
 *  Created on: Sep 10, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <src/reconstructor.h>

namespace reco {
namespace workbench {

reconstructor::reconstructor(std::shared_ptr<point_cloud_buffer> result_buffer)
	:worker(),
	 result_buffer(result_buffer){

}

reconstructor::~reconstructor(){
	// TODO Auto-generated destructor stub
}

bool reconstructor::do_unit_of_work(){
	return true;
}

} /* namespace workbench */
} /* namespace reco */
