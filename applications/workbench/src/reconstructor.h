/*
 * reconstructor.h
 *
 *  Created on: Sep 10, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef RECO_WORKBENCH_RECONSTRUCTOR_H_
#define RECO_WORKBENCH_RECONSTRUCTOR_H_

namespace reco {
namespace workbench {
/**
 * Responsible for processing the range images and coming up with 3D meshes reconstructed from them
 */
class reconstructor {
public:
	reconstructor();
	virtual ~reconstructor();
};

} /* namespace workbench */
} /* namespace reco */

#endif /* RECO_WORKBENCH_RECONSTRUCTOR_H_ */
