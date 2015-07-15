/*
 * offset_channel_viewer.h
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#ifndef RECO_DATAPIPE_OFFSET_CHANNEL_VIEWER_H_
#define RECO_DATAPIPE_OFFSET_CHANNEL_VIEWER_H_
#pragma once

//datapipe
#include <reco/datapipe/multichannel_viewer.h>

//std
#include <cstddef>

namespace reco {
namespace datapipe {

template<std::size_t N, std::size_t X>
class offset_channel_viewer: public multichannel_viewer {
public:
	offset_channel_viewer(QString window_title = "Feed Viewer", QWidget* parent = NULL): multichannel_viewer(window_title,parent){};
	virtual ~offset_channel_viewer(){};

protected:
	virtual std::vector<int> select_channels(int total_channels);
};

} /* namespace datapipe */
} /* namespace reco */

#include "offset_channel_viewer.tpp"

#endif /* MODULES_DATAPIPE_INCLUDE_RECO_DATAPIPE_OFFSET_CHANNEL_VIEWER_H_ */
