/*
 * offset_channel_viewer.cpp
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#include <reco/datapipe/offset_channel_viewer.h>

namespace reco {
namespace datapipe {


template<std::size_t N, std::size_t X>
std::vector<int>
offset_channel_viewer<N,X>::select_channels(int total_channels){
	std::vector<int> selections;
	for(int ix_channel = X; ix_channel < total_channels; ix_channel+=N){
		selections.push_back(ix_channel);
	}
	return selections;
}

} /* namespace datapipe */
} /* namespace reco */
