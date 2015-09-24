/*
 * multifeed_pipe.h
 *
 *  Created on: Sep 21, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef RECO_DATAPIPE_MULTIFEED_PIPE_H_
#define RECO_DATAPIPE_MULTIFEED_PIPE_H_

//datapipe
#include <reco/datapipe/data_channel.h>
#include <reco/datapipe/multichannel_pipe.h>

//utils
#include <reco/utils/cpp_exception_util.h>

namespace reco{
namespace datapipe{

template<int N, const std::array<const data_channel*,N>& CH_INFO>
class multifeed_pipe: public multichannel_pipe{

public:

	multifeed_pipe(multichannel_pipe::buffer_type buffer, std::string cam_uri)
		: multichannel_pipe(buffer, cam_uri){};
	virtual ~multifeed_pipe(){};

protected:
	virtual void check_channel_number(const std::string& cam_uri, size_t num_channels);
	virtual void check_channel_dimensions(const std::string& cam_uri, int ix_channel);
};


template<int N, const std::array<const data_channel*,N>& CH_INFO>
void multifeed_pipe<N,CH_INFO>::check_channel_number(const std::string& cam_uri, size_t num_channels){
	if (num_channels % N != 0) {
			err(std::invalid_argument)
			<< "Incorrect number of channels for a set of feeds! Need a multiple of "
					<< N << "! Please check cam uri. Current uri: "
					<< cam_uri
					<< enderr;
		}
}

template<int N, const std::array<const data_channel*,N>& CH_INFO>
void multifeed_pipe<N,CH_INFO>::check_channel_dimensions(const std::string& cam_uri, int ix_channel){
	int channel_offset = ix_channel % N;
	if ((int) camera.Width(ix_channel) != CH_INFO[channel_offset]->width()
			||
			(int) camera.Height(ix_channel)
					!= CH_INFO[channel_offset]->height()) {
		err(std::invalid_argument) << "Wrong camera dimensions. " << std::endl
				<< "Expecting (width x height) for "
				<< CH_INFO[channel_offset]->name() << ":" << std::endl
				<< CH_INFO[channel_offset]->width() << " x "
				<< CH_INFO[channel_offset]->height() << std::endl
				<< "Got dimensions (width x height):" << std::endl
				<< camera.Width(ix_channel) << " x "
				<< camera.Height(ix_channel) << std::endl
				<< "Got cam_uri: " << cam_uri
				<< enderr;
	}
}

}//end namespace datapipe
}//end namespace reco

#endif /* RECO_DATAPIPE_MULTIFEED_PIPE_H_ */
