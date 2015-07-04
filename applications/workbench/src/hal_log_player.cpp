/*
 * hallogplayer.cpp
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

//local
#include <src/hal_log_player.h>
#include <reco/workbench/kinect_v2_info.h>
#include <reco/utils/cpp_exception_util.h>

//arpg
#include <HAL/Messages/ImageArray.h>
#include <HAL/Messages/Logger.h>
#include <HAL/Messages/Matrix.h>


namespace reco {uni
namespace workbench {

/**
 * If the is_ready() returns true, successively calles the next() function until the program reaches
 * the end or the stop_requested flag is set to true.
 * Resets the stop_requested flag before exit.
 */
void hal_log_player::play(){
	if(ready){
		while(!stop_requested && next()){
			emit frame(*images.get());
		}
		stop_requested = false;
	}
}
/**
 * Sets the stop_requested flag to true, thereby causing the playback to stop at next iteration if it is underway
 */
void hal_log_player::stop(){
	stop_requested = true;
}
bool hal_log_player::is_ready(){
	return ready;
}

bool hal_log_player::next(){
	//TODO
		throw new reco::utils::not_implemented();
}

bool hal_log_player::prev(){
	//TODO
	throw new reco::utils::not_implemented();
}
void hal_log_player::go_to_start(){
	//TODO
	throw new reco::utils::not_implemented();
}
void hal_log_player::go_to_end(){
	//TODO
	throw new reco::utils::not_implemented();
}
void hal_log_player::go_to_frame(uint index){
	//TODO
	throw new reco::utils::not_implemented();
}
uint hal_log_player::frame_count(){
	//TODO	bool file_opened = false;
	throw new reco::utils::not_implemented();
}

/**
 *
 * @param path
 */
void hal_log_player::open_file(const std::string& path){
	if(!ready){
		reader = std::unique_ptr<hal::Reader>(new hal::Reader(path));
	}else{
		close_file();
	}
}

void hal_log_player::close_file(){
	if(ready){

	}
}

hal_log_player::hal_log_player()
{
	throw new reco::utils::not_implemented();

}

hal_log_player::~hal_log_player()
{
	// TODO Auto-generated destructor stub
}

} /* namespace workbench */
} /* namespace reco */
