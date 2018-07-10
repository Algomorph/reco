//
// Created by algomorph on 4/25/16.
//

#include <reco/datapipe/opencv_pipe.h>
namespace reco {
    namespace datapipe {

        opencv_pipe::opencv_pipe(frame_buffer_type buffer): pipe(buffer, 1) { }
        opencv_pipe::~opencv_pipe() { }

    }
}