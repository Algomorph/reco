//
// Created by algomorph on 4/25/16.
//

#pragma once

//qt
#include <QObject>

//datapipe
#include <reco/datapipe/pipe.h>

namespace reco {
    namespace datapipe {

        class opencv_pipe : public pipe {

            Q_OBJECT

        public:
            opencv_pipe(frame_buffer_type buffer);
            virtual ~opencv_pipe();

        private:


//        public slots:
//            void stop();
//            void pause();
//            void run();
        };

    }//end namespace datapipe
}