
/* 
 * Copyright (C) 2014 Francesco Giovannini, iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Francesco Giovannini
 * email:   francesco.giovannini@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#ifndef __ICUB_TACTILEGRASP_GAZETHREAD_H__
#define __ICUB_TACTILEGRASP_GAZETHREAD_H__

#include <string>

#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>

namespace iCub {
    namespace tactileGrasp {
        class GazeThread : public yarp::os::RateThread {
            private:
                /* ******* Module attributes.               ******* */
                int period;
                yarp::os::ResourceFinder rf;

                /* ******* Cartesian controller.                ******* */
                yarp::dev::PolyDriver clientCart;
                yarp::dev::ICartesianControl *iCart;
                int startup_context_id_cart;
                
                /* ******* Gaze controller.                     ******* */
                yarp::dev::PolyDriver clientGaze;
                yarp::dev::IGazeControl *iGaze;
                int startup_context_id_gaze;

                yarp::sig::Vector startGaze;

                /* ******* Debug attributes.                ******* */
                std::string dbgTag;

            public:
                GazeThread(const int aPeriod, const yarp::os::ResourceFinder &aRf);
                
                bool threadInit();     
                void threadRelease();
                void run();

            private:
                bool lookAtObject();
        };
    } //namespace tactileGrasp
} //namespace iCub

#endif

