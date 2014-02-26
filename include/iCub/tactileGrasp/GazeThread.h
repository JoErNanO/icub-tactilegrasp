
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
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>

namespace iCub {
    namespace tactileGrasp {
        class GazeThread : public yarp::os::RateThread {
        private:
            /* ******* Module attributes.               ******* */
            bool rightHand;						// if true use the right hand, otherwise use the left hand

            /* ******* Cartesian and Gaze controller.   ******* */
            yarp::dev::ICartesianControl *iCart;
            yarp::dev::IGazeControl *iGaze;

            yarp::sig::Vector startGaze;

            /* ******* Debug attributes.                ******* */
            std::string dbgTag;

        public:
            /* class methods */
            GazeThread(int aPeriod, yarp::dev::ICartesianControl *aICart, yarp::dev::IGazeControl *aIGaze);
            
            bool threadInit();     
            void threadRelease();
            void run();

        private:
            bool lookAtObject();
        };
    } //namespace tactileGrasp
} //namespace iCub

#endif

