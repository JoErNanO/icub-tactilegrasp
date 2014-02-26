
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


#include <iostream>

#include "iCub/tactileGrasp/GazeThread.h"

#include <yarp/sig/Vector.h>

using std::cout;

using iCub::tactileGrasp::GazeThread;
using yarp::os::RateThread;
using yarp::dev::ICartesianControl;
using yarp::dev::IGazeControl;

GazeThread::GazeThread(int aPeriod, ICartesianControl *aICart, IGazeControl *aIGaze)
    : RateThread(aPeriod) {
        iCart = aICart;
        iGaze = aIGaze;

        dbgTag = "GazeThread: ";
}

bool GazeThread::threadInit() {
    cout << dbgTag << "Starting thread. \n";

    // Store initial gaze
    iGaze->getFixationPoint(startGaze);

    cout << dbgTag << "Done. \n";
    
    return true;
}

void GazeThread::threadRelease() {
    cout << dbgTag << "Stopping thread. \n";

    // Restore initial gaze
    iGaze->lookAtFixationPoint(startGaze);

    // Stop cartesian and gaze controller
    iCart->stopControl();
    iGaze->stopControl();

    cout << dbgTag << "Done. \n";
}

void GazeThread::run() {
    lookAtObject();
}

/* *********************************************************************************************************************** */
/* ******* Look at object                                                   ********************************************** */
bool GazeThread::lookAtObject() {
    using yarp::sig::Vector;

    Vector position(3), orientation(4);
    
    // Get pose
    iCart->getPose(position, orientation);
     
    // Look at object
    position[0] -= 0.1;
    iGaze->lookAtFixationPoint(position);                 // move the gaze to the desired fixation point
    bool ok = iGaze->waitMotionDone();                        // wait until the operation is done

    return ok;
}
/* *********************************************************************************************************************** */

