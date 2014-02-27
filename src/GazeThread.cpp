
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
#include <sstream>

#include "iCub/tactileGrasp/GazeThread.h"

#include <yarp/sig/Vector.h>
#include <yarp/os/Property.h>

using std::cout;
using std::string;

using iCub::tactileGrasp::GazeThread;
using yarp::os::RateThread;
using yarp::dev::ICartesianControl;
using yarp::dev::IGazeControl;

GazeThread::GazeThread(const int aPeriod, const string &aRobotName, const string &aWhichHand)
    : RateThread(aPeriod) {
        period = aPeriod;
        robotName = aRobotName;
        whichHand = aWhichHand;

        dbgTag = "GazeThread: ";
}

bool GazeThread::threadInit() {
    using yarp::os::Property;
    using yarp::os::Bottle;
    using std::stringstream;

    cout << dbgTag << "Starting thread. \n";

    /* ****** Cartesian controller stuff                      ****** */
    Property optCart;
    optCart.put("device", "cartesiancontrollerclient");
    optCart.put("remote", ("/" + robotName + "/cartesianController/" + whichHand + "_arm").c_str());
    optCart.put("local", ("/cartesian_client/" + whichHand + "_arm").c_str());
    
    if (!clientCart.open(optCart))
        return false;
    // open the view
    clientCart.view(iCart);
    // latch the controller context in order to preserve it after closing the module
    iCart->storeContext(&startup_context_id_cart);
    // print out some info about the controller
    Bottle info;
    iCart->getInfo(info);
    stringstream ss;
    ss << "Cartesian controller info = " << info.toString().c_str();
    cout << ss;

    /* ****** Gaze controller stuff                               ****** */
    Property optGaze;
    optGaze.put("device", "gazecontrollerclient");
    optGaze.put("remote", "/iKinGazeCtrl");
    optGaze.put("local", "/client/gaze");

    if (!clientGaze.open(optGaze))
        return false;
    // open the view
    clientGaze.view(iGaze);
    // latch the controller context in order to preserve it after closing the module
    iGaze->storeContext(&startup_context_id_gaze);
    // print out some info about the controller
    info.clear();
    iGaze->getInfo(info);
    ss.str(std::string());
    ss << "Gaze controller info = " << info.toString().c_str();
    cout << ss;


    cout << dbgTag << "Done. \n";
    
    return true;
}

void GazeThread::threadRelease() {
    cout << dbgTag << "Stopping thread. \n";

    // Stop cartesian and gaze controller
    if (iCart) {
        iCart->stopControl();
        // restore the controller context as it was before opening the module
        iCart->restoreContext(startup_context_id_cart);
    }
    if (iGaze) {
        iGaze->stopControl();
        // restore the controller context as it was before opening the module
        iGaze->restoreContext(startup_context_id_gaze);
    }

    clientCart.close();
    clientGaze.close();

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

