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



#include "iCub/tactileGrasp/GraspThread.h"

#include <iostream>
#include <algorithm>

#include <yarp/os/Property.h>

using std::cerr;
using std::cout;
using std::string;

using yarp::os::RateThread;
using iCub::tactileGrasp::GraspThread;

/* *********************************************************************************************************************** */
/* ******* Constructor                                                      ********************************************** */   
GraspThread::GraspThread(const int aPeriod, const string &aRobotName, const string &aWhichHand) 
    : RateThread(aPeriod) {
        period = aPeriod;
        robotName = aRobotName;
        whichHand = aWhichHand;

        dbgTag = "GraspThread: ";
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Destructor                                                       ********************************************** */   
GraspThread::~GraspThread() {}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Initialise thread                                                ********************************************** */
bool GraspThread::threadInit(void) {
    using yarp::os::Property;

    cout << dbgTag << "Initialising. \n";

    /* ******* Ports                                ******* */
    portGraspThreadInSkinComp.open("/TactileGrasp/skin/" + whichHand + "_hand_comp:i");
    portGraspThreadInSkinRaw.open("/TactileGrasp/skin/" + whichHand + "_hand_raw:i");
    portGraspThreadInSkinContacts.open("/TactileGrasp/skin/contacts:i");

    /* ******* Joint interfaces                     ******* */
    string arm = whichHand + "_arm";
    Property options;
    options.put("robot", robotName.c_str()); 
    options.put("device", "remote_controlboard");        
    options.put("part", arm.c_str());
    options.put("local", ("/TactileGrap/" + arm).c_str());
    options.put("remote", ("/" + robotName + arm).c_str());
    
    // Open driver
    if (!clientArm.open(options)) {
        return false;
    }
    // Open interfaces
    clientArm.view(iEncs);
    if (!iEncs) {
        return false;
    }
    clientArm.view(iPos);
    if (!iPos) {
        return false;
    }
    clientArm.view(iVel);
    if (!iVel) {
        return false;
    }

    /* ******* Store position prior to acquiring control.           ******* */
    int nJoints;
    iEncs->getAxes(&nJoints);
    startPos.resize(nJoints);
    iEncs->getEncoders(startPos.data());
    
    cout << dbgTag << "Initialised correctly. \n";
    
    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Run thread                                                       ********************************************** */
void GraspThread::run(void) {
    using std::vector;

    vector<double> contacts;
    detectContact(contacts);
}  
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Release thread                                                   ********************************************** */
void GraspThread::threadRelease(void) {
    cout << dbgTag << "Releasing. \n";
    
    // Close ports
    portGraspThreadInSkinComp.interrupt();
    portGraspThreadInSkinRaw.interrupt();
    portGraspThreadInSkinContacts.interrupt();
    portGraspThreadInSkinComp.close();
    portGraspThreadInSkinRaw.close();
    portGraspThreadInSkinContacts.close();

    // Stop interfaces
    if (iVel) {
        iVel->stop();
    }
    if (iPos) {
        iPos->stop();
        // Restore initial robot position
        iPos->positionMove(startPos.data());
        bool done = false;
        while (!done) {
            iPos->checkMotionDone(&done);
        }
    }

    // Close driver
    clientArm.close();

    cout << dbgTag << "Released. \n";
}
/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* Detect contact on each finger.                                   ********************************************** */
bool GraspThread::detectContact(std::vector<double> &o_contacts) {
    using yarp::sig::Vector;
    using std::vector;

    Vector *inComp = portGraspThreadInSkinComp.read(false);
    if (inComp->size() > 0) {
        // Convert yarp vector into stl vector
        vector<double> contacts(12*5);
        for (size_t i = 0; i < contacts.size(); ++i) {
            contacts[i] = (*inComp)[i];
        }

        // Find maximum for each finger
        o_contacts.resize(5, 0.0);
        vector<double>::iterator start;
        vector<double>::iterator end;
        for (int i = 0; i < 5; ++i) {
            start = contacts.begin() + 12*i;
            end = start + 11;
            
            cout << dbgTag << *start << " " << *end << "\n";

            o_contacts[i] = *std::max_element(start, end);
        }
    }

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Detect contact on each finger.                                   ********************************************** */
bool GraspThread::detectContact(iCub::skinDynLib::skinContactList &o_contacts) {
    using iCub::skinDynLib::skinContactList;

    // Read contacts from port
    skinContactList *scl = portGraspThreadInSkinContacts.read(false);    
   
    return false;
}
/* *********************************************************************************************************************** */
