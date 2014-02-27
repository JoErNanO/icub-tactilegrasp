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

#define TACTILEGRASP_DEBUG 1

/* *********************************************************************************************************************** */
/* ******* Constructor                                                      ********************************************** */   
GraspThread::GraspThread(const int aPeriod, const string &aRobotName, const string &aWhichHand) 
    : RateThread(aPeriod) {
        period = aPeriod;
        robotName = aRobotName;
        whichHand = aWhichHand;

        VELOCITY_CRUSH = 50;

        // Generate vectors
        for (int i = 0; i < NUM_JOINTS; ++i) {
            touchThresholds.push_back(2);
            graspJoints.push_back(11+i);
        }

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
    options.put("writeStrict", "on");
    options.put("part", arm.c_str());
    options.put("local", ("/TactileGrasp/" + arm).c_str());
    options.put("remote", ("/" + robotName + "/" + arm).c_str());
    
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
    iPos->getAxes(&nJoints);
    startPos.resize(nJoints);
    iEncs->getEncoders(startPos.data());
    for (int i = 0; i < startPos.size(); ++i) {
        cout << startPos[i] << " ";
    }
    cout << "\n";

    reachArm();

    
    cout << dbgTag << "Initialised correctly. \n";
    
    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Run thread                                                       ********************************************** */
void GraspThread::run(void) {
    using std::vector;

    vector<bool> contacts;
    detectContact(contacts);
    moveFingers(contacts);
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
        //iPos->positionMove(startPos.data());
    }

    // Close driver
    clientArm.close();

    cout << dbgTag << "Released. \n";
}
/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* Detect contact on each finger.                                   ********************************************** */
bool GraspThread::detectContact(std::vector<bool> &o_contacts) {
    using yarp::sig::Vector;
    using std::vector;

    Vector *inComp = portGraspThreadInSkinComp.read(false);
    if (inComp) {
        // Convert yarp vector to stl vector
        vector<double> contacts(12*NUM_JOINTS);
        for (size_t i = 0; i < contacts.size(); ++i) {
            contacts[i] = (*inComp)[i];
        }

        // Find maximum for each finger
        vector<double> maxContacts(NUM_JOINTS);
        vector<double>::iterator start;
        vector<double>::iterator end;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            start = contacts.begin() + 12*i;
            end = start + 11;
            maxContacts[i] = *std::max_element(start, end);
        }

#if TACTILEGRASP_DEBUG
        for (size_t i = 0; i < maxContacts.size(); ++i) {
            cout << maxContacts[i] << " ";
        }
        cout << "\n";
#endif

        // Check if contact is greater than threshold
        o_contacts.resize(NUM_JOINTS, false);
        for (size_t i = 0; i < maxContacts.size(); ++i) {
            o_contacts[i] = (maxContacts[i] >= touchThresholds[i]);
        }
    } else {
#if TACTILEGRASP_DEBUG
        cout << dbgTag << "No skin data. \n";
#endif
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

 
/* *********************************************************************************************************************** */
/* ******* Move fingers to perform grasp movement                           ********************************************** */
bool GraspThread::moveFingers(const std::vector<bool> &i_contacts) {
    using std::vector;

    vector<double> graspVelocities(NUM_JOINTS, VELOCITY_CRUSH);
    for (size_t i = 0; i < i_contacts.size(); ++i) {
        if (i_contacts[i]) {
            graspVelocities[i] = 0;
        }
    }

    return iVel->velocityMove(graspJoints.size(), &graspJoints[0], &graspVelocities[0]);
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Set touch threshold.                                             ********************************************** */
bool GraspThread::setTouchThreshold(const int aFinger, const double aThreshold) {
    if ((aFinger > 0) && (aFinger < NUM_JOINTS)) {
        touchThresholds[aFinger] = aThreshold;
        return true;
    } else {
        cerr << dbgTag << "RPC::setTouchThreshold() - The specified finger is out of range. \n";
        return false;
    }
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Open hand                                                        ********************************************** */
bool GraspThread::openHand(void) {
    iVel->stop();

    // set the fingers to the original position
    iPos->positionMove(11, 5);
    iPos->positionMove(12, 35);
    iPos->positionMove(13, 15);
    iPos->positionMove(14, 20);
    iPos->positionMove(15, 40);
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Place arm in grasping position                                   ********************************************** */ 
bool GraspThread::reachArm(void) {
    iVel->stop();

    // set the arm in the starting position
    iPos->positionMove(0 ,-25);
    iPos->positionMove(1 , 35);
    iPos->positionMove(2 , 18);
    iPos->positionMove(3 , 86);
    iPos->positionMove(4 ,-32);
    iPos->positionMove(5 , 9);
    iPos->positionMove(6 , 11);
    iPos->positionMove(7 , 40);
    iPos->positionMove(8 , 60);
    iPos->positionMove(9 , 30);
    iPos->positionMove(10, 30);
}
/* *********************************************************************************************************************** */
