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
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

using std::cerr;
using std::cout;
using std::string;

using iCub::tactileGrasp::GraspThread;

using yarp::os::RateThread;
using yarp::os::Value;

#define TACTILEGRASP_DEBUG 1

/* *********************************************************************************************************************** */
/* ******* Constructor                                                      ********************************************** */   
GraspThread::GraspThread(const int aPeriod, const yarp::os::ResourceFinder &aRf) 
    : RateThread(aPeriod) {
        period = aPeriod;
        rf = aRf;

        nFingers = 0;

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
    using yarp::os::Network;
    using yarp::os::Bottle;
    using std::vector;

    cout << dbgTag << "Initialising. \n";

    /* ******* Extract configuration files          ******* */
    string robotName = rf.check("robot", Value("icub"), "The robot name.").asString().c_str();
    string whichHand = rf.check("whichHand", Value("right"), "The hand to be used for the grasping.").asString().c_str();


    // Build grasp parameters
    Bottle &confGrasp = rf.findGroup("graspTh");
    if (!confGrasp.isNull()) {
        // Individual touch thresholds per fingertip
        Bottle *confTouchThr = confGrasp.find("touchThresholds").asList();

        if (!confTouchThr->isNull()) {
            // Generate parameter vectors
            nFingers = confTouchThr->size();
            for (size_t i = 0; i < confTouchThr->size(); ++i) {
                touchThresholds.push_back(confTouchThr->get(i).asDouble());
            }
        } else {
            cerr << dbgTag << "Could not find the touch thresholds in the specified configuration file under the [graspTh] parameter group. \n";
            return false;
        }
    } else {
        cerr << dbgTag << "Could not find grasp configuration [graspTh] group in the specified configuration file. \n";
        return false;
    }


    /* ******* Build finger to joint map.           ******* */
    generateJointMap(touchThresholds);


    // Print out debug information
#ifdef TACTILEGRASP_DEBUG
    cout << "\n";
    cout << dbgTag << "Configured joints and thresholds: \n";
    for (size_t i = 0; i < touchThresholds.size(); ++i) {
        cout << dbgTag << "\tFinger ID: " << i << "\t Touch threshold: " << touchThresholds[i] << "\t Joints: ";
        vector<int> fingerJoints = jointMap[i];
        for (size_t j = 0; j < fingerJoints.size(); ++j) {
            cout << fingerJoints[j] << " ";
        }
        cout << "\n";
    }
    cout << "\n";
#endif


    /* ******* Initialise previous contacts.        ******* */
    previousContacts.resize(nFingers, false);


    /* ******* Ports                                ******* */
    portGraspThreadInSkinComp.open("/TactileGrasp/skin/" + whichHand + "_hand_comp:i");
    portGraspThreadInSkinRaw.open("/TactileGrasp/skin/" + whichHand + "_hand_raw:i");
    portGraspThreadInSkinContacts.open("/TactileGrasp/skin/contacts:i");


    /* ******* Joint interfaces                     ******* */
    string arm = whichHand + "_arm";
    Property options;
    options.put("robot", robotName.c_str()); 
    options.put("device", "remote_controlboard");
//    options.put("writeStrict", "on");
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
    // Set velocity control parameters
    iVel->getAxes(&nJointsVel);
    std::vector<double> refAccels(nJointsVel, 10^6);
    iVel->setRefAccelerations(&refAccels[0]);


    /* ******* Store position prior to acquiring control.           ******* */
    int nnJoints;
    iPos->getAxes(&nnJoints);
    startPos.resize(nnJoints);
    iEncs->getEncoders(startPos.data());
    // Set reference speeds
    vector<double> refSpeeds(nnJoints, 0);
    iPos->getRefSpeeds(&refSpeeds[0]);
    for (int i = 11; i < 15; ++i) {
        refSpeeds[i] = 50;
    }
    iPos->setRefSpeeds(&refSpeeds[0]);

#if TACTILEGRASP_DEBUG
    cout << "\n";
    cout << dbgTag << "Stored initial arm positions are: ";
    for (int i = 0; i < startPos.size(); ++i) {
        cout << startPos[i] << " ";
    }
    cout << "\n";
#endif

    // Put arm in position
    reachArm();


    // Connecting ports
    Network::connect(("/icub/skin/" + whichHand + "_hand_comp"), ("/TactileGrasp/skin/" + whichHand + "_hand_comp:i"));

    
    cout << dbgTag << "Initialised correctly. \n";
    
    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Run thread                                                       ********************************************** */
void GraspThread::run(void) {
    using std::deque;
    using std::vector;


    // Check that the control thread is actually being run or if this is just the module::configure() acting.
    if (velocities.grasp.size() > 0) {
        deque<bool> contacts (false, nFingers);
        vector<double> graspVelocities(nJointsVel, 0);
        if (detectContact(contacts)) {
            // Loop all contacts
            for (size_t i = 0; i < contacts.size(); ++i) {
                vector<int> fingerJoints = jointMap[i];
                if (!contacts[i]) {
                    // Loop all joints in that finger
                    for (int j = 0; j < fingerJoints.size(); ++j) {
                        // FG: -8 is required as the velocities array contains only finger joints speeds i.e. joints with id >= 8.
                        graspVelocities[fingerJoints[j]] = velocities.grasp[fingerJoints[j] - 8];
                    }
                } else {
                    // Loop all joints in that finger
                    for (int j = 0; j < fingerJoints.size(); ++j) {
                        // FG: -8 is required as the velocities array contains only finger joints speeds i.e. joints with id >= 8.
                        graspVelocities[fingerJoints[j]] = velocities.stop[fingerJoints[j] - 8];
                    }
                }
            }
        } else {
            cout << dbgTag << "No contact. \n";
        }

#if TACTILEGRASP_DEBUG
        cout << dbgTag << "Moving joints at velocities: \t";
        for (size_t i = 0; i < graspVelocities.size(); ++i) {
            cout << i << " " << graspVelocities[i] << "\t";
        }
        cout << "\n";
#endif

        // Send move command
        iVel->velocityMove(&graspVelocities[0]);
    } else {
#if TACTILEGRASP_DEBUG
        cout << dbgTag << "Module initialisation running. \n";
#endif
    }
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
bool GraspThread::detectContact(std::deque<bool> &o_contacts) {
    using yarp::sig::Vector;
    using std::deque;
    using std::vector;

    Vector *inComp = portGraspThreadInSkinComp.read(false);
    if (inComp) {
        // Convert yarp vector to stl vector
        vector<double> contacts(12*nFingers);
        for (size_t i = 0; i < contacts.size(); ++i) {
            contacts[i] = (*inComp)[i];
        }

        // Find maximum for each finger
        vector<double> maxContacts(nFingers);
        vector<double>::iterator start;
        vector<double>::iterator end;
        for (int i = 0; i < nFingers; ++i) {
            start = contacts.begin() + 12*i;
            end = start + 11;
            maxContacts[i] = *std::max_element(start, end);
        }

#if TACTILEGRASP_DEBUG
        cout << dbgTag << "Maximum contact detected: \t\t";
        for (size_t i = 0; i < maxContacts.size(); ++i) {
            cout << maxContacts[i] << " ";
        }
        cout << "\n";
#endif

        // Check if contact is greater than threshold
        o_contacts.resize(nFingers, false);
        for (size_t i = 0; i < maxContacts.size(); ++i) {
            o_contacts[i] = (maxContacts[i] >= touchThresholds[i]);
        }

        // Store previous contacts
        previousContacts = o_contacts;
    } else {
#if TACTILEGRASP_DEBUG
        cout << dbgTag << "No skin data. \n";
        cout << dbgTag << "Using previous skin value. \n";
#endif
        o_contacts = previousContacts;
    }

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Move fingers to perform grasp movement                           ********************************************** */
bool GraspThread::moveFingers(const std::deque<bool> &i_contacts) {
    using std::deque;
    using std::vector;

//    vector<double> graspVelocities(i_contacts.size(), velocities.grasp);
//    vector<double> graspVelocities(velocities.grasp);
//    for (size_t i = 0; i < i_contacts.size(); ++i) {
//        if (i_contacts[i]) {
//            graspVelocities[i] = velocities.stop[i];
//        }
//    }
//
//#ifdef TACTILEGRASP_DEBUG
//    cout << dbgTag << "Moving joints at speeds: \t\t";
//    for (int i = 0; i < i_contacts.size(); ++i) {
//        cout << graspJoints[i] << " " << graspVelocities[i] << "\t";
//    }
//    cout << "\n";
//#endif
//
//    double vels[5];
//    int join[5];
//    std::copy(graspVelocities.begin(), graspVelocities.end(), vels);
//    std::copy(graspJoints.begin(), graspJoints.end(), vels);

//    return iVel2->velocityMove(graspJoints.size(), &graspJoints[0], &graspVelocities[0]);
//    return iVel2->velocityMove(graspJoints.size(), join, vels);
    
    vector<double> graspVelocities(nJointsVel, 0);
    for (size_t i = 0; i < i_contacts.size(); ++i) {
        if (i_contacts[i]) {
            graspVelocities[11+i] = velocities.stop[i];
        } else {
            graspVelocities[11+i] = velocities.grasp[i];
        }
    }
    
#ifdef TACTILEGRASP_DEBUG
    cout << dbgTag << "Moving joints at speeds: \t\t";
    for (size_t i = 11; i < graspVelocities.size(); ++i) {
        cout << i << " " << graspVelocities[i] << "\t";
    }
    cout << "\n";
#endif

    return iVel->velocityMove(&graspVelocities[0]);
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Set touch threshold.                                             ********************************************** */
bool GraspThread::setTouchThreshold(const int aFinger, const double aThreshold) {
    if ((aFinger > 0) && (aFinger < nFingers)) {
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

    // Set the fingers to the original position
    iPos->positionMove(11, 5);
    iPos->positionMove(12, 35);
    iPos->positionMove(13, 15);
    iPos->positionMove(14, 20);
    iPos->positionMove(15, 40);

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Place arm in grasping position                                   ********************************************** */ 
bool GraspThread::reachArm(void) {
    using yarp::os::Time;

    iVel->stop();

    // Set the arm in the starting position
    // Arm
    iPos->positionMove(0 ,-25);
    iPos->positionMove(1 , 35);
    iPos->positionMove(2 , 18);
    iPos->positionMove(3 , 46);
    iPos->positionMove(4 ,-32);
    iPos->positionMove(5 , 9);
    iPos->positionMove(6 , 11);
    iPos->positionMove(7 , 40);
    // Hand
    iPos->positionMove(8 , 60);
    iPos->positionMove(9 , 30);
    iPos->positionMove(10, 30);
    iPos->positionMove(11, 5);
    iPos->positionMove(12, 0);
    iPos->positionMove(13, 15);
    iPos->positionMove(14, 0);
    iPos->positionMove(15, 40);

    // Check motion done
    bool ok = false;
    double start = Time::now();
    while (!ok && (start - Time::now() <= 10)) {
        iPos->checkMotionDone(&ok);
    }

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Set the given velocity                                           ********************************************** */
bool GraspThread::setVelocities(const int &i_type, const std::vector<double> &i_vel) {
    switch (i_type) {
        case GraspType::Stop :
            velocities.stop = i_vel;
            break;
        case GraspType::Grasp :
            velocities.grasp = i_vel;
            break;

        default:
            cerr << dbgTag << "Unknown velocity type specified. \n";
            return false;
            break;
    }

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Set the velocity for the given joint.                            ********************************************** */
bool GraspThread::setVelocity(const int &i_type, const int &i_joint, const double &i_vel) {
    if ((i_joint > 0) && (i_joint < nJointsVel)) {
        switch (i_type) {
            case GraspType::Stop :
                velocities.stop[i_joint - 8] = i_vel;
                break;
            case GraspType::Grasp :
                velocities.grasp[i_joint - 8] = i_vel;
                break;

            default:
                cerr << dbgTag << "Unknown velocity type specified. \n";
                return false;
                break;
        }
    } else {
        cerr << dbgTag << "Invalid joint specified. \n";
        return false;
    }

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Generate the mapping of each finger into the controllable joints it contains.  ******************************** */
bool GraspThread::generateJointMap(std::vector<double> &i_thresholds) {
    // FG:  This is probably one of the dirtiest pieces of code I have ever concocted. I found no clean why to map finger IDs
    //      (0,1,2,3,4) into their respective joints in the robotic kinematic chain. I chose this one.
    //
    using std::vector;

    // Initialise map
    jointMap.resize(i_thresholds.size());

    // Loop thresholds
    for (size_t i = 0; i < i_thresholds.size(); ++i) {
        vector<int> tmp;
        if (i_thresholds[i] >= 0) {
            // Create joint list
            if (i == 0) {
                tmp.push_back(11);
                tmp.push_back(12);
            } else if (i == 1) {
                tmp.push_back(13);
                tmp.push_back(14);
            } else if (i == 2) {
                tmp.push_back(15);
            } else if (i == 3) {
                tmp.push_back(15);
            } else if (i == 4) {
                tmp.push_back(8);
                tmp.push_back(9);
                tmp.push_back(10);
            } 
        }
        // Add joint list to map
        jointMap[i] = tmp;
    }

    return true;
}
/* *********************************************************************************************************************** */
