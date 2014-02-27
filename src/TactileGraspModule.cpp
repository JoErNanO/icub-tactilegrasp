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



#include "iCub/tactileGrasp/TactileGraspModule.h"

#include <iostream>

using iCub::tactileGrasp::TactileGraspModule;

using std::cerr;
using std::cout;

using yarp::os::ResourceFinder;
using yarp::os::Value;
using yarp::os::Bottle;


/* *********************************************************************************************************************** */
/* ******* Constructor                                                      ********************************************** */   
TactileGraspModule::TactileGraspModule() 
    : RFModule(), tactileGrasp_IDLServer() {
        closing = false;

        dbgTag = "TactileGraspModule: ";
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Destructor                                                       ********************************************** */   
TactileGraspModule::~TactileGraspModule() {}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Get Period                                                       ********************************************** */   
double TactileGraspModule::getPeriod() { return period; }
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Configure module                                                 ********************************************** */   
bool TactileGraspModule::configure(ResourceFinder &rf) {
    using std::string;

    cout << dbgTag << "Starting. \n";

    /* ****** Configure the Module                            ****** */
    // Get resource finder and extract properties
    moduleName = rf.check("name", Value("contactDetector"), "The module name.").asString().c_str();
    period = rf.check("period", 1.0).asDouble();
    robotName = rf.check("robot", Value("icub"), "The robot name.").asString().c_str();


    /* ******* Open ports                                       ******* */
    portTactileGraspRPC.open("/TactileGrasp/cmd:io");
    attach(portTactileGraspRPC);


    /* ******* Get parameters from rf                           ******* */
    string whichHand = rf.check("whichHand", Value("right"), "The hand to be used for the grasping.").asString().c_str();


    /* ******* Threads                                          ******* */
    // Gaze thread
    gazeThread = new GazeThread(100, robotName, whichHand);
    if (!gazeThread->start()) {
        cout << dbgTag << "Could not start the gaze thread. \n";
        return false;
    }
    // Grasp hread
    graspThread = new GraspThread(100, robotName, whichHand);
    if (!graspThread->start()) {
        cout << dbgTag << "Could not start the grasp thread. \n";
        return false;
    }
    graspThread->suspend();
    
    cout << dbgTag << "Started correctly. \n";

    return true;
}
/* *********************************************************************************************************************** */


bool TactileGraspModule::attach(yarp::os::RpcServer &source) {
    return this->yarp().attachAsServer(source);
}

/* *********************************************************************************************************************** */
/* ******* Update    module                                                 ********************************************** */   
bool TactileGraspModule::updateModule() { 
    return !closing; 
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Interrupt module                                                 ********************************************** */   
bool TactileGraspModule::interruptModule() {
    cout << dbgTag << "Interrupting. \n";
    
    // Interrupt ports
    portTactileGraspRPC.interrupt();

    cout << dbgTag << "Interrupted correctly. \n";

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Close module                                                     ********************************************** */   
bool TactileGraspModule::close() {
    cout << dbgTag << "Closing. \n";
    
    // Stop threads
    gazeThread->stop();
    graspThread->stop();

    // Close ports
    portTactileGraspRPC.close();

    cout << dbgTag << "Closed. \n";
    
    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Respond to rpc calls                                             ********************************************** */   
bool TactileGraspModule::respond(const Bottle &command, Bottle &reply) {

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Open hand                                                    ********************************************** */
bool TactileGraspModule::open(void) {
    graspThread->suspend();

    return graspThread->openHand();
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Grasp object                                                 ********************************************** */
bool TactileGraspModule::grasp(void) {
    graspThread->resume();

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Crush object                                                 ********************************************** */
bool TactileGraspModule::crush(void) {
    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Quit module                                                  ********************************************** */
bool TactileGraspModule::quit(void) {
    return closing = true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* Set touch threshold.                                             ********************************************** */
bool TactileGraspModule::setThreshold(const int aFinger, const double aThreshold) {
    return graspThread->setTouchThreshold(aFinger, aThreshold);
}
/* *********************************************************************************************************************** */
