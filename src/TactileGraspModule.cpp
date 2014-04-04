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


#define TACTILEGRASP_DEBUG 1 

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
    using yarp::os::Property;

    cout << dbgTag << "Starting. \n";

    /* ****** Configure the Module                            ****** */
    // Get resource finder and extract properties
    moduleName = rf.check("name", Value("contactDetector"), "The module name.").asString().c_str();
    period = rf.check("period", 1.0).asDouble();


    /* ******* Open ports                                       ******* */
    portTactileGraspRPC.open("/TactileGrasp/cmd:io");
    attach(portTactileGraspRPC);


    /* ******* Get parameters from rf                           ******* */
    // Build velocities
    Bottle &confVelocity = rf.findGroup("velocity");
    if (!confVelocity.isNull()) {
        // Grasp velocities
        Bottle *confVelGrasp = confVelocity.find("grasp").asList();
        if (confVelGrasp->size() > 0) {
            for (int i = 0; i < confVelGrasp->size(); ++i) {
                velocities.grasp.push_back(confVelGrasp->get(i).asDouble());
            }
        } else {
            cerr << dbgTag << "No grasp velocities were found in the specified configuration file. \n";
            return false;
        }
        
        // Stop velocities
        velocities.stop.resize(velocities.grasp.size() , confVelocity.check("stop", Value(0.0)).asDouble());
    } else {
        cerr << dbgTag << "Could not find the velocities parameter group [velocity] in the given configuration file. \n";
        return false;
    }

#ifdef TACTILEGRASP_DEBUG
    cout << "\n";
    cout << dbgTag << "Configured velocities: \n";
    for (size_t i = 0; i < velocities.grasp.size(); ++i) {
        cout << dbgTag << "\t Joint " << i + 8 << ":\t" << velocities.grasp[i] << "\t" << velocities.stop[i] << "\n";
    }
    cout << "\n";
#endif

    /* ******* Threads                                          ******* */
    // Gaze thread
    gazeThread = new GazeThread(100, rf);
    if (!gazeThread->start()) {
        cout << dbgTag << "Could not start the gaze thread. \n";
        return false;
    }
    // Grasp hread
    graspThread = new GraspThread(20, rf);
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
/* ******* RPC Open hand                                                    ********************************************** */
bool TactileGraspModule::open(void) {
    graspThread->suspend();

    return graspThread->openHand();
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Grasp object                                                 ********************************************** */
bool TactileGraspModule::grasp(void) {
    graspThread->setVelocities(GraspType::Grasp, velocities.grasp);
    graspThread->setVelocities(GraspType::Stop, velocities.stop);       // Set velocity to stop upon contact detection
    graspThread->resume();

    return true;
}
/* *********************************************************************************************************************** */


/* *********************************************************************************************************************** */
/* ******* RPC Crush object                                                 ********************************************** */
bool TactileGraspModule::crush(void) {
    graspThread->setVelocities(GraspType::Grasp, velocities.grasp);
    graspThread->setVelocities(GraspType::Stop, velocities.grasp);      // Set velocity to crush object
    graspThread->resume();

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
