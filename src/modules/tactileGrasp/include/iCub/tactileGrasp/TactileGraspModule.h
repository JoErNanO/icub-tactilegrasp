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



/**
 * @ingroup icub_module
 * \defgroup TactileGraspModule TactileGraspModule
 * The TactileGraspModule is a module to perform grasping actions using tactile feedback from the fingertips, to avoid damaging grasped objects.
 * Version: 2.0
 * 
 * \section intro_sec Description
 * 
 * The Tactile Grasp is a module which perfors several types of object grasping using different levels of control and compliance.
 * Available grasping modes are:
 * - Soft grasp
 * - Crush grasp
 * 
 * 
 * <b>Soft Grasp</b> <br />
 * The Soft Grasp uses feedback from the fingertip tactile sensors to stop the grasping action once contact with an object is detected.
 * The contact thresholds are to be tuned appropriately to improve the grasping motion.
 * 
 * 
 * <b>Crush Grasp</b> <br />
 * The Crush Grasp does not use any feedback and will continue with the grasping action regardless if the fingertips are sensing anything.
 * 
 * 
 * \section lib_sec Libraries
 * YARP
 * 
 * 
 * \section parameters_sec Parameters
 * <b>Command-line Parameters</b> 
 * - -- from : Module ini configuration file.
 * 
 * <b>Configuration File Parameters </b>
 * - -- name : The module name.
 * - -- period : The module period in seconds.
 * - -- robotName : The robot name.
 * - -- whichHand : The hand to use while grasping.
 * - -- grasp : The grasping velocity for each joint &gt;= 8.
 * - -- stop : The stop velocity.
 * - -- touchThresholds : The touch threshold for each finger. Finger IDs are: 0 1 2 3 4
 *  
 * 
 * \section portsa_sec Ports Accessed
 * - /icub/skin/left_hand_comp [yarp::sig::Vector]  [default carrier:tcp]: This is the compensated skin port for the selected grasping hand.
 * - /icub/skin/right_hand_comp [yarp::sig::Vector]  [default carrier:tcp]: This is the compensated skin port for the selected grasping hand.
 * 
 * \section portsc_sec Ports Created
 * <b>RPC ports</b>
 * - /tactileGrasp/cmd:io [yarp::os::RpcServer]  [default carrier:rpc]: This is the RPC port used to control the grasping motion.
 *   - The documentation for the available RPC commands can be found in the thrift IDL implementation of the RPC server here: tactileGrasp_IDLServer. One can also type "help" in the rpc port to display the full list of commands.
 * 
 * 
 * \section conf_file_sec Configuration Files
 * - confTactileGrasp.ini : The module configuration file. 
 * 
 * 
 * \section tested_os_sec Tested OS
 * Linux
 * 
 * 
 * \section example_sec Example Instantiation of the Module
 * tactileGrasp --from confTactileGrasp.ini
 * 
 * \author Francesco Giovannini (francesco.giovannini@iit.it)
 * 
 * \copyright 
 * Copyright (C) 2014 Francesco Giovannini, iCub Facility - Istituto Italiano di Tecnologia <br />
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at src/modules/TactileGrasp/include/TactileGraspModule.h
 */

#ifndef __TACTILEGRASP_MODULE_H__
#define __TACTILEGRASP_MODULE_H__

#include "tactileGrasp_IDLServer.h"
#include "iCub/tactileGrasp/GazeThread.h"
#include "iCub/tactileGrasp/GraspThread.h"

#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>

namespace iCub {
    namespace tactileGrasp {
        class TactileGraspModule : public yarp::os::RFModule, public tactileGrasp_IDLServer {
            private:
                /* ****** Module attributes                             ****** */
                double period;

                std::string moduleName;
                std::string robotName;

                bool closing;
                
                
                /* ******* Grasp configuration                          ******* */
                GraspVelocity velocities;
         
         
                /* ****** Ports                                         ****** */
                yarp::os::RpcServer portTactileGraspRPC;


                /* ******* Threads                                      ******* */
                iCub::tactileGrasp::GazeThread *gazeThread;
                iCub::tactileGrasp::GraspThread *graspThread;

         
                /* ****** Debug attributes                              ****** */
                std::string dbgTag;

            public:
                /**
                 * Default constructor.
                 */
                TactileGraspModule();
                virtual ~TactileGraspModule();
                virtual double getPeriod();
                virtual bool configure(yarp::os::ResourceFinder &rf);
                virtual bool updateModule();
                virtual bool interruptModule();
                virtual bool close();
                virtual bool attach(yarp::os::RpcServer &source);

                // RPC Methods
                virtual bool open(void);
                virtual bool grasp(void);
                virtual bool crush(void);
                virtual bool quit(void);
                virtual bool setThreshold(const int aFinger, const double aThreshold);
        };
    }
}

#endif

