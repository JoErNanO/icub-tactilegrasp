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
 * \defgroup icub_MyModule TactileGraspModule
 * 
 * The TactileGraspModule is a
 * 
 * \section intro_sec Description
 * Description here...
 * 
 * 
 * \section lib_sec Libraries
 * YARP
 * 
 * 
 * \section parameters_sec Parameters
 * <b>Command-line Parameters</b> 
 * <b>Configuration File Parameters </b>
 *  
 * 
 * \section portsa_sec Ports Accessed
 * 
 * 
 * \section portsc_sec Ports Created
 * <b>Output ports </b>
 * <b>Input ports</b>
 * 
 * 
 * \section in_files_sec Input Data Files
 * 
 * 
 * \section out_data_sec Output Data Files
 * 
 *  
 * \section conf_file_sec Configuration Files
 * 
 * 
 * \section tested_os_sec Tested OS
 * 
 * 
 * \section example_sec Example Instantiation of the Module
 * 
 * 
 * \author Francesco Giovannini (francesco.giovannini@iit.it)
 * 
 * Copyright (C) 2014 Francesco Giovannini, iCub Facility - Istituto Italiano di Tecnologia
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at .... .
 */

#ifndef __TACTILEGRASP_MODULE_H__
#define __TACTILEGRASP_MODULE_H__

#include "iCub/tactileGrasp/GazeThread.h"
#include "iCub/tactileGrasp/GraspThread.h"

#include <string>

#include <yarp/os/RFModule.h>

namespace iCub {
    namespace tactileGrasp {
        class TactileGraspModule : public yarp::os::RFModule {
            private:
                /* ****** Module attributes                             ****** */
                double period;

                std::string moduleName;
                std::string robotName;
         
                /* ****** Ports                                         ****** */

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
                virtual bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);
        };
    }
}

#endif

