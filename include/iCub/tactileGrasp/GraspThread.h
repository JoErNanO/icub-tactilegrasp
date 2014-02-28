
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


#ifndef __ICUB_TACTILEGRASP_GRASPTHREAD_H__
#define __ICUB_TACTILEGRASP_GRASPTHREAD_H__

#include <iCub/tactileGrasp/TactileGraspEnums.h>

#include <string>
#include <vector>

#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/sig/Vector.h>

#include <iCub/skinDynLib/skinContactList.h>

namespace iCub {
    namespace tactileGrasp {
        /**
         * Structure containing the velocities to be used for each grasping movement.
         */
        struct GraspVelocity {
            std::vector<double> grasp;
            std::vector<double> stop;
        };

        class GraspThread : public yarp::os::RateThread {
            private:
                /* ****** Module attributes                             ****** */
                int period;
                yarp::os::ResourceFinder rf;


                /* ******* Controllers                                  ******* */
                yarp::dev::PolyDriver clientArm;
                yarp::dev::IEncoders *iEncs;
                yarp::dev::IPositionControl *iPos;
                yarp::dev::IVelocityControl2 *iVel;

                /** Robot arm start position. */
                yarp::sig::Vector startPos;


                /* ******* Grasp configuration                          ******* */
                GraspVelocity velocities;
                int nJoints;
                std::vector<double> touchThresholds;
                std::vector<int> graspJoints;

                
                /* ****** Ports                                         ****** */
                yarp::os::BufferedPort<yarp::sig::Vector> portGraspThreadInSkinComp;
                yarp::os::BufferedPort<yarp::sig::Vector> portGraspThreadInSkinRaw;
                yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> portGraspThreadInSkinContacts;
                

                /* ****** Debug attributes                              ****** */
                std::string dbgTag;

            public:
                GraspThread(const int aPeriod, const yarp::os::ResourceFinder &aRf);
                virtual ~GraspThread();

                virtual bool threadInit(void);
                virtual void run(void);
                virtual void threadRelease(void);

                bool setTouchThreshold(const int aFinger, const double aThreshold);
                bool setVelocity(const int &i_type, const std::vector<double> &i_vel);
                bool setVelocity(const int &i_type, const int &i_joint, const double &i_vel);
                bool openHand(void); 

            private:
                bool detectContact(std::vector<bool> &o_contacts);
                bool detectContact(iCub::skinDynLib::skinContactList &o_contacts);

                bool moveFingers(const std::vector<bool> &i_contacts);

                bool reachArm(void);
        };
    }
}

#endif

