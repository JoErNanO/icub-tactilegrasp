
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
#include <deque>

#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
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

        /** Typedef representing the mapping of each finger into the controllable joints it contains. */
        typedef std::vector<std::vector<int> > FingerJointMap;

        class GraspThread : public yarp::os::RateThread {
            private:
                /* ****** Module attributes                             ****** */
                int period;
                yarp::os::ResourceFinder rf;


                /* ******* Controllers                                  ******* */
                yarp::dev::PolyDriver clientArm;
                yarp::dev::IEncoders *iEncs;
                yarp::dev::IPositionControl *iPos;
                yarp::dev::IVelocityControl *iVel;

                /** Robot arm start position. */
                yarp::sig::Vector startPos;


                /* ******* Contact detection configuration              ******* */
                /** The previous skin contacts. This is stored to avoid herratic behaviour when clocking this thread faster than the skin threads. */
                std::deque<bool> previousContacts;
                /** The touch threshold for each fingertip. */
                std::vector<double> touchThresholds;

                /* ******* Grasp configuration                          ******* */
                /** The grasp velocities. */
                GraspVelocity velocities;
                /** Number of fingers used for the grasping movement. */
                int nFingers;
                /** Total number of joints to be controlled by the velocity interface. This is set by yarp::dev::IVelocityControl::getAxes(). */
                int nJointsVel;
                /** IDs of the joints to be used for the grasping movement. */
                std::vector<int> graspJoints;
                /** Mapping of each finger into the controllable joints it contains. */
                FingerJointMap jointMap;

                
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

                /**
                 * Set velocities of all joints.
                 *
                 * \param i_type The velocity type (grasp or stop).
                 * \param i_vel The vector of joint velocities
                 * \return True upon success
                 */
                bool setVelocities(const int &i_type, const std::vector<double> &i_vel);

                /**
                 * Set the velocity of the given joint.
                 *
                 * \param i_type The velocity type (grasp or stop).
                 * \param i_joint The join to be set
                 * \param i_vel The vector of joint velocities
                 * \return True upon success
                 */
                 
                bool setVelocity(const int &i_type, const int &i_joint, const double &i_vel);
                bool openHand(void); 

            private:
                bool generateJointMap(std::vector<double> &i_thresholds);

                bool detectContact(std::deque<bool> &o_contacts);

                bool reachArm(void);
        };
    }
}

#endif

