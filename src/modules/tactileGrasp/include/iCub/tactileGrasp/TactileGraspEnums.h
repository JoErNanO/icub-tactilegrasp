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



#ifndef __ICUB_TACTILEGRASP_ENUMS_H__
#define __ICUB_TACTILEGRASP_ENUMS_H__

namespace iCub {
    namespace tactileGrasp {
        /**
         * Enum to provide numeric representation of the different grasping modes.
         */
        struct GraspType {
        public:
            enum Type  {
                Stop = 0,
                Grasp = 1,
                Crush = 2
            };

            Type t_;
            GraspType(Type t) : t_(t) {}
            operator Type () const {return t_;}

        private:
           //prevent automatic conversion for any other built-in types such as bool, int, etc
           template<typename T>
           operator T () const;
        };
    }
}

#endif

