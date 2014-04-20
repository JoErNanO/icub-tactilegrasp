// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_tactileGrasp_IDLServer
#define YARP_THRIFT_GENERATOR_tactileGrasp_IDLServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class tactileGrasp_IDLServer;


/**
 * tactileGrasp_IDLServer
 * IDL Interface to \ref tactileGrasp services.
 */
class tactileGrasp_IDLServer : public yarp::os::Wire {
public:
  tactileGrasp_IDLServer() { yarp().setOwner(*this); }
/**
 * Opens the robot hand.
 * @return true/false on success/failure
 */
  virtual bool open();
/**
 * Grasp an object using feedback from the fingertips tactile sensors.
 * The grasping movement is stopped upon contact detection.
 * @return true/false on success/failure.
 */
  virtual bool grasp();
/**
 * Grasp object without using the feedback from the fingertips tactile sensors.
 * The grasping movement is not controlled and the object is therefore crushed.
 * @return true/false on success/failure.
 */
  virtual bool crush();
/**
 * Quit the module.
 * @return true/false on success/failure.
 */
  virtual bool quit();
/**
 * Set the touch threshold.
 * @return true/false on success/failure.
 */
  virtual bool setThreshold(const int32_t aFinger, const double aThreshold);
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

