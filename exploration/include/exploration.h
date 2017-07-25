#ifndef EXPLORATION_H
#define EXPLORATION_H

#include <iostream>
#include <string.h>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IImpedanceControl.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/sig/Vector.h>

class Exploration
{
  private:
      
      std::string robot_name_;
      std::string part_name_;
      std::string exploration_mode_;

      int njoints_;
      yarp::sig::Vector encoders_;
      bool motion_done_;
      
      yarp::sig::Vector stiff_;
      yarp::sig::Vector damp_;
      yarp::sig::Vector torques_;
      
  public:
      
      yarp::os::Property device_options_;
      yarp::dev::PolyDriver motor_device_;
      
      yarp::dev::IPositionControl  *pos_;
      yarp::dev::IVelocityControl  *vel_;
      yarp::dev::IEncoders         *encs_;
      yarp::dev::IControlMode2     *ictrl_;
      yarp::dev::IInteractionMode  *iint_;
      yarp::dev::IImpedanceControl *iimp_;
      yarp::dev::ITorqueControl    *itrq_;
      
      yarp::os::Property cart_options_;
      yarp::dev::PolyDriver cart_device_;
      
      yarp::dev::ICartesianControl *icart_;
      
      yarp::sig::Vector position_, orientation_;
      
      bool anchor();
      bool explore();
      
      Exploration(std::string&, std::string&, std::string&);
      ~Exploration();
};

#endif