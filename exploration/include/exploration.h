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
      int left_njoints_, right_njoints_;
      yarp::sig::Vector left_encoders_,right_encoders_;
      bool left_motion_done_, right_motion_done_;
  
  public:
      yarp::os::Property left_device_options_;
      yarp::dev::PolyDriver left_motor_device_;
      
      yarp::dev::IPositionControl  *left_pos_;
      yarp::dev::IVelocityControl  *left_vel_;
      yarp::dev::IEncoders         *left_encs_;
      yarp::dev::IControlMode2     *left_ictrl_;
      yarp::dev::IInteractionMode  *left_iint_;
      yarp::dev::IImpedanceControl *left_iimp_;
      yarp::dev::ITorqueControl    *left_itrq_;
      
      yarp::os::Property left_cart_options_;
      yarp::dev::PolyDriver left_cart_device_;
      yarp::dev::ICartesianControl *left_icart_;
      
      yarp::os::Property right_device_options_;
      yarp::dev::PolyDriver right_motor_device_;
      
      yarp::dev::IPositionControl  *right_pos_;
      yarp::dev::IVelocityControl  *right_vel_;
      yarp::dev::IEncoders         *right_encs_;
      yarp::dev::IControlMode2     *right_ictrl_;
      yarp::dev::IInteractionMode  *right_iint_;
      yarp::dev::IImpedanceControl *right_iimp_;
      yarp::dev::ITorqueControl    *right_itrq_;
      
      yarp::os::Property right_cart_options_;
      yarp::dev::PolyDriver right_cart_device_;
      yarp::dev::ICartesianControl *right_icart_;
      
      yarp::sig::Vector left_position_, left_orientation_;
      yarp::sig::Vector right_position_, right_orientation_;
      
      bool explore();
      
      Exploration(std::string&);
      ~Exploration();
};

#endif