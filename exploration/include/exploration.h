#ifndef EXPLORATION_H
#define EXPLORATION_H

#include <iostream>
#include <string.h>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

class Exploration
{
  private:
      
      std::string robot_name_;
      
  
  public:
      yarp::os::Property left_device_options_;
      yarp::dev::PolyDriver left_motor_device_;
      yarp::dev::IPositionControl *left_pos_;
      yarp::dev::IVelocityControl *left_vel_;
      
      yarp::os::Property right_device_options_;
      yarp::dev::PolyDriver right_motor_device_;
      yarp::dev::IPositionControl *right_pos_;
      yarp::dev::IVelocityControl *right_vel_;
      
      Exploration(std::string&);
      ~Exploration();
};

#endif