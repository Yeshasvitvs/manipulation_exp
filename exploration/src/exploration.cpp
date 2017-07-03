#include <exploration.h>

Exploration::Exploration(std::string& robot)
{
    robot_name_ = robot;
    
    left_device_options_.put("device", "remote_controlboard");
    left_device_options_.put("local","/exploration/left_hand");
    std::string left_dummy = "/" + robot_name_ + "/left_arm";
    left_device_options_.put("remote",left_dummy);
    
    left_motor_device_.open(left_device_options_);
    if(!left_motor_device_.isValid())
    {
        yError() << "[left_motor_device] Device not available";
        //yError() << "Here are the known devices:";
        //yError() <<  yarp::dev::Drivers::factory().toString().c_str();
        exit;
    }
    
    left_motor_device_.view(left_pos_);
    if(left_pos_ == 0)
    {
        yError() << "[left_motor_device] Error getting IPositionsControl interface";
        exit;
    }
    left_motor_device_.view(left_vel_);
    if(left_vel_ == 0)
    {
        yError() << "[right_motor_device] Error getting IVelocityControl interface";
    }
    
    
    
    right_device_options_.put("device","remote_controlboard");
    right_device_options_.put("local","/exploration/right_hand");
    std::string right_dummy = "/" + robot_name_ + "/right_arm";
    right_device_options_.put("remote",right_dummy);
    
    right_motor_device_.open(right_device_options_);
    if(!right_motor_device_.isValid())
    {
        yError() << "[right_motor_device] Device not available";
        //yError() << "Here are the known devices:";
        //yError() <<  yarp::dev::Drivers::factory().toString().c_str();
        exit;
    }
    
    right_motor_device_.view(right_pos_);
    if(right_pos_ == 0)
    {
        yError() << "[right_motor_device] Error getting IPositionsControl interface";
        exit;
    }
    right_motor_device_.view(right_vel_);
    if(right_vel_ == 0)
    {
        yError() << "[right_motor_device] Error getting IVelocityControl interface";
    }
}

Exploration::~Exploration()
{
    left_motor_device_.close();
    right_motor_device_.close();
}
