#include <exploration.h>

Exploration::Exploration(std::string& robot, std::string& part)
{
    robot_name_ = robot;
    part_name_ = part;
    motion_done_ = false;
    
    //Motor Control Device
    device_options_.put("device", "remote_controlboard");
    
    std::string local_motor_device_port_name = "/exploration/" + part_name_;
    device_options_.put("local",local_motor_device_port_name);
    
    std::string remote_motor_device_port_name = "/" + robot_name_ + "/" + part_name_;
    device_options_.put("remote",remote_motor_device_port_name);
    
    std::string log_motor_device = "[" + part_name_ + " : motor_device]";
    motor_device_.open(device_options_);
    if(!motor_device_.isValid())
    {
        yError() << log_motor_device << " Device not available";
        //yError() << "Here are the known devices:";
        //yError() <<  yarp::dev::Drivers::factory().toString().c_str();
        exit(EXIT_FAILURE);
    }
    
    motor_device_.view(pos_);
    if(pos_ == 0)
    {
        yError() << log_motor_device << " Error getting IPositionsControl interface";
        exit(EXIT_FAILURE);
    }else pos_->getAxes(&njoints_);
    
    motor_device_.view(vel_);
    if(vel_ == 0)
    {
        yError() << log_motor_device << " Error getting IVelocityControl interface";
        exit(EXIT_FAILURE);
    }
    
    motor_device_.view(encs_);
    if(encs_ == 0)
    {
        yError() << log_motor_device << " Error getting IEncoders interface";
        exit(EXIT_FAILURE);
    }
    
    motor_device_.view(ictrl_);
    if(ictrl_ == 0)
    {
        yError() << log_motor_device << " Error getting IControlMode2 interface";
        exit(EXIT_FAILURE);
    }
    
    motor_device_.view(iint_);
    if(iint_ == 0)
    {
        yError() << log_motor_device << " Error getting IInteractionMode interface";
        exit(EXIT_FAILURE);
    }
    
    motor_device_.view(iimp_);
    if(iimp_ == 0)
    {
        yError() << log_motor_device << " Error getting IImpedanceControl interface";
        exit(EXIT_FAILURE);
    }
    
    motor_device_.view(itrq_);
    if(itrq_ == 0)
    {
        yError() << log_motor_device << " Error getting ITorqueControl interface";
        exit(EXIT_FAILURE);
    }
    
    
    //Cartesian Controller Device
    cart_options_.put("device","cartesiancontrollerclient");
    
    std::string local_cartesian_device_port_name = "/exploration/cartesianController/" + part_name_; 
    cart_options_.put("local",local_cartesian_device_port_name);
    
    std::string remote_cartesian_device_port_name = "/" + robot_name_ + "/cartesianController/" + part_name_;
    yInfo() << remote_cartesian_device_port_name;
    cart_options_.put("remote",remote_cartesian_device_port_name);
    
    std::string log_cartesian_device = "[" + part_name_ + " : cartesian_device]";
    cart_device_.open(cart_options_);
    if(!cart_device_.isValid())
    {
        yError() << log_cartesian_device << " Device not available";
        //yError() << "Here are the known devices:";
        //yError() <<  yarp::dev::Drivers::factory().toString().c_str();
        exit(EXIT_FAILURE);
    }
    
    cart_device_.view(icart_);
    if(icart_ == 0)
    {
        yError() << log_cartesian_device << " Error getting ICartesianControl interface";
        exit(EXIT_FAILURE);
    }
    
    
}

bool Exploration::explore()
{
    icart_ ->getPose(position_,orientation_);
    yInfo() << part_name_ << "Pose Acquisition : " << position_[0];
   
    //Left arm acts as an anchor
    icart_->goToPoseSync(position_,orientation_);
    icart_->waitMotionDone(0.04);

}


Exploration::~Exploration()
{
    motor_device_.close();
    cart_device_.close();
}
