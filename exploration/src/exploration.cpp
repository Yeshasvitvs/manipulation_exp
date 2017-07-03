#include <exploration.h>

Exploration::Exploration(std::string& robot)
{
    robot_name_ = robot;
    left_motion_done_ = false;
    right_motion_done_ = false;
    
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
        exit(EXIT_FAILURE);
    }
    
    left_motor_device_.view(left_pos_);
    if(left_pos_ == 0)
    {
        yError() << "[left_motor_device] Error getting IPositionsControl interface";
        exit(EXIT_FAILURE);
    }else left_pos_->getAxes(&left_njoints_);
    left_motor_device_.view(left_vel_);
    if(left_vel_ == 0)
    {
        yError() << "[left_motor_device] Error getting IVelocityControl interface";
        exit(EXIT_FAILURE);
    }
    left_motor_device_.view(left_encs_);
    if(left_encs_ == 0)
    {
        yError() << "[left_motor_device] Error getting IEncoders interface";
        exit(EXIT_FAILURE);
    }
    left_motor_device_.view(left_ictrl_);
    if(left_ictrl_ == 0)
    {
        yError() << "[left_motor_device] Error getting IControlMode2 interface";
        exit(EXIT_FAILURE);
    }
    left_motor_device_.view(left_iint_);
    if(left_iint_ == 0)
    {
        yError() << "[left_motor_device] Error getting IInteractionMode interface";
        exit(EXIT_FAILURE);
    }
    left_motor_device_.view(left_iimp_);
    if(left_iimp_ == 0)
    {
        yError() << "[left_motor_device] Error getting IImpedanceControl interface";
        exit(EXIT_FAILURE);
    }
    left_motor_device_.view(left_itrq_);
    if(left_itrq_ == 0)
    {
        yError() << "[left_motor_device] Error getting ITorqueControl interface";
        exit(EXIT_FAILURE);
    }
    
    left_cart_options_.put("device","cartesiancontrollerclient");
    left_cart_options_.put("local","/exploration/cartesiancontroller/left_arm");
    left_dummy = "/" + robot_name_ + "/cartesiancontroller/left_arm";
    left_cart_options_.put("remote",left_dummy);
    
    left_cart_device_.open(left_cart_options_);
    if(!left_cart_device_.isValid())
    {
        yError() << "[left_cart_device_] Device not available";
        //yError() << "Here are the known devices:";
        //yError() <<  yarp::dev::Drivers::factory().toString().c_str();
        exit(EXIT_FAILURE);
    }
    left_cart_device_.view(left_icart_);
    if(left_icart_ == 0)
    {
        yError() << "[left_cart_device] Error getting ICartesianControl interface";
        exit(EXIT_FAILURE);
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
        exit(EXIT_FAILURE);
    }
    
    right_motor_device_.view(right_pos_);
    if(right_pos_ == 0)
    {
        yError() << "[right_motor_device] Error getting IPositionsControl interface";
        exit(EXIT_FAILURE);
    }right_pos_->getAxes(&right_njoints_);
    right_motor_device_.view(right_vel_);
    if(right_vel_ == 0)
    {
        yError() << "[right_motor_device] Error getting IVelocityControl interface";
        exit(EXIT_FAILURE);
    }
    right_motor_device_.view(right_encs_);
    if(right_encs_ == 0)
    {
        yError() << "[right_motor_device] Error getting IEncoders interface";
        exit(EXIT_FAILURE);
    }
    right_motor_device_.view(right_ictrl_);
    if(right_ictrl_ == 0)
    {
        yError() << "[right_motor_device] Error getting IControlMode2 interface";
        exit(EXIT_FAILURE);
    }
    right_motor_device_.view(right_iint_);
    if(right_iint_ == 0)
    {
        yError() << "[right_motor_device] Error getting IInteractionMode interface";
        exit(EXIT_FAILURE);
    }
    right_motor_device_.view(right_iimp_);
    if(right_iimp_ == 0)
    {
        yError() << "[right_motor_device] Error getting IImpedanceControl interface";
        exit(EXIT_FAILURE);
    }
    right_motor_device_.view(right_itrq_);
    if(right_itrq_ == 0)
    {
        yError() << "[right_motor_device] Error getting ITorqueControl interface";
        exit(EXIT_FAILURE);
    }
    
    right_cart_options_.put("device","cartesiancontrollerclient");
    right_cart_options_.put("local","/exploration/cartesiancontroller/right_arm");
    right_dummy = "/" + robot_name_ + "/cartesiancontroller/right_arm";
    right_cart_options_.put("remote",right_dummy);
    
    right_cart_device_.open(right_cart_options_);
    if(!right_cart_device_.isValid())
    {
        yError() << "[right_cart_device] Device not available";
        //yError() << "Here are the known devices:";
        //yError() <<  yarp::dev::Drivers::factory().toString().c_str();
        exit(EXIT_FAILURE);
    }
    right_cart_device_.view(right_icart_);
    if(right_icart_ == 0)
    {
        yError() << "[right_cart_device] Error getting ICartesianControl interface";
        exit(EXIT_FAILURE);
    }
    
}

bool Exploration::explore()
{
    yInfo() << "Pose Acquisition";
    left_icart_ ->getPose(left_position_,left_orientation_);
    right_icart_ ->getPose(right_position_,right_orientation_);
    
    yInfo() << "iCub left arm anchoring";
    //Left arm acts as an anchor
    left_icart_->goToPoseSync(left_position_,left_orientation_);
    left_icart_->waitMotionDone(0.04);

}


Exploration::~Exploration()
{
    left_motor_device_.close();
    right_motor_device_.close();
}
