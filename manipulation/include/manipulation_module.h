#ifndef MANIPULATION_MODULE_H
#define MANIPULATION_MODULE_H

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>

#include <manipulation.h>

class ManipulationModule:public yarp::os::RFModule
{
    yarp::os::RpcServer rpc_port;
    
    std::string robotName;
    
    Manipulation *manipulation;
    
public:
    
    cv::Mat image;
    
    double getPeriod();
    bool updateModule();
    
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
    {
        std::string cmd = command.get(0).asString();
        std::string cmd1 = command.get(1).asString();
        
        yInfo() << "Received command : " << cmd << " " << cmd1;
        
        if (cmd == "quit") return false; 
        else
        {
            //set log data flag
            if(cmd == "log")
            {
                if(cmd1 == "start")
                {
                    if(manipulation->log_data_ == true)
                        reply.addString("Data Logging: [INPROGRESS]");
                    
                    else
                    {
                        if(command.size() < 3)
                            reply.addString("Incorrect arguments! Correct Usage : log start filename");
                        else
                        {
                            std::string cmd2 = command.get(2).asString();
                            cmd2 = manipulation->data_directory_ + "/" + cmd2;
                            std::cout << "filename : " << cmd2 << std::endl;
                            manipulation->log_data_ = true;
                            manipulation->file_name_.open(cmd2);
                            std::string dummy = "Data Logging : [START] to " + cmd2;
                            reply.addString(dummy);
                        }
                    }
                }
                if(cmd1 == "stop")
                {
                    if(manipulation->log_data_ == false)
                        reply.addString("Data Logging : [NOT IN PROGRESS]");
                    else
                    {
                        manipulation->log_data_ = false;
                        manipulation->file_name_.close();
                        reply.addString("Data Logging : [STOP]");
                    } 
                }
            }
            else if(cmd == "applyWrenches")
            {
                manipulation->applyWrenches();
                reply.addString("Applying external wrenches [OK]");
            }
            else reply.addString("UNKNOW COMMAND");
        }
        return true;
    }
    
    bool configure(yarp::os::ResourceFinder& rf)
    {
        if(!yarp::os::Network::initialized())
            yarp::os::Network::init();
        
        if(rpc_port.open("/manipulationModule/rpc:i"))
            attach(rpc_port);
        
        robotName = rf.find("robot").asString();
        
        manipulation = new Manipulation(robotName);
        
        return true;
    }
    
    bool interruptModule();
    bool close();
};

#endif