#ifndef EXPLORATION_MODULE_H
#define EXPLORATION_MODULE_H

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>

#include <exploration.h>

class ExplorationModule:public yarp::os::RFModule
{
    yarp::os::RpcServer rpc_port;
    
    std::string robotName;
    std::string partName;
    std::string expMode;
    
    Exploration *exploration;
    
public:
    
    double getPeriod();
    bool updateModule();
    
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
    {
        std::string cmd = command.get(0).asString();
        std::string cmd1 = command.get(1).asString();
        
        yInfo() << "Received command : " << cmd << " " << cmd1;
        
        return true;
    }
    
    bool configure(yarp::os::ResourceFinder& rf)
    {
        if(!yarp::os::Network::initialized())
            yarp::os::Network::init();
        
        if(rpc_port.open("/explorationModule/rpc:i"))
            attach(rpc_port);
        
        robotName = rf.find("robot").asString();
        partName = rf.find("part").asString();
        expMode = rf.find("exploration").asString();
        
        exploration = new Exploration(robotName,partName,expMode);
        
        return true;
    }
    
    bool interruptModule();
    bool close();
};

#endif
