#include <exploration_module.h>

int main(int argc,char** argv)
{
    yarp::os::Network yarp;
    
    ExplorationModule explrModule;
    yarp::os::ResourceFinder rf;
    if(argc < 2)
    {
        yError() << "Manipulation module error, configuration file not passed as an argument!";
        return 1;
    }
    else rf.configure(argc,argv);
    
    if(!explrModule.runModule(rf))
    {
        std::cerr << "Manipulation Module failed to start!" << std::endl;
        return 1;
    }
       
    return 0;
}