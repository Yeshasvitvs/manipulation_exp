#include <manipulation_module.h>

int main(int argc,char** argv)
{
    yarp::os::Network yarp;
    
    ManipulationModule maniModule;
    yarp::os::ResourceFinder rf;
    if(argc < 2)
    {
        yError() << "Manipulation module error, configuration file not passed as an argument!";
        return 1;
    }
    else rf.configure(argc,argv);
    
    if(!maniModule.runModule(rf))
    {
        std::cerr << "Manipulation Module failed to start!" << std::endl;
        return 1;
    }
       
    return 0;
}