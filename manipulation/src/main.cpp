#include <manipulation_module.h>

int main(int argc,char** argv)
{
    yarp::os::Network yarp;
    
    ManipulationModule maniModule;
    yarp::os::ResourceFinder rf;
    //rf.setVerbose(true);
    rf.configure(argc,argv);
       
    maniModule.runModule(rf);
    
    return 0;
    
}